#include "remote_state.h"
#include "application_config.h"
#include <string.h>

static RemoteState_t remote_state; // 遥控任务发布给底盘任务的状态快照。

typedef struct
{
    bool turnaround_wheel_armed; // 拨轮回中后允许再次调头。
    uint32_t turnaround_request_count; // 调头事件累计数。
    bool spin_switch_seen; // 已记录小陀螺内右拨杆初始档位。
    uint8_t spin_previous_switch; // 小陀螺内上一帧右拨杆档位。
    bool spin_armed; // 小陀螺内右拨杆已换档。
    bool spin_wheel_ready; // 拨轮回中后允许再次切换。
    bool spin_selected; // 拨轮选择小陀螺模式。
    uint8_t previous_left_switch; // 左拨杆变化时取消小陀螺。
} RemotePhysicalLatch_t;

typedef struct
{
    bool seen; // 上线首帧只记录键位。
    bool active; // V 键选择键鼠控制。
    uint16_t previous_key; // 上一帧键盘位图。
    RemoteMode_t mode; // Z/X 选择的基础模式枚举。
    bool spin_active; // G 键选择小陀螺。
    bool spin_g_released; // G 先松开才接受按下沿。
    bool q_released; // Q 先松开才接受调头输入。
} RemoteKeyboardLatch_t;

typedef struct
{
    RemotePhysicalLatch_t physical; // 遥控拨杆与拨轮的边沿锁存。
    RemoteKeyboardLatch_t keyboard; // 键鼠模式与按键边沿锁存。
} RemoteStateMachine_t;

static RemoteStateMachine_t machine; // 仅状态机任务修改，不直接暴露给底盘任务。

// 将左拨杆档位映射为模式枚举；非法档位保持失能。
static RemoteMode_t RemoteState_MapSwitchMode(uint8_t left_switch)
{
    if (left_switch == chassis_config.follow_switch_position)
    { return REMOTE_MODE_FOLLOW; }
    if (left_switch == CHASSIS_MECHANICAL_SWITCH_POSITION ||
        left_switch == CHASSIS_MECHANICAL_DOWN_POSITION)
    { return REMOTE_MODE_MECHANICAL; }
    return REMOTE_MODE_DISABLED;
}

// 键鼠：WASD 移动，Z/X 选模式，G 切换小陀螺，Q 调头。
// 鼠标输入映射到现有摇杆通道。
static int16_t RemoteState_ClampChannel(int32_t value)
{
    if (value > 660) { return 660; }
    if (value < -660) { return -660; }
    return (int16_t)value;
}

static void RemoteState_MapKeyboard(RemoteState_t *next,
                                    const RC_ctrl_t *control,
                                    uint16_t pressed)
{
    uint16_t key = control->key;
    int16_t move = keyboard_sensitivity_config.move_raw;
    int32_t mouse_x = control->mouse.x;
    int32_t mouse_y = control->mouse.y;
    if (pressed & RC_KEY_Z)
    {
        machine.keyboard.mode = REMOTE_MODE_FOLLOW;
        machine.keyboard.spin_active = false;
    }
    else if (pressed & RC_KEY_X)
    {
        machine.keyboard.mode = REMOTE_MODE_MECHANICAL;
        machine.keyboard.spin_active = false;
    }
    if (!(key & RC_KEY_G)) { machine.keyboard.spin_g_released = true; }
    if (machine.keyboard.spin_g_released && (pressed & RC_KEY_G))
    { machine.keyboard.spin_active = !machine.keyboard.spin_active; }

    if (key & RC_KEY_CTRL) { move = keyboard_sensitivity_config.slow_raw; }
    else if (key & RC_KEY_SHIFT) { move = keyboard_sensitivity_config.sprint_raw; }
    memset(next->input.channel, 0, sizeof(next->input.channel));
    next->input.channel[3] = ((key & RC_KEY_W) ? move : 0) -
                       ((key & RC_KEY_S) ? move : 0);
    next->input.channel[2] = ((key & RC_KEY_D) ? move : 0) -
                       ((key & RC_KEY_A) ? move : 0);
    if (control->mouse.right && keyboard_sensitivity_config.aim_divisor > 0)
    {
        mouse_x /= keyboard_sensitivity_config.aim_divisor;
        mouse_y /= keyboard_sensitivity_config.aim_divisor;
    }
    next->input.channel[0] = RemoteState_ClampChannel(
        mouse_x * keyboard_sensitivity_config.mouse_yaw_gain);
    next->input.channel[1] = RemoteState_ClampChannel(
        -mouse_y * keyboard_sensitivity_config.mouse_pitch_gain);
    if (!(key & RC_KEY_Q)) { machine.keyboard.q_released = true; }
    if (machine.keyboard.q_released && (key & RC_KEY_Q))
    { next->input.channel[4] = -660; }

    next->mode.chassis = machine.keyboard.spin_active ? REMOTE_MODE_SPIN : machine.keyboard.mode;
    next->safety.spin_enabled = machine.keyboard.spin_active;
}

void RemoteState_Init(void)
{
    RemoteState_Update(NULL, false);
}

void RemoteState_Update(const RC_ctrl_t *control, bool online)
{
    RemoteState_t next;
    uint32_t primask;

    memset(&next, 0, sizeof(next));
    next.mode.chassis = REMOTE_MODE_DISABLED;
    if (online && control != NULL)
    {
        uint16_t pressed = 0U;

        next.safety.online = true;
        memcpy(next.input.channel, control->rc.ch, sizeof(next.input.channel));
        if (machine.keyboard.seen)
        { pressed = control->key & (uint16_t)~machine.keyboard.previous_key; }
        else
        { machine.keyboard.seen = true; }
        machine.keyboard.previous_key = control->key;

        if (pressed & RC_KEY_V)
        {
            machine.keyboard.active = !machine.keyboard.active;
            machine.keyboard.spin_g_released = false;
            machine.keyboard.spin_active = false;
            machine.keyboard.q_released = false;
            machine.keyboard.mode = REMOTE_MODE_DISABLED;
        }

        next.mode.chassis = RemoteState_MapSwitchMode(control->rc.s[0]);
        if (next.mode.chassis == REMOTE_MODE_DISABLED)
        {
            next.safety.online = false;
            memset(next.input.channel, 0, sizeof(next.input.channel));
        }

        if (next.safety.online)
        {
            int16_t wheel = control->rc.ch[4];
            if (control->rc.s[0] != machine.physical.previous_left_switch)
            {
                machine.physical.previous_left_switch = control->rc.s[0];
                machine.physical.spin_selected = false;
                machine.physical.spin_wheel_ready = false;
            }
            if (machine.keyboard.active)
            {
                machine.physical.spin_selected = false;
                machine.physical.spin_wheel_ready = false;
            }
            else
            {
                if (wheel >= -chassis_config.spin_wheel_rearm_raw &&
                    wheel <= chassis_config.spin_wheel_rearm_raw)
                { machine.physical.spin_wheel_ready = true; }
                else if (machine.physical.spin_wheel_ready &&
                         wheel >= chassis_config.spin_wheel_trigger_raw)
                {
                    machine.physical.spin_wheel_ready = false;
                    if (machine.physical.spin_selected)
                    { machine.physical.spin_selected = false; }
                    else if (control->rc.s[1] == RC_SW_DOWN)
                    { machine.physical.spin_selected = true; }
                }
                if (machine.physical.spin_selected)
                { next.mode.chassis = REMOTE_MODE_SPIN; }
            }
        }

        if (next.safety.online && machine.keyboard.active)
        {
            if (machine.keyboard.mode == REMOTE_MODE_DISABLED)
            { machine.keyboard.mode = next.mode.chassis; }
            RemoteState_MapKeyboard(&next, control, pressed);
        }
        next.input.keyboard_active = next.safety.online && machine.keyboard.active;

        if (next.safety.online && !machine.keyboard.active &&
            next.mode.chassis == REMOTE_MODE_SPIN &&
            (control->rc.s[1] == RC_SW_UP ||
             control->rc.s[1] == RC_SW_MID ||
             control->rc.s[1] == RC_SW_DOWN))
        {
            if (machine.physical.spin_switch_seen &&
                control->rc.s[1] != machine.physical.spin_previous_switch)
            {
                machine.physical.spin_armed = true;
            }
            machine.physical.spin_switch_seen = true;
            machine.physical.spin_previous_switch = control->rc.s[1];
            next.safety.spin_enabled = machine.physical.spin_armed &&
                control->rc.s[1] == CHASSIS_SPIN_SWITCH_1_POSITION;
        }
        else
        {
            // 离开模式、断联或非法档位后，下次进入必须重新拨动。
            machine.physical.spin_switch_seen = false;
            machine.physical.spin_previous_switch = 0U;
            machine.physical.spin_armed = false;
        }

        if (next.safety.online)
        {
            if (next.input.channel[4] > -chassis_config.turn_wheel_rearm_raw)
            {
                machine.physical.turnaround_wheel_armed = true;
            }
            else if (machine.physical.turnaround_wheel_armed &&
                     next.input.channel[4] <= -chassis_config.turn_wheel_trigger_raw)
            {
                machine.physical.turnaround_wheel_armed = false;
                machine.physical.turnaround_request_count++;
            }
        }
    }
    if (!next.safety.online)
    {
        machine.physical.turnaround_wheel_armed = false;
        machine.physical.spin_switch_seen = false;
        machine.physical.spin_previous_switch = 0U;
        machine.physical.spin_armed = false;
        machine.physical.spin_wheel_ready = false;
        machine.physical.spin_selected = false;
        machine.physical.previous_left_switch = 0U;
        machine.keyboard.seen = false;
        machine.keyboard.previous_key = 0U;
        machine.keyboard.active = false;
        machine.keyboard.mode = REMOTE_MODE_DISABLED;
        machine.keyboard.spin_g_released = false;
        machine.keyboard.spin_active = false;
        machine.keyboard.q_released = false;
    }
    next.event.turnaround_request_count = machine.physical.turnaround_request_count;

    primask = __get_PRIMASK();
    __disable_irq();
    remote_state = next;
    __set_PRIMASK(primask);
}

void RemoteState_Get(RemoteState_t *state)
{
    uint32_t primask;

    if (state == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    *state = remote_state;
    __set_PRIMASK(primask);
}
