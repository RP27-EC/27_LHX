#include "remote_state.h"
#include "application_config.h"
#include <string.h>

static RemoteState_t remote_state; // 通信任务发布给各控制任务的遥控状态。

typedef struct
{
    bool shoot_switch_seen; // 已记录右拨杆的初始档位。
    uint8_t shoot_previous_switch; // 上一帧右拨杆档位。
    bool shoot_armed; // 右拨杆换档后允许发射。
    bool spin_switch_seen; // 已记录小陀螺内右拨杆的初始档位。
    uint8_t spin_previous_switch; // 小陀螺内上一帧右拨杆档位。
    bool spin_armed; // 小陀螺内右拨杆已换档。
    bool spin_wheel_ready; // 拨轮回中后允许再次切换。
    bool spin_selected; // 拨轮选择了小陀螺模式。
    bool spin_last_mode; // 上一帧是否为小陀螺模式。
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
    bool f_released; // F 先松开才接受摩擦轮开关。
    bool friction_on; // F 按下沿锁存摩擦轮状态。
    bool mouse_released; // 左键先松开才允许发射。
    bool mouse_press_active; // 正在记录一次有效左键按下。
    bool mouse_continuous; // 本次按下已进入连发。
    uint32_t mouse_press_ms; // 左键按下时间，ms。
    uint32_t single_request_count; // 短按单发事件累计数。
    uint32_t lift_request_count; // B 键升降切换事件累计数。
} RemoteKeyboardLatch_t;

typedef struct
{
    RemotePhysicalLatch_t physical; // 遥控拨杆与拨轮的边沿锁存。
    RemoteKeyboardLatch_t keyboard; // 键鼠模式与按键边沿锁存。
} RemoteStateMachine_t;

static RemoteStateMachine_t machine; // 仅状态机任务修改，不直接暴露给控制任务。

// 将左拨杆档位映射为模式枚举；非法档位保持失能。
static RemoteMode_t RemoteState_MapSwitchMode(uint8_t left_switch)
{
    if (left_switch == COMM_RC_SW_DOWN || left_switch == COMM_RC_SW_MID)
    { return REMOTE_MODE_MECHANICAL; }
    if (left_switch == COMM_RC_SW_UP) { return REMOTE_MODE_FOLLOW; }
    return REMOTE_MODE_DISABLED;
}

// 键鼠：Z/X 选模式，G 切换小陀螺，B 在机械模式切换升降目标。
// F/左键控制发射；小陀螺中也可发射。
static int16_t RemoteState_ClampChannel(int32_t value)
{
    if (value > 660) { return 660; }
    if (value < -660) { return -660; }
    return (int16_t)value;
}

static void RemoteState_MapKeyboard(RemoteState_t *next,
                                    const Communication_RcControl_t *control,
                                    uint16_t pressed)
{
    uint16_t key = control->key;
    int16_t move = keyboard_sensitivity_config.move_raw;
    int32_t mouse_x = control->mouse.x;
    int32_t mouse_y = control->mouse.y;
    if (pressed & COMM_RC_KEY_Z)
    {
        machine.keyboard.mode = REMOTE_MODE_FOLLOW;
        machine.keyboard.spin_active = false;
    }
    else if (pressed & COMM_RC_KEY_X)
    {
        machine.keyboard.mode = REMOTE_MODE_MECHANICAL;
        machine.keyboard.spin_active = false;
    }
    if (!(key & COMM_RC_KEY_G)) { machine.keyboard.spin_g_released = true; }
    if (machine.keyboard.spin_g_released && (pressed & COMM_RC_KEY_G))
    { machine.keyboard.spin_active = !machine.keyboard.spin_active; }

    if (key & COMM_RC_KEY_CTRL) { move = keyboard_sensitivity_config.slow_raw; }
    else if (key & COMM_RC_KEY_SHIFT) { move = keyboard_sensitivity_config.sprint_raw; }
    memset(next->input.channel, 0, sizeof(next->input.channel));
    next->input.channel[3] = ((key & COMM_RC_KEY_W) ? move : 0) -
                       ((key & COMM_RC_KEY_S) ? move : 0);
    next->input.channel[2] = ((key & COMM_RC_KEY_D) ? move : 0) -
                       ((key & COMM_RC_KEY_A) ? move : 0);
    if (control->mouse.right && keyboard_sensitivity_config.aim_divisor > 0)
    {
        mouse_x /= keyboard_sensitivity_config.aim_divisor;
        mouse_y /= keyboard_sensitivity_config.aim_divisor;
    }
    next->input.channel[0] = RemoteState_ClampChannel(
        mouse_x * keyboard_sensitivity_config.mouse_yaw_gain);
    next->input.channel[1] = RemoteState_ClampChannel(
        mouse_y * keyboard_sensitivity_config.mouse_pitch_gain);
    if (!(key & COMM_RC_KEY_Q)) { machine.keyboard.q_released = true; }
    if (machine.keyboard.q_released && (key & COMM_RC_KEY_Q))
    { next->input.channel[4] = -660; }

    next->mode.chassis = machine.keyboard.spin_active ? REMOTE_MODE_SPIN : machine.keyboard.mode;
    next->safety.spin_enabled = machine.keyboard.spin_active;
    if (next->mode.chassis == REMOTE_MODE_MECHANICAL &&
        (pressed & COMM_RC_KEY_B))
    { machine.keyboard.lift_request_count++; }

    if (!(key & COMM_RC_KEY_F)) { machine.keyboard.f_released = true; }
    if (machine.keyboard.f_released && (pressed & COMM_RC_KEY_F))
    {
        machine.keyboard.friction_on = !machine.keyboard.friction_on;
        machine.keyboard.mouse_released = false;
        machine.keyboard.mouse_press_active = false;
        machine.keyboard.mouse_continuous = false;
    }
    if (!control->mouse.left)
    {
        if (machine.keyboard.mouse_press_active && machine.keyboard.friction_on &&
            !machine.keyboard.mouse_continuous)
        { machine.keyboard.single_request_count++; }
        machine.keyboard.mouse_press_active = false;
        machine.keyboard.mouse_continuous = false;
        machine.keyboard.mouse_released = true;
    }
    else if (machine.keyboard.mouse_released && !machine.keyboard.mouse_press_active)
    {
        machine.keyboard.mouse_press_active = true;
        machine.keyboard.mouse_continuous = false;
        machine.keyboard.mouse_press_ms = HAL_GetTick();
    }
    next->input.right_up = machine.keyboard.mouse_press_active;
    next->safety.shoot_armed = machine.keyboard.friction_on;
    if (machine.keyboard.friction_on)
    {
        if (machine.keyboard.mouse_press_active &&
            (uint32_t)(HAL_GetTick() - machine.keyboard.mouse_press_ms) >
                shoot_config.mouse_continuous_threshold_ms)
        { machine.keyboard.mouse_continuous = true; }
        next->mode.shooting = machine.keyboard.mouse_continuous ?
            REMOTE_SHOOT_CONTINUOUS : REMOTE_SHOOT_READY;
    }
}

void RemoteState_Init(void)
{
    RemoteState_Update(NULL, false);
}

void RemoteState_Update(const Communication_RcControl_t *control, bool online)
{
    RemoteState_t next;
    uint32_t primask;

    memset(&next, 0, sizeof(next));
    next.mode.chassis = REMOTE_MODE_DISABLED;
    next.mode.shooting = REMOTE_SHOOT_OFF;
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
        if (pressed & COMM_RC_KEY_V)
        {
            machine.keyboard.active = !machine.keyboard.active;
            machine.keyboard.mode = REMOTE_MODE_DISABLED;
            machine.keyboard.spin_g_released = false;
            machine.keyboard.spin_active = false;
            machine.keyboard.q_released = false;
            machine.keyboard.f_released = false;
            machine.keyboard.friction_on = false;
            machine.keyboard.mouse_released = false;
            machine.keyboard.mouse_press_active = false;
            machine.keyboard.mouse_continuous = false;
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
                if (wheel >= -cloud_config.spin_wheel_rearm_raw &&
                    wheel <= cloud_config.spin_wheel_rearm_raw)
                { machine.physical.spin_wheel_ready = true; }
                else if (machine.physical.spin_wheel_ready &&
                         wheel >= cloud_config.spin_wheel_trigger_raw)
                {
                    machine.physical.spin_wheel_ready = false;
                    if (machine.physical.spin_selected)
                    { machine.physical.spin_selected = false; }
                    else if (control->rc.s[1] == COMM_RC_SW_DOWN)
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
        next.safety.lift_enabled = next.safety.online && next.mode.chassis == REMOTE_MODE_MECHANICAL &&
            (machine.keyboard.active || control->rc.s[0] == COMM_RC_SW_DOWN);
        if (next.safety.lift_enabled && !machine.keyboard.active)
        { next.input.lift_right_switch = control->rc.s[1]; }

        if (next.safety.online && !machine.keyboard.active &&
            (control->rc.s[1] == COMM_RC_SW_UP ||
             control->rc.s[1] == COMM_RC_SW_MID ||
             control->rc.s[1] == COMM_RC_SW_DOWN))
        {
            next.input.right_up = control->rc.s[1] == COMM_RC_SW_UP;
            if ((next.mode.chassis == REMOTE_MODE_SPIN) != machine.physical.spin_last_mode)
            {
                machine.physical.shoot_switch_seen = false;
                machine.physical.shoot_armed = false;
            }
            machine.physical.spin_last_mode = next.mode.chassis == REMOTE_MODE_SPIN;
            if (next.mode.chassis == REMOTE_MODE_SPIN)
            {
                if (machine.physical.spin_switch_seen &&
                    control->rc.s[1] != machine.physical.spin_previous_switch)
                {
                    machine.physical.spin_armed = true;
                }
                machine.physical.spin_switch_seen = true;
                machine.physical.spin_previous_switch = control->rc.s[1];
                next.safety.spin_enabled = machine.physical.spin_armed && next.input.right_up;
            }
            else
            {
                machine.physical.spin_switch_seen = false;
                machine.physical.spin_previous_switch = 0U;
                machine.physical.spin_armed = false;
            }
            if (machine.physical.shoot_switch_seen &&
                control->rc.s[1] != machine.physical.shoot_previous_switch)
            { machine.physical.shoot_armed = true; }
            machine.physical.shoot_switch_seen = true;
            next.safety.shoot_armed = machine.physical.shoot_armed;
            if (machine.physical.shoot_armed && control->rc.s[1] == COMM_RC_SW_MID)
            { next.mode.shooting = REMOTE_SHOOT_READY; }
            else if (machine.physical.shoot_armed && next.input.right_up)
            {
                next.mode.shooting = next.mode.chassis == REMOTE_MODE_FOLLOW ?
                    REMOTE_SHOOT_CONTINUOUS : REMOTE_SHOOT_SINGLE;
            }
            machine.physical.shoot_previous_switch = control->rc.s[1];
        }
        else if (machine.keyboard.active)
        {
            // 键鼠接管时不让原右拨杆的锁存状态跨模式沿用。
            machine.physical.shoot_switch_seen = false;
            machine.physical.shoot_previous_switch = 0U;
            machine.physical.shoot_armed = false;
            machine.physical.spin_switch_seen = false;
            machine.physical.spin_previous_switch = 0U;
            machine.physical.spin_armed = false;
            machine.physical.spin_last_mode = false;
        }
        if (next.safety.lift_enabled && !machine.keyboard.active)
        {
            // 升降档不同时使能发射；离开后须重新拨动右拨杆。
            next.mode.shooting = REMOTE_SHOOT_OFF;
            next.safety.shoot_armed = false;
            machine.physical.shoot_switch_seen = false;
            machine.physical.shoot_armed = false;
        }
    }

    if (!next.safety.online ||
        (control != NULL && control->rc.s[1] != COMM_RC_SW_UP &&
         control->rc.s[1] != COMM_RC_SW_MID &&
         control->rc.s[1] != COMM_RC_SW_DOWN))
    {
        // 初次上线只记住档位，不能把离线默认值当作真实拨杆动作。
        machine.physical.shoot_switch_seen = false;
        machine.physical.shoot_armed = false;
        machine.physical.shoot_previous_switch = 0U;
        machine.physical.spin_switch_seen = false;
        machine.physical.spin_previous_switch = 0U;
        machine.physical.spin_armed = false;
        machine.physical.spin_wheel_ready = false;
        machine.physical.spin_selected = false;
        machine.physical.spin_last_mode = false;
        machine.physical.previous_left_switch = 0U;
    }
    if (!next.safety.online)
    {
        machine.keyboard.seen = false;
        machine.keyboard.previous_key = 0U;
        machine.keyboard.active = false;
        machine.keyboard.mode = REMOTE_MODE_DISABLED;
        machine.keyboard.spin_g_released = false;
        machine.keyboard.spin_active = false;
        machine.keyboard.q_released = false;
        machine.keyboard.f_released = false;
        machine.keyboard.friction_on = false;
        machine.keyboard.mouse_released = false;
        machine.keyboard.mouse_press_active = false;
        machine.keyboard.mouse_continuous = false;
    }

    next.event.shoot_single_request_count = machine.keyboard.single_request_count;
    next.event.lift_toggle_request_count = machine.keyboard.lift_request_count;

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
