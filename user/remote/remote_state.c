#include "remote_state.h"
#include "application_config.h"
#include <string.h>

static RemoteState_t remote_state; // 遥控任务发布给底盘任务的状态快照。

typedef struct
{
    bool turnaround_wheel_armed; // 拨轮回中后允许再次调头。
    uint32_t turnaround_request_count; // 调头事件累计数。
} RemotePhysicalLatch_t;

typedef struct
{
    bool q_released; // Q 先松开才接受调头输入。
} RemoteKeyboardLatch_t;

typedef struct
{
    RemotePhysicalLatch_t physical; // 遥控拨杆与拨轮的边沿锁存。
    RemoteKeyboardLatch_t keyboard; // 键鼠模式与按键边沿锁存。
} RemoteStateMachine_t;


static RpInputSelector input_selector; // 来源、模式及复位按键的边沿状态。
static volatile bool operation_locked; // 通信任务更新，输入映射读取。
volatile RpResetState control_reset_state; // 双板停机与重启握手进度。

void RemoteState_SetOperationLock(bool locked) { operation_locked = locked; }

void RemoteState_ServiceControl(uint8_t phase, uint8_t sequence)
{
    RpResetState state = control_reset_state;
    RpReset_Coordinator(&state, input_selector.reset_requests, phase, sequence, HAL_GetTick());
    control_reset_state = state;
}

static RemoteStateMachine_t machine; // 仅状态机任务修改，不直接暴露给底盘任务。

// 将左拨杆档位映射为模式枚举；非法档位保持失能。
static RemoteMode_t RemoteState_MapSwitchMode(uint8_t left_switch)
{
    if (left_switch == RC_SW_UP)
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
                                    const RC_ctrl_t *control)
{
    uint16_t key = control->key;
    int16_t move = keyboard_sensitivity_config.move_raw;
    int32_t mouse_x = control->mouse.x;
    int32_t mouse_y = control->mouse.y;
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
    if (operation_locked) { machine.keyboard.q_released = false; }
    else if (!(key & RC_KEY_Q)) { machine.keyboard.q_released = true; }
    if (!operation_locked && machine.keyboard.q_released && (key & RC_KEY_Q))
    { next->input.channel[4] = -660; }

    next->mode.chassis = input_selector.spin ? REMOTE_MODE_SPIN : ((RemoteMode_t)input_selector.mode);
    next->safety.spin_enabled = input_selector.spin;
}

// 按遥控离线状态复位模式、操作输入及键鼠切换状态。
void RemoteState_Init(void)
{
    memset(&input_selector, 0, sizeof(input_selector));
    control_reset_state = (RpResetState){0};
    operation_locked = false;
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
        bool physical_changed = !input_selector.seen || input_selector.keyboard ||
            control->rc.s[0] != input_selector.previous.left ||
            control->rc.s[1] != input_selector.previous.right;
        next.safety.online = true;
        memcpy(next.input.channel, control->rc.ch, sizeof(next.input.channel));

        {
            InputControlConfig cfg = input_control_config;
            RpInputSample sample = {0};
            sample.key = control->key; sample.left = control->rc.s[0]; sample.right = control->rc.s[1];
            memcpy(sample.channel, control->rc.ch, sizeof(sample.channel));
            sample.mouse_x = control->mouse.x; sample.mouse_y = control->mouse.y;
            sample.mouse_left = control->mouse.left; sample.mouse_right = control->mouse.right;
            RpInput_Update(&input_selector, &sample, &cfg, operation_locked, HAL_GetTick());
        }

        next.mode.chassis = RemoteState_MapSwitchMode(control->rc.s[0]);
        if (next.mode.chassis == REMOTE_MODE_DISABLED)
        {
            next.safety.online = false;
            memset(next.input.channel, 0, sizeof(next.input.channel));
        }

        if (next.safety.online) {
            next.mode.chassis = input_selector.spin ? REMOTE_MODE_SPIN : (RemoteMode_t)input_selector.mode;
        }

        if (next.safety.online && input_selector.keyboard)
        {
            RemoteState_MapKeyboard(&next, control);
        }
        next.input.keyboard_active = next.safety.online && input_selector.keyboard;
        next.input.right_action = next.safety.online ? input_selector.right_action : 0U;

        if (!input_selector.keyboard)
        {
            next.safety.spin_enabled = next.mode.chassis == REMOTE_MODE_SPIN;
            next.input.channel[4] = operation_locked ? 0 : (int16_t)-control->rc.ch[4];
        }

        // 与上板一致：切档时消费已按住的拨轮，回中后才接受新调头。
        if (operation_locked || (!input_selector.keyboard && (physical_changed ||
            input_selector.right_action == 0U ||
            (control->rc.s[0] == RC_SW_DOWN && control->rc.s[1] != RC_SW_MID))))
        { machine.physical.turnaround_wheel_armed = false; }
        if (next.safety.online && !operation_locked && !input_selector.reset_gesture &&
            (input_selector.keyboard || (input_selector.right_action != 0U &&
             (control->rc.s[0] != RC_SW_DOWN || control->rc.s[1] == RC_SW_MID))))
        {
            if (next.input.channel[4] >= -chassis_config.turn.wheel_rearm_raw)
            {
                machine.physical.turnaround_wheel_armed = true;
            }
            else if (machine.physical.turnaround_wheel_armed &&
                     (next.input.channel[4] < -chassis_config.turn.wheel_trigger_raw ||
                      (input_selector.keyboard && next.input.channel[4] == -chassis_config.turn.wheel_trigger_raw)))
            {
                machine.physical.turnaround_wheel_armed = false;
                machine.physical.turnaround_request_count++;
            }
        }
    }
    if (input_selector.reset_gesture) {
        next.mode.chassis = REMOTE_MODE_DISABLED;
        memset(next.input.channel, 0, sizeof(next.input.channel));
    }
    if (!next.safety.online)
    {
        uint32_t requests = input_selector.reset_requests;
        memset(&input_selector, 0, sizeof(input_selector));
        input_selector.reset_requests = requests;
        machine.physical.turnaround_wheel_armed = false;
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
    state->safety.reset_active = control_reset_state.phase != 0U;
    if (state->safety.reset_active) {
        state->safety.online = false; state->mode.chassis = REMOTE_MODE_DISABLED;
        memset(state->input.channel, 0, sizeof(state->input.channel));
    }
    __set_PRIMASK(primask);
}
