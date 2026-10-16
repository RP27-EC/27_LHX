#include "remote_state.h"
#include "application_config.h"
#include <string.h>

static RemoteState_t remote_state; // 通信任务发布给各控制任务的遥控状态。

typedef struct
{
    bool turn_ready; // 调头拨轮已回中，锁定期间清除。
    bool wheel_ready; // 发射拨轮已回中。
    bool wheel_active; // 当前拨轮按下已登记首发。
    uint32_t wheel_start_ms; // 本次拨轮发射按下时间。
    uint32_t lift_request_count; // 遥控升降方向请求序号。
} RemotePhysicalLatch_t;

typedef struct
{
    bool q_released; // Q 先松开才接受调头输入。
    bool friction_on; // F 按下沿锁存摩擦轮状态。
    bool mouse_released; // 左键先松开才允许发射。
    bool mouse_press_active; // 正在记录一次有效左键按下。
    bool mouse_continuous; // 本次按下已进入连发。
    uint32_t mouse_press_ms; // 左键按下时间，ms。
    uint32_t arm_request_count; // 新的手动布防操作序号。
    uint32_t single_request_count; // 左键按下时先登记一发。
    uint32_t lift_request_count; // B 键升降切换事件累计数。
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
    RpReset_Responder(&state, phase, sequence, HAL_GetTick());
    control_reset_state = state;
}

static RemoteStateMachine_t machine; // 仅状态机任务修改，不直接暴露给控制任务。

// 将左拨杆档位映射为模式枚举；非法档位保持失能。
static RemoteMode_t RemoteState_MapSwitchMode(uint8_t left_switch)
{
    if (left_switch == COMM_RC_SW_DOWN || left_switch == COMM_RC_SW_MID)
    { return REMOTE_MODE_MECHANICAL; }
    if (left_switch == COMM_RC_SW_UP) { return REMOTE_MODE_FOLLOW; }
    return REMOTE_MODE_DISABLED;
}

// 键鼠：Z/X 选模式，G 切换小陀螺，B 自动切机械并切换升降目标。
// F/左键控制发射，两种来源的小陀螺均允许供弹。
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
    if (operation_locked) { machine.keyboard.q_released = false; }
    else if (!(key & COMM_RC_KEY_Q)) { machine.keyboard.q_released = true; }
    if (!operation_locked && machine.keyboard.q_released && (key & COMM_RC_KEY_Q))
    { next->input.channel[4] = -660; }

    next->mode.chassis = input_selector.spin ? REMOTE_MODE_SPIN : ((RemoteMode_t)input_selector.mode);
    next->safety.spin_enabled = input_selector.spin;
    if (pressed & COMM_RC_KEY_B)
    { machine.keyboard.lift_request_count++; }

    if (operation_locked)
    {
        machine.keyboard.mouse_released = false;
        machine.keyboard.mouse_press_active = false;
        machine.keyboard.mouse_continuous = false;
    }
    else
    {
        if (pressed & COMM_RC_KEY_F)
        {
            machine.keyboard.friction_on = !machine.keyboard.friction_on;
            machine.keyboard.arm_request_count++;
            machine.keyboard.mouse_released = false;
            machine.keyboard.mouse_press_active = false;
            machine.keyboard.mouse_continuous = false;
        }
        if (!control->mouse.left)
        {
            machine.keyboard.mouse_press_active = false;
            machine.keyboard.mouse_continuous = false;
            machine.keyboard.mouse_released = true;
        }
        else if (machine.keyboard.mouse_released && !machine.keyboard.mouse_press_active)
        {
            machine.keyboard.mouse_press_active = true;
            machine.keyboard.friction_on = true;
            machine.keyboard.single_request_count++;
            machine.keyboard.arm_request_count++;
            machine.keyboard.mouse_continuous = false;
            machine.keyboard.mouse_press_ms = HAL_GetTick();
        }
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

// 按遥控离线状态复位模式、操作输入及键鼠切换状态。
void RemoteState_Init(void)
{
    memset(&input_selector, 0, sizeof(input_selector));
    control_reset_state = (RpResetState){0};
    operation_locked = false;
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
        uint16_t pressed = input_selector.seen ?
            control->key & (uint16_t)~input_selector.previous.key : 0U;
        bool previous_keyboard = input_selector.keyboard;
        bool physical_changed = !input_selector.seen || previous_keyboard ||
            control->rc.s[0] != input_selector.previous.left ||
            control->rc.s[1] != input_selector.previous.right;
        bool new_mouse_press = input_selector.seen && control->mouse.left && !input_selector.previous.mouse_left;

        next.safety.online = true;
        memcpy(next.input.channel, control->rc.ch, sizeof(next.input.channel));
        next.input.right_switch = control->rc.s[1];
        if (operation_locked) { pressed &= COMM_RC_KEY_B; }
        {
            InputControlConfig cfg = input_control_config;
            RpInputSample sample = {0};
            sample.key = control->key; sample.left = control->rc.s[0]; sample.right = control->rc.s[1];
            memcpy(sample.channel, control->rc.ch, sizeof(sample.channel));
            sample.mouse_x = control->mouse.x; sample.mouse_y = control->mouse.y;
            sample.mouse_left = control->mouse.left; sample.mouse_right = control->mouse.right;
            RpInput_Update(&input_selector, &sample, &cfg, operation_locked, HAL_GetTick());
        }

        if (control->control_valid) {
            input_selector.keyboard = (control->control_flags & 1U) != 0U;
            input_selector.spin = ((control->control_flags >> 1) & 3U) == REMOTE_MODE_SPIN;
            input_selector.mode = input_selector.spin ? REMOTE_MODE_FOLLOW : (control->control_flags >> 1) & 3U;
        }

        if (!previous_keyboard && input_selector.keyboard && new_mouse_press) { machine.keyboard.mouse_released = true; }
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
            RemoteState_MapKeyboard(&next, control, pressed);
        }
        next.input.keyboard_active = next.safety.online && input_selector.keyboard;
        next.safety.lift_enabled = next.safety.online && next.mode.chassis == REMOTE_MODE_MECHANICAL &&
            (input_selector.keyboard || control->rc.s[0] == COMM_RC_SW_DOWN);
        if (control->control_valid) { next.safety.spin_enabled = !operation_locked && (control->control_flags & 8U) != 0U; }
        if (next.safety.lift_enabled && !input_selector.keyboard)
        { next.input.lift_right_switch = control->rc.s[1]; }

        if (next.safety.online && !input_selector.keyboard)
        {
            bool changed = physical_changed;
            int16_t wheel = control->rc.ch[4];
            bool shoot_gear = control->rc.s[0] != COMM_RC_SW_DOWN &&
                (control->rc.s[1] == COMM_RC_SW_MID || control->rc.s[1] == COMM_RC_SW_UP);
            bool turn_gear = control->rc.s[0] != COMM_RC_SW_DOWN || control->rc.s[1] == COMM_RC_SW_MID;
            // 调头只发一次输入脉冲；升降中按住的拨轮须先回中。
            next.input.channel[4] = 0;
            if (changed || operation_locked || !turn_gear) { machine.physical.turn_ready = false; }
            if (!operation_locked && turn_gear && !input_selector.reset_gesture)
            {
                if (wheel <= cloud_config.yaw.turn.wheel_rearm_raw) { machine.physical.turn_ready = true; }
                else if (machine.physical.turn_ready && wheel > cloud_config.yaw.turn.wheel_trigger_raw)
                { next.input.channel[4] = (int16_t)-wheel; machine.physical.turn_ready = false; }
            }
            if (changed || operation_locked || !shoot_gear || input_selector.reset_gesture)
            { machine.physical.wheel_active = false; machine.physical.wheel_ready = false; }
            // 升降期间只接收新的升降方向，其他动作继续忽略。
            if (!input_selector.reset_gesture && control->rc.s[0] == COMM_RC_SW_DOWN && changed &&
                (control->rc.s[1] == COMM_RC_SW_UP || control->rc.s[1] == COMM_RC_SW_DOWN))
            { machine.physical.lift_request_count++; }
            next.safety.spin_enabled = !operation_locked && next.mode.chassis == REMOTE_MODE_SPIN;
            if (!operation_locked && !input_selector.reset_gesture)
            {
                if (wheel >= -input_control_config.wheel.rearm_raw)
                { machine.physical.wheel_active = false; machine.physical.wheel_ready = true; }
                else if (shoot_gear && machine.physical.wheel_ready &&
                         wheel < -input_control_config.wheel.trigger_raw)
                {
                    machine.physical.wheel_ready = false;
                    machine.physical.wheel_active = true;
                    machine.physical.wheel_start_ms = HAL_GetTick();
                    machine.keyboard.single_request_count++;
                    machine.keyboard.arm_request_count++;
                }
            }
            if (shoot_gear && !operation_locked && !input_selector.reset_gesture)
            {
                next.safety.shoot_armed = true;
                next.input.right_up = machine.physical.wheel_active;
                next.mode.shooting = machine.physical.wheel_active &&
                    (uint32_t)(HAL_GetTick() - machine.physical.wheel_start_ms) >
                        input_control_config.wheel.continuous_ms ? REMOTE_SHOOT_CONTINUOUS : REMOTE_SHOOT_READY;
            }
        }
        else
        { machine.physical.wheel_active = false; machine.physical.wheel_ready = false; }
    }

    if (control != NULL && control->control_valid && control->reset_phase != 0U)
    { next.safety.online = false; next.mode.shooting = REMOTE_SHOOT_OFF; }
    if (input_selector.reset_gesture) {
        next.mode.chassis = REMOTE_MODE_DISABLED;
        next.mode.shooting = REMOTE_SHOOT_OFF;
        next.safety.shoot_armed = false;
        next.safety.lift_enabled = false;
        memset(next.input.channel, 0, sizeof(next.input.channel));
    }

    if (!next.safety.online)
    {
        uint32_t requests = input_selector.reset_requests;
        memset(&input_selector, 0, sizeof(input_selector));
        input_selector.reset_requests = requests;
        machine.physical.wheel_active = false;
        machine.physical.wheel_ready = false;
        machine.physical.turn_ready = false;
        machine.keyboard.q_released = false;
        machine.keyboard.friction_on = false;
        machine.keyboard.mouse_released = false;
        machine.keyboard.mouse_press_active = false;
        machine.keyboard.mouse_continuous = false;
    }

    next.event.shoot_arm_request_count = machine.keyboard.arm_request_count;
    next.event.shoot_single_request_count = machine.keyboard.single_request_count;
    next.event.lift_toggle_request_count = machine.keyboard.lift_request_count;
    next.event.lift_motion_request_count = machine.physical.lift_request_count;

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
