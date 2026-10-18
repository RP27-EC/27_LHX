#ifndef RP_CONTROL_INPUT_H
#define RP_CONTROL_INPUT_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RP_INPUT_METADATA_MAGIC 0xA7U // D3 控制来源扩展标识。
#define RP_INPUT_RIGHT_ACTION_SHIFT 4U // D3 标志中有效右拨杆档位的起始位。
#define RP_INPUT_RIGHT_ACTION_MASK 3U // 有效右拨杆档位占用两位，零表示未重新操作。
#define RP_INPUT_CONTROL_FLAGS_MASK (0x0FU | (RP_INPUT_RIGHT_ACTION_MASK << RP_INPUT_RIGHT_ACTION_SHIFT)) // D3 允许使用的控制标志位。
#define RP_KEY_RESET_MASK ((1U << 5) | (1U << 8)) // Ctrl + R 复位组合。

typedef struct {
    int16_t trigger_raw; // 下拨发射或复位的触发幅值，原始值取负方向。
    int16_t rearm_raw; // 下拨动作释放幅值，回中后允许下一次触发。
    uint32_t continuous_ms; // 拨轮持续发射超过此时间转为连发，ms。
} InputWheelConfig;

typedef struct {
    InputWheelConfig wheel; // 遥控拨轮阈值与单连发时间。
    struct {
        int16_t trigger_raw; // 原调头侧拨轮超过此正向幅值时切换小陀螺。
        int16_t rearm_raw; // 拨轮回到此幅值以下后允许再次切换。
    } spin_wheel; // 小陀螺切换的触发和回中门槛。
    uint32_t reset_hold_ms; // 复位组合连续保持时间，超过后触发，ms。
    int16_t takeover_raw; // 摇杆或拨轮接管的幅值门槛，需同时满足变化门槛。
    int16_t change_raw; // 新遥控操作相对最近有效操作基准的变化门槛。
    int16_t mouse_raw; // 鼠标单次位移超过此幅值时接管键鼠。
} InputControlConfig;
extern volatile InputControlConfig input_control_config;
void InputControlConfig_Init(void);

typedef struct {
    int16_t channel[5], mouse_x, mouse_y;
    uint16_t key;
    uint8_t left, right;
    bool mouse_left, mouse_right;
} RpInputSample;

typedef struct {
    bool seen, keyboard, spin, resetting, reset_fired;
    uint8_t reset_gesture; // 当前复位组合：位0键盘、位1遥控。
    uint8_t right_action; // 当前左档内新操作的右档，零表示尚未重新拨动。
    bool was_locked; // 解锁时重新读取档位，但不重放按键。
    bool spin_wheel_ready; // 小陀螺拨轮已释放，等待下一次越过门槛。
    uint8_t mode; // 基础模式：1机械、2惯性；自旋单独保存。
    uint32_t reset_since, reset_requests;
    int16_t channel_reference[5]; // 最近有效操作的摇杆基准，累计识别缓慢拨杆。
    RpInputSample previous;
} RpInputSelector;

static inline int RpInput_Abs(int value) { return value < 0 ? -value : value; }

// 分别记录两种组合，切换复位操作来源时重新计时。
static inline uint8_t RpInput_ResetGesture(const RpInputSample *p, const InputControlConfig *c)
{
    return ((p->key & RP_KEY_RESET_MASK) == RP_KEY_RESET_MASK ? 1U : 0U) |
        (p->left == 2U && p->right == 3U && p->channel[4] < -c->wheel.trigger_raw ? 2U : 0U);
}

// 仅新操作接管；锁定期间消费变化但不切模式，结束后须重新操作。
static inline void RpInput_Update(RpInputSelector *s, const RpInputSample *p,
    const InputControlConfig *c, bool locked, uint32_t now)
{
    uint16_t pressed;
    bool resuming = s->was_locked && !locked;
    bool from_keyboard = s->keyboard;
    bool physical, keyboard;
    uint8_t previous_gesture = s->reset_gesture;
    uint8_t reset_gesture = RpInput_ResetGesture(p, c);
    bool left_changed = s->seen && p->left != s->previous.left;
    bool right_changed = s->seen && p->right != s->previous.right;
    // 切档、锁定和输入来源变化时，消费仍按住的拨轮。
    if (!s->seen || locked || resuming || left_changed || right_changed || from_keyboard)
    { s->spin_wheel_ready = false; }
    if (!s->seen || left_changed || resuming || (from_keyboard && !right_changed))
    { s->right_action = 0U; }
    else if (right_changed && (!locked || p->left == 2U))
    { s->right_action = p->right; }
    if (s->right_action != p->right) { reset_gesture &= 1U; }
    s->was_locked = locked;
    s->reset_gesture = reset_gesture;
    if (!s->seen) {
        // 上电已按住复位组合时先等松开，避免重启后循环复位。
        s->reset_fired = s->reset_gesture;
        s->resetting = s->reset_gesture;
        s->reset_since = now;
        memcpy(s->channel_reference, p->channel, sizeof(s->channel_reference));
        s->seen = true; s->previous = *p;
        s->mode = !locked && p->left == 1U ? 2U : 1U;
        s->spin = false;
        return;
    }
    pressed = p->key & (uint16_t)~s->previous.key;
    if (s->reset_gesture) {
        if (!s->resetting || reset_gesture != previous_gesture)
        { s->resetting = true; s->reset_since = now; }
        if (!s->reset_fired && (uint32_t)(now - s->reset_since) > c->reset_hold_ms)
        { s->reset_fired = true; s->reset_requests++; }
    } else { s->resetting = false; s->reset_fired = false; }
    physical = p->left != s->previous.left || p->right != s->previous.right;
    for (unsigned i = 0; i < 5U; ++i)
        physical |= (RpInput_Abs(p->channel[i]) >= c->takeover_raw ||
                     RpInput_Abs(s->channel_reference[i]) >= c->takeover_raw) &&
                    RpInput_Abs((int)p->channel[i] - s->channel_reference[i]) >= c->change_raw;
    keyboard = pressed != 0U || p->mouse_left != s->previous.mouse_left ||
        p->mouse_right != s->previous.mouse_right || RpInput_Abs(p->mouse_x) > c->mouse_raw ||
        RpInput_Abs(p->mouse_y) > c->mouse_raw;
    if (physical || locked || (keyboard && !s->keyboard))
    { memcpy(s->channel_reference, p->channel, sizeof(s->channel_reference)); }
    if (!locked && !s->reset_gesture) {
        if (physical) {
            if (s->keyboard) { s->mode = p->left == 1U ? 2U : 1U; s->spin = false; }
            s->keyboard = false;
        }
        else if (keyboard && !s->keyboard) { s->keyboard = true; s->mode = 2U; s->spin = false; }
        if (s->keyboard) {
            if (pressed & (1U << 11)) { s->mode = 2U; s->spin = false; }
            else if (pressed & (1U << 12)) { s->mode = 1U; s->spin = false; }
            if (pressed & (1U << 10)) { s->spin = !s->spin; if (!s->spin) s->mode = 2U; }
            if (pressed & (1U << 15)) { s->mode = 1U; s->spin = false; }
        } else {
            bool select = p->left != s->previous.left || p->right != s->previous.right || from_keyboard;
            if (resuming) { s->mode = p->left == 1U ? 2U : 1U; s->spin = false; }
            else if (left_changed || (from_keyboard && !right_changed)) {
                s->mode = p->left == 1U ? 2U : 1U; s->spin = false;
            }
            else if (select) {
                if (p->left == 2U) { s->mode = 1U; s->spin = false; }
                else if (s->right_action == 2U && s->spin) { s->spin = false; s->mode = 2U; }
                else if (!s->spin) { s->mode = p->left == 1U ? 2U : 1U; }
            }
            // 正常档把原调头拨轮作为自旋开关，退出后保持惯性模式。
            if (!resuming && !left_changed && !right_changed && !from_keyboard &&
                (p->left == 1U || p->left == 3U) &&
                (s->right_action == 1U || s->right_action == 3U) &&
                c->spin_wheel.trigger_raw > c->spin_wheel.rearm_raw &&
                c->spin_wheel.rearm_raw >= 0) {
                if (p->channel[4] <= c->spin_wheel.rearm_raw) { s->spin_wheel_ready = true; }
                else if (s->spin_wheel_ready && p->channel[4] > c->spin_wheel.trigger_raw) {
                    s->spin_wheel_ready = false;
                    s->spin = !s->spin;
                    s->mode = 2U;
                }
            }
        }
        if (s->keyboard) { s->right_action = 0U; s->spin_wheel_ready = false; }
    } else {
        s->spin_wheel_ready = false;
        if (locked) { s->mode = 1U; s->spin = false; }
    }
    s->previous = *p;
}

#define RP_RESET_STOP_WAIT_MS 100U // 等待控制任务停机后应答。
#define RP_RESET_WAIT_TIMEOUT_MS 2000U // 提交前等待对板的最长时间。
#define RP_RESET_ABORT_SEND_MS 100U // 取消状态重复发送的时间。
#define RP_RESET_PEER_TIMEOUT_MS 2500U // 上板等待下一握手阶段的最长时间。

typedef struct {
    uint8_t phase, sequence; // 0空闲、1停机准备、2预提交、3提交重启、4取消握手。
    uint8_t failed_phase; // 最近一次取消前所处的阶段。
    uint32_t since_ms, request_seen;
    uint32_t abort_count; // 本次上电后未提交即取消的次数。
} RpResetState;
extern volatile RpResetState control_reset_state;

static inline void RpReset_Abort(RpResetState *s, uint32_t now)
{
    s->failed_phase = s->phase;
    s->abort_count++;
    s->phase = 4U;
    s->since_ms = now;
}

// 下板协调：先等上板停机，再确认提交准备，最后重复发送重启提交。
static inline void RpReset_Coordinator(RpResetState *s, uint32_t requests,
    uint8_t ack, uint8_t sequence, uint32_t now)
{
    if (requests != s->request_seen) {
        s->request_seen = requests;
        // 握手中的新事件只消费，不能打断已提交的重启。
        if (s->phase == 0U) {
            s->sequence++; s->phase = 1U; s->since_ms = now;
        }
    }
    if (sequence == s->sequence && ((s->phase == 1U && ack == 1U) ||
        (s->phase == 2U && ack == 2U))) { s->phase++; s->since_ms = now; }
    if ((s->phase == 1U || s->phase == 2U) &&
        ((uint32_t)(now - s->since_ms) > RP_RESET_WAIT_TIMEOUT_MS ||
         (sequence == s->sequence && ack == 4U)))
    { RpReset_Abort(s, now); }
    else if (s->phase == 4U && (uint32_t)(now - s->since_ms) >= RP_RESET_ABORT_SEND_MS)
    { s->phase = 0U; }
}

// 只接受完整准备链；对板取消或重新上电后撤销未提交的停机锁存。
static inline void RpReset_Responder(RpResetState *s, uint8_t phase,
    uint8_t sequence, uint32_t now)
{
    if (s->phase == 3U) { return; }
    if (phase == 0U) {
        if (s->phase == 1U || s->phase == 2U) { RpReset_Abort(s, now); }
        if (s->phase == 4U) { s->phase = 0U; }
        return;
    }
    if (phase == 4U && sequence == s->sequence &&
        (s->phase == 1U || s->phase == 2U))
    { RpReset_Abort(s, now); return; }
    if (s->phase == 4U) {
        if ((uint32_t)(now - s->since_ms) >= RP_RESET_ABORT_SEND_MS) { s->phase = 0U; }
        return;
    }
    if ((s->phase == 1U || s->phase == 2U) &&
        (uint32_t)(now - s->since_ms) > RP_RESET_PEER_TIMEOUT_MS)
    { RpReset_Abort(s, now); return; }
    if (phase == 1U && (s->phase == 0U || sequence != s->sequence))
    { s->phase = 1U; s->sequence = sequence; s->since_ms = now; }
    else if (sequence == s->sequence && ((phase == 2U && s->phase == 1U &&
        (uint32_t)(now - s->since_ms) >= RP_RESET_STOP_WAIT_MS) ||
        (phase == 3U && s->phase == 2U)))
    { s->phase = phase; s->since_ms = now; }
}

static inline uint8_t RpReset_Ack(const RpResetState *s, uint32_t now)
{
    return s->phase == 1U && (uint32_t)(now - s->since_ms) < RP_RESET_STOP_WAIT_MS ? 0U : s->phase;
}
#endif
