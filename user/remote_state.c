#include "remote_state.h"
#include <string.h>

static RemoteState_t remote_state; /* 通信任务发布给各控制任务的遥控状态。 */
static bool shoot_switch_seen; /* 是否已记录本次上线后的首个右拨杆档位。 */
static uint8_t shoot_previous_switch; /* 上一次有效遥控快照的右拨杆档位。 */
static bool shoot_armed; /* 右拨杆切换后锁存，断联时清除。 */
static bool spin_switch_seen; /* 本次进入小陀螺后是否已记录右拨杆初始档位。 */
static uint8_t spin_previous_switch; /* 小陀螺模式内上一次有效右拨杆档位。 */
static bool spin_armed; /* 本次进入小陀螺后右拨杆是否真实换过档。 */
static bool keyboard_seen; /* 上线首帧只记录键位，不误判为按键事件。 */
static bool keyboard_active; /* V 键切换的键鼠控制状态。 */
static uint16_t keyboard_previous_key; /* 上一帧键盘位图。 */
static RemoteMode_t keyboard_mode; /* 键鼠模式下沿用现有的三种模式。 */
static bool keyboard_spin_g_released; /* 进入小陀螺后先观察 G 松开。 */
static bool keyboard_spin_armed; /* G 按下沿切换的自旋状态。 */
static bool keyboard_q_released; /* 进入键鼠模式后先观察 Q 松开。 */
static bool keyboard_f_released; /* 进入键鼠模式或离开小陀螺后先观察 F 松开。 */
static bool keyboard_friction_on; /* F 按下沿切换摩擦轮准备状态。 */
static bool keyboard_mouse_released; /* 键鼠模式内左键先松开才能发射。 */
static bool keyboard_mouse_press_active; /* 已记录一次有效左键按下。 */
static bool keyboard_mouse_continuous; /* 本次按住已真正进入连发状态。 */
static uint32_t keyboard_mouse_press_ms; /* 有效左键按下的 HAL 毫秒时间。 */
static uint32_t keyboard_single_request_count; /* 短按松开产生的单发事件累计数。 */

/* V 进出键鼠；Z/X/C 选跟随/机械/小陀螺；WASD 移动、鼠标瞄准。
 * F 开关摩擦轮，左键短按单发/长按连发，右键减速瞄准，G 自旋，Q 调头。 */
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
    int16_t move = RC_KEYBOARD_MOVE_RAW;
    int32_t mouse_x = control->mouse.x;
    int32_t mouse_y = control->mouse.y;
    RemoteMode_t previous_mode = keyboard_mode;

    if (pressed & COMM_RC_KEY_Z)
    { keyboard_mode = REMOTE_MODE_FOLLOW; }
    else if (pressed & COMM_RC_KEY_X)
    { keyboard_mode = REMOTE_MODE_MECHANICAL; }
    else if (pressed & COMM_RC_KEY_C)
    { keyboard_mode = REMOTE_MODE_SPIN; }

    if (key & COMM_RC_KEY_CTRL) { move = RC_KEYBOARD_SLOW_RAW; }
    else if (key & COMM_RC_KEY_SHIFT) { move = RC_KEYBOARD_SPRINT_RAW; }
    memset(next->channel, 0, sizeof(next->channel));
    next->channel[3] = ((key & COMM_RC_KEY_W) ? move : 0) -
                       ((key & COMM_RC_KEY_S) ? move : 0);
    next->channel[2] = ((key & COMM_RC_KEY_D) ? move : 0) -
                       ((key & COMM_RC_KEY_A) ? move : 0);
    if (control->mouse.right)
    {
        mouse_x /= RC_KEYBOARD_AIM_DIVISOR;
        mouse_y /= RC_KEYBOARD_AIM_DIVISOR;
    }
    next->channel[0] = RemoteState_ClampChannel(
        mouse_x * RC_KEYBOARD_MOUSE_YAW_GAIN);
    next->channel[1] = RemoteState_ClampChannel(
        mouse_y * RC_KEYBOARD_MOUSE_PITCH_GAIN);
    if (!(key & COMM_RC_KEY_Q)) { keyboard_q_released = true; }
    if (keyboard_q_released && (key & COMM_RC_KEY_Q))
    { next->channel[4] = -660; }

    next->mode = keyboard_mode;
    if (keyboard_mode != previous_mode)
    {
        keyboard_spin_g_released = false;
        keyboard_spin_armed = false;
        if (keyboard_mode == REMOTE_MODE_SPIN ||
            previous_mode == REMOTE_MODE_SPIN)
        {
            keyboard_f_released = false;
            keyboard_friction_on = false;
            keyboard_mouse_released = false;
            keyboard_mouse_press_active = false;
            keyboard_mouse_continuous = false;
        }
    }
    if (keyboard_mode == REMOTE_MODE_SPIN)
    {
        if (!(key & COMM_RC_KEY_G)) { keyboard_spin_g_released = true; }
        if (keyboard_spin_g_released && (pressed & COMM_RC_KEY_G))
        { keyboard_spin_armed = !keyboard_spin_armed; }
        next->spin_enabled = keyboard_spin_armed;
        keyboard_f_released = false;
        keyboard_friction_on = false;
        keyboard_mouse_released = false;
        keyboard_mouse_press_active = false;
        keyboard_mouse_continuous = false;
        return;
    }

    if (!(key & COMM_RC_KEY_F)) { keyboard_f_released = true; }
    if (keyboard_f_released && (pressed & COMM_RC_KEY_F))
    {
        keyboard_friction_on = !keyboard_friction_on;
        keyboard_mouse_released = false;
        keyboard_mouse_press_active = false;
        keyboard_mouse_continuous = false;
    }
    if (!control->mouse.left)
    {
        if (keyboard_mouse_press_active && keyboard_friction_on &&
            !keyboard_mouse_continuous)
        { keyboard_single_request_count++; }
        keyboard_mouse_press_active = false;
        keyboard_mouse_continuous = false;
        keyboard_mouse_released = true;
    }
    else if (keyboard_mouse_released && !keyboard_mouse_press_active)
    {
        keyboard_mouse_press_active = true;
        keyboard_mouse_continuous = false;
        keyboard_mouse_press_ms = HAL_GetTick();
    }
    next->right_up = keyboard_mouse_press_active;
    next->shoot_armed = keyboard_friction_on;
    if (keyboard_friction_on)
    {
        if (keyboard_mouse_press_active &&
            (uint32_t)(HAL_GetTick() - keyboard_mouse_press_ms) >
                SHOOT_MOUSE_CONTINUOUS_THRESHOLD_MS)
        { keyboard_mouse_continuous = true; }
        next->shoot = keyboard_mouse_continuous ?
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
    next.mode = REMOTE_MODE_DISABLED;
    next.shoot = REMOTE_SHOOT_OFF;
    if (online && control != NULL)
    {
        uint16_t pressed = 0U;

        next.online = true;
        memcpy(next.channel, control->rc.ch, sizeof(next.channel));
        if (keyboard_seen)
        { pressed = control->key & (uint16_t)~keyboard_previous_key; }
        else
        { keyboard_seen = true; }
        keyboard_previous_key = control->key;
        if (pressed & COMM_RC_KEY_V)
        {
            keyboard_active = !keyboard_active;
            keyboard_mode = REMOTE_MODE_DISABLED;
            keyboard_spin_g_released = false;
            keyboard_spin_armed = false;
            keyboard_q_released = false;
            keyboard_f_released = false;
            keyboard_friction_on = false;
            keyboard_mouse_released = false;
            keyboard_mouse_press_active = false;
            keyboard_mouse_continuous = false;
        }

        /* 左下档始终为小陀螺模式；自旋需本次进入后拨动右拨杆。 */
        if (control->rc.s[0] == COMM_RC_SW_DOWN)
        {
            next.mode = REMOTE_MODE_SPIN;
        }
        else if (control->rc.s[0] == COMM_RC_SW_MID)
        {
            next.mode = REMOTE_MODE_MECHANICAL;
        }
        else if (control->rc.s[0] == COMM_RC_SW_UP)
        {
            next.mode = REMOTE_MODE_FOLLOW;
        }
        else
        {
            next.online = false;
            memset(next.channel, 0, sizeof(next.channel));
        }

        if (next.online && keyboard_active)
        {
            if (keyboard_mode == REMOTE_MODE_DISABLED)
            { keyboard_mode = next.mode; }
            RemoteState_MapKeyboard(&next, control, pressed);
        }
        next.keyboard_active = next.online && keyboard_active;

        if (next.online && !keyboard_active &&
            (control->rc.s[1] == COMM_RC_SW_UP ||
             control->rc.s[1] == COMM_RC_SW_MID ||
             control->rc.s[1] == COMM_RC_SW_DOWN))
        {
            next.right_up = control->rc.s[1] == COMM_RC_SW_UP;
            if (next.mode == REMOTE_MODE_SPIN)
            {
                if (spin_switch_seen &&
                    control->rc.s[1] != spin_previous_switch)
                {
                    spin_armed = true;
                }
                spin_switch_seen = true;
                spin_previous_switch = control->rc.s[1];
                next.spin_enabled = spin_armed && next.right_up;
                /* 左下档全保险；离开小陀螺后必须重新拨动右拨杆。 */
                shoot_switch_seen = false;
                shoot_armed = false;
            }
            else
            {
                spin_switch_seen = false;
                spin_previous_switch = 0U;
                spin_armed = false;
                if (shoot_switch_seen &&
                    control->rc.s[1] != shoot_previous_switch)
                {
                    shoot_armed = true;
                }
                shoot_switch_seen = true;
                next.shoot_armed = shoot_armed;
                if (shoot_armed && control->rc.s[1] == COMM_RC_SW_MID)
                {
                    next.shoot = REMOTE_SHOOT_READY;
                }
                else if (shoot_armed && next.right_up)
                {
                    next.shoot = next.mode == REMOTE_MODE_FOLLOW ?
                        REMOTE_SHOOT_CONTINUOUS : REMOTE_SHOOT_SINGLE;
                }
            }
            shoot_previous_switch = control->rc.s[1];
        }
        else if (keyboard_active)
        {
            /* 键鼠接管时不让原右拨杆的锁存状态跨模式沿用。 */
            shoot_switch_seen = false;
            shoot_previous_switch = 0U;
            shoot_armed = false;
            spin_switch_seen = false;
            spin_previous_switch = 0U;
            spin_armed = false;
        }
    }

    if (!next.online ||
        (control != NULL && control->rc.s[1] != COMM_RC_SW_UP &&
         control->rc.s[1] != COMM_RC_SW_MID &&
         control->rc.s[1] != COMM_RC_SW_DOWN))
    {
        /* 初次上线只记住档位，不能把离线默认值当作真实拨杆动作。 */
        shoot_switch_seen = false;
        shoot_armed = false;
        shoot_previous_switch = 0U;
        spin_switch_seen = false;
        spin_previous_switch = 0U;
        spin_armed = false;
    }
    if (!next.online)
    {
        keyboard_seen = false;
        keyboard_previous_key = 0U;
        keyboard_active = false;
        keyboard_mode = REMOTE_MODE_DISABLED;
        keyboard_spin_g_released = false;
        keyboard_spin_armed = false;
        keyboard_q_released = false;
        keyboard_f_released = false;
        keyboard_friction_on = false;
        keyboard_mouse_released = false;
        keyboard_mouse_press_active = false;
        keyboard_mouse_continuous = false;
    }

    next.shoot_single_request_count = keyboard_single_request_count;

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
