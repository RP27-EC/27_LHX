#include "remote_state.h"
#include "parameter.h"
#include <string.h>

static RemoteState_t remote_state; /* 遥控任务发布给底盘任务的状态快照。 */
static bool turnaround_wheel_armed; /* 拨轮回到中位附近后才允许再次触发。 */
static uint32_t turnaround_request_count; /* 拨轮下降沿事件累计数。 */
static bool spin_switch_seen; /* 本次进入小陀螺后是否已记录右拨杆初始档位。 */
static uint8_t spin_previous_switch; /* 小陀螺模式内上一次有效右拨杆档位。 */
static bool spin_armed; /* 本次进入小陀螺后右拨杆是否真实换过档。 */
static bool spin_wheel_ready; /* 正拨前必须先回中位。 */
static bool physical_spin_selected; /* 拨轮切换的小陀螺状态。 */
static uint8_t previous_left_switch; /* 左拨杆换档时退出小陀螺。 */
static bool keyboard_seen; /* 上线后的首帧只记录键位，不作为切换动作。 */
static bool keyboard_active; /* V 键切换的键鼠控制状态。 */
static uint16_t keyboard_previous_key; /* 上一次键盘位图，用于按下沿判断。 */
static RemoteMode_t keyboard_mode; /* 键鼠模式下的底盘模式。 */
static bool keyboard_spin_g_released; /* 进入小陀螺后须先观察 G 松开。 */
static bool keyboard_spin_armed; /* 本次小陀螺模式内 G 按下后允许自旋。 */
static bool keyboard_q_released; /* 进入键鼠模式后须先观察 Q 松开。 */
static bool keyboard_f_released; /* 键鼠发射键先松开才接受按下。 */
static bool keyboard_friction_on; /* 与上板一致的 F 键保险状态。 */

/* V 进出键鼠；Z/X/C 选跟随/机械/小陀螺；WASD 移动，G 控自旋，Q 调头。
 * 鼠标位移仍转成原有摇杆通道，底盘控制和归中状态机无需另开路径。 */
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
    int16_t move = RC_KEYBOARD_MOVE_RAW;
    int32_t mouse_x = control->mouse.x;
    int32_t mouse_y = control->mouse.y;
    RemoteMode_t previous_mode = keyboard_mode;

    if (pressed & RC_KEY_Z)
    { keyboard_mode = REMOTE_MODE_FOLLOW; }
    else if (pressed & RC_KEY_X)
    { keyboard_mode = REMOTE_MODE_MECHANICAL; }
    else if ((pressed & RC_KEY_C) && !keyboard_friction_on)
    { keyboard_mode = REMOTE_MODE_SPIN; }

    if (key & RC_KEY_CTRL) { move = RC_KEYBOARD_SLOW_RAW; }
    else if (key & RC_KEY_SHIFT) { move = RC_KEYBOARD_SPRINT_RAW; }
    memset(next->channel, 0, sizeof(next->channel));
    next->channel[3] = ((key & RC_KEY_W) ? move : 0) -
                       ((key & RC_KEY_S) ? move : 0);
    next->channel[2] = ((key & RC_KEY_D) ? move : 0) -
                       ((key & RC_KEY_A) ? move : 0);
    if (control->mouse.right)
    {
        mouse_x /= RC_KEYBOARD_AIM_DIVISOR;
        mouse_y /= RC_KEYBOARD_AIM_DIVISOR;
    }
    next->channel[0] = RemoteState_ClampChannel(
        mouse_x * RC_KEYBOARD_MOUSE_YAW_GAIN);
    next->channel[1] = RemoteState_ClampChannel(
        -mouse_y * RC_KEYBOARD_MOUSE_PITCH_GAIN);
    if (!(key & RC_KEY_Q)) { keyboard_q_released = true; }
    if (keyboard_q_released && (key & RC_KEY_Q))
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
        }
    }
    if (keyboard_mode == REMOTE_MODE_SPIN)
    {
        if (!(key & RC_KEY_G)) { keyboard_spin_g_released = true; }
        if (keyboard_spin_g_released && (pressed & RC_KEY_G))
        { keyboard_spin_armed = !keyboard_spin_armed; }
        next->spin_enabled = keyboard_spin_armed;
        keyboard_f_released = false;
        keyboard_friction_on = false;
        return;
    }
    if (!(key & RC_KEY_F)) { keyboard_f_released = true; }
    if (keyboard_f_released && (pressed & RC_KEY_F))
    { keyboard_friction_on = !keyboard_friction_on; }
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
    next.mode = REMOTE_MODE_DISABLED;
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

        if (pressed & RC_KEY_V)
        {
            keyboard_active = !keyboard_active;
            keyboard_spin_g_released = false;
            keyboard_spin_armed = false;
            keyboard_q_released = false;
            keyboard_f_released = false;
            keyboard_friction_on = false;
            keyboard_mode = REMOTE_MODE_DISABLED;
        }

        if (control->rc.s[0] == CHASSIS_FOLLOW_SWITCH_POSITION)
        {
            next.mode = REMOTE_MODE_FOLLOW;
        }
        else if (control->rc.s[0] == CHASSIS_MECHANICAL_SWITCH_POSITION ||
                 control->rc.s[0] == CHASSIS_MECHANICAL_DOWN_POSITION)
        {
            next.mode = REMOTE_MODE_MECHANICAL;
        }
        else
        {
            next.online = false;
            memset(next.channel, 0, sizeof(next.channel));
        }

        if (next.online)
        {
            int16_t wheel = control->rc.ch[4];
            if (control->rc.s[0] != previous_left_switch)
            {
                previous_left_switch = control->rc.s[0];
                physical_spin_selected = false;
                spin_wheel_ready = false;
            }
            if (keyboard_active)
            {
                physical_spin_selected = false;
                spin_wheel_ready = false;
            }
            else
            {
                if (wheel >= -CHASSIS_SPIN_WHEEL_REARM_RAW &&
                    wheel <= CHASSIS_SPIN_WHEEL_REARM_RAW)
                { spin_wheel_ready = true; }
                else if (spin_wheel_ready &&
                         wheel >= CHASSIS_SPIN_WHEEL_TRIGGER_RAW)
                {
                    spin_wheel_ready = false;
                    if (physical_spin_selected)
                    { physical_spin_selected = false; }
                    else if (control->rc.s[1] == RC_SW_DOWN)
                    { physical_spin_selected = true; }
                }
                if (physical_spin_selected)
                { next.mode = REMOTE_MODE_SPIN; }
            }
        }

        if (next.online && keyboard_active)
        {
            if (keyboard_mode == REMOTE_MODE_DISABLED)
            { keyboard_mode = next.mode; }
            RemoteState_MapKeyboard(&next, control, pressed);
        }
        next.keyboard_active = next.online && keyboard_active;

        if (next.online && !keyboard_active &&
            next.mode == REMOTE_MODE_SPIN &&
            (control->rc.s[1] == RC_SW_UP ||
             control->rc.s[1] == RC_SW_MID ||
             control->rc.s[1] == RC_SW_DOWN))
        {
            if (spin_switch_seen &&
                control->rc.s[1] != spin_previous_switch)
            {
                spin_armed = true;
            }
            spin_switch_seen = true;
            spin_previous_switch = control->rc.s[1];
            next.spin_enabled = spin_armed &&
                control->rc.s[1] == CHASSIS_SPIN_SWITCH_1_POSITION;
        }
        else
        {
            /* 离开模式、断联或非法档位后，下次进入必须重新拨动。 */
            spin_switch_seen = false;
            spin_previous_switch = 0U;
            spin_armed = false;
        }

        if (next.online)
        {
            if (next.channel[4] > -CHASSIS_TURN_WHEEL_REARM_RAW)
            {
                turnaround_wheel_armed = true;
            }
            else if (turnaround_wheel_armed &&
                     next.channel[4] <= -CHASSIS_TURN_WHEEL_TRIGGER_RAW)
            {
                turnaround_wheel_armed = false;
                turnaround_request_count++;
            }
        }
    }
    if (!next.online)
    {
        turnaround_wheel_armed = false;
        spin_switch_seen = false;
        spin_previous_switch = 0U;
        spin_armed = false;
        spin_wheel_ready = false;
        physical_spin_selected = false;
        previous_left_switch = 0U;
        keyboard_seen = false;
        keyboard_previous_key = 0U;
        keyboard_active = false;
        keyboard_mode = REMOTE_MODE_DISABLED;
        keyboard_spin_g_released = false;
        keyboard_spin_armed = false;
        keyboard_q_released = false;
        keyboard_f_released = false;
        keyboard_friction_on = false;
    }
    next.turnaround_request_count = turnaround_request_count;

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
