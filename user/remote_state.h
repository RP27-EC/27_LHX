#ifndef REMOTE_STATE_H
#define REMOTE_STATE_H

#include "telecontrol.h"
#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_MECHANICAL_SWITCH_POSITION 3U
#define CHASSIS_MECHANICAL_DOWN_POSITION   2U
#define CHASSIS_SPIN_SWITCH_1_POSITION     1U

// 下板依据遥控拨杆运行的有限状态机。
typedef enum
{
    REMOTE_MODE_DISABLED = 0, // 断联或非法输入，底盘停止。
    REMOTE_MODE_MECHANICAL, // 底盘由输入直接转向。
    REMOTE_MODE_FOLLOW, // 底盘按云台相对角跟随。
    REMOTE_MODE_SPIN // 小陀螺旋转并可按云台朝向平移。
} RemoteMode_t;

typedef struct
{
    RemoteMode_t chassis; // 底盘控制模式的枚举值。
} RemoteModeState_t;

typedef struct
{
    int16_t channel[5]; // 与模式同步的五路输入通道。
    bool keyboard_active; // V 键选择键鼠输入。
} RemoteInputState_t;

typedef struct
{
    bool online; // 遥控器当前是否在线。
    bool spin_enabled; // 右拨杆换档或键鼠 G 使能自旋。
} RemoteSafetyState_t;

typedef struct
{
    uint32_t turnaround_request_count; // 拨轮或 Q 键调头事件累计数。
} RemoteEventState_t;

typedef struct
{
    RemoteModeState_t mode; // 枚举模式选择。
    RemoteInputState_t input; // 遥控器或键鼠的当前输入。
    RemoteSafetyState_t safety; // 在线状态和自旋使能标志。
    RemoteEventState_t event; // 供底盘任务消费的边沿事件。
} RemoteState_t;

// 由遥控解析任务写入，底盘任务读取。
void RemoteState_Init(void);
void RemoteState_Update(const RC_ctrl_t *control, bool online);
void RemoteState_Get(RemoteState_t *state);

#endif // REMOTE_STATE_H
