#ifndef REMOTE_STATE_H
#define REMOTE_STATE_H

#include "telecontrol.h"
#include <stdbool.h>
#include <stdint.h>

/* 下板依据遥控拨杆运行的有限状态机。 */
typedef enum
{
    REMOTE_MODE_DISABLED = 0,
    REMOTE_MODE_MECHANICAL,
    REMOTE_MODE_FOLLOW,
    REMOTE_MODE_SPIN
} RemoteMode_t;

typedef struct
{
    RemoteMode_t mode;   /* 当前底盘控制模式。 */
    int16_t channel[5]; /* 与模式同步的五路遥控通道。 */
    bool online;        /* 遥控器当前是否在线。 */
    bool keyboard_active; /* V 键切换的键鼠控制状态。 */
    bool spin_enabled;  /* 遥控模式需右拨杆换档；键鼠模式需 G 按下沿使能自旋。 */
    uint32_t turnaround_request_count; /* 拨轮上拨下降沿累计次数；底盘任务逐次消费。 */
} RemoteState_t;

/* 由遥控解析任务写入，底盘任务读取。 */
void RemoteState_Init(void);
void RemoteState_Update(const RC_ctrl_t *control, bool online);
void RemoteState_Get(RemoteState_t *state);

#endif /* REMOTE_STATE_H */
