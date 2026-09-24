#ifndef REMOTE_STATE_H
#define REMOTE_STATE_H

#include "communication.h"
#include <stdbool.h>
#include <stdint.h>

/* 左拨杆决定整车模式；断联或非法档位均进入安全停止态。 */
typedef enum
{
    REMOTE_MODE_DISABLED = 0,
    REMOTE_MODE_MECHANICAL,
    REMOTE_MODE_FOLLOW,
    REMOTE_MODE_SPIN
} RemoteMode_t;

/* 左右拨杆组合决定发射状态，小陀螺和手动底盘均强制保险。 */
typedef enum
{
    REMOTE_SHOOT_OFF = 0,
    REMOTE_SHOOT_READY,
    REMOTE_SHOOT_SINGLE,
    REMOTE_SHOOT_CONTINUOUS
} RemoteShoot_t;

typedef struct
{
    RemoteMode_t mode;       /* 当前底盘/云台协同模式。 */
    RemoteShoot_t shoot;     /* 当前发射档位。 */
    int16_t channel[5];     /* 同一次遥控快照中的五路通道。 */
    bool online;            /* 遥控帧当前有效且未超时。 */
    bool shoot_armed;       /* 本次在线期间右拨杆已真实切换过档位。 */
    bool right_up;          /* 右拨杆当前是否上档，供单发边沿判定。 */
    bool spin_enabled;      /* 左下且右上时才允许底盘自旋。 */
} RemoteState_t;

/* 仅通信任务写入；断联时立即清零通道并切安全态。 */
void RemoteState_Init(void);
void RemoteState_Update(const Communication_RcControl_t *control, bool online);
/* 控制任务原子读取同一份模式与通道快照。 */
void RemoteState_Get(RemoteState_t *state);

#endif /* REMOTE_STATE_H */
