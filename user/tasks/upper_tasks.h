#ifndef UPPER_TASKS_H
#define UPPER_TASKS_H

#include <stdint.h>

// 任务超过设定周期的累计次数，供 Keil Watch 检查时序。
typedef struct
{
    uint32_t communication_overruns; // 板间通信任务超期次数。
    uint32_t gimbal_overruns; // 云台与升降控制任务超期次数。
    uint32_t shoot_overruns; // 发射控制任务超期次数。
} UpperTaskTimingState;

extern volatile UpperTaskTimingState upper_task_timing;

typedef enum
{
    UPPER_SHOOT_BLOCK_NONE = 0, // 发射许可有效。
    UPPER_SHOOT_BLOCK_REMOTE, // 遥控无效或离线。
    UPPER_SHOOT_BLOCK_LIFT, // 升降安全快照尚未放行。
    UPPER_SHOOT_BLOCK_SPIN, // 遥控拨杆小陀螺禁止发射，键鼠小陀螺允许发射。
    UPPER_SHOOT_BLOCK_REARM, // 等待新的拨杆操作或键鼠布防。
    UPPER_SHOOT_BLOCK_FRICTION, // 摩擦轮电机离线。
    UPPER_SHOOT_BLOCK_DIAL // 拨盘离线，只允许摩擦轮待发。
} UpperShootBlockReason;
extern volatile UpperShootBlockReason upper_shoot_block_reason; // Keil 可查看的发射阻止原因。

// 各任务的初始化、单周期逻辑和节拍均封装在对应入口中。
void UpperTasks_RunCommunication(void);
void UpperTasks_RunControl(void);
void UpperTasks_RunShoot(void);

#endif
