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

// 各任务的初始化、单周期逻辑和节拍均封装在对应入口中。
void UpperTasks_RunCommunication(void);
void UpperTasks_RunControl(void);
void UpperTasks_RunShoot(void);

#endif
