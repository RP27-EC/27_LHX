#ifndef REMOTE_STATE_H
#define REMOTE_STATE_H

#include "communication.h"
#include <stdbool.h>
#include <stdint.h>

// 左拨杆或键鼠决定整车模式；断联时安全停止。
typedef enum
{
    REMOTE_MODE_DISABLED = 0, // 断联或非法输入，所有模式停用。
    REMOTE_MODE_MECHANICAL, // 云台锁车头，底盘由输入直接转向。
    REMOTE_MODE_FOLLOW, // 云台指向由输入控制，底盘跟随云台。
    REMOTE_MODE_SPIN // 小陀螺，可按云台朝向平移。
} RemoteMode_t;

// 遥控右拨杆或键鼠 F/左键决定发射状态。
typedef enum
{
    REMOTE_SHOOT_OFF = 0, // 发射保险。
    REMOTE_SHOOT_READY, // 仅启动摩擦轮。
    REMOTE_SHOOT_SINGLE, // 单发拨盘动作。
    REMOTE_SHOOT_CONTINUOUS // 连续供弹。
} RemoteShoot_t;

typedef struct
{
    RemoteMode_t chassis; // 底盘与云台的协同模式。
    RemoteShoot_t shooting; // 发射机构的工作模式。
} RemoteModeState_t;

typedef struct
{
    int16_t channel[5]; // 同一遥控快照内的五路通道。
    uint8_t lift_right_switch; // 升降模式下的右拨杆档位。
    bool keyboard_active; // V 键选择键鼠输入。
    bool right_up; // 右拨杆上档或鼠标左键按下。
} RemoteInputState_t;

typedef struct
{
    bool online; // 遥控帧有效且未超时。
    bool shoot_armed; // 右拨杆换档或键鼠 F 布防发射。
    bool spin_enabled; // 右拨杆换档或键鼠 G 使能自旋。
    bool lift_enabled; // 左下档或键鼠机械模式允许升降。
} RemoteSafetyState_t;

typedef struct
{
    uint32_t shoot_single_request_count; // 鼠标短按产生的单发事件累计数。
    uint32_t lift_toggle_request_count; // B 键产生的升降切换事件累计数。
} RemoteEventState_t;

typedef struct
{
    RemoteModeState_t mode; // 枚举模式选择。
    RemoteInputState_t input; // 遥控器或键鼠的当前输入。
    RemoteSafetyState_t safety; // 在线状态与各机构保险标志。
    RemoteEventState_t event; // 供控制任务消费的边沿事件。
} RemoteState_t;

// 仅通信任务写入；断联时立即清零通道并切安全态。
void RemoteState_Init(void);
void RemoteState_Update(const Communication_RcControl_t *control, bool online);
// 控制任务原子读取同一份模式与通道快照。
void RemoteState_Get(RemoteState_t *state);

#endif // REMOTE_STATE_H
