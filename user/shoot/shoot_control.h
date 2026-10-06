#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "remote_state.h"
#include "shoot_speed.h"

#define SHOOT_LEFT_FRIC_MOTOR_ID     1U
#define SHOOT_RIGHT_FRIC_MOTOR_ID    2U
#define SHOOT_DIAL_ONE_BULLET_COUNTS 65536LL

typedef enum
{
    SHOOT_DIAL_IDLE = 0, // 保持配置的单圈位置，等待供弹请求。
    SHOOT_DIAL_FEED, // 向累计单发目标正向供弹。
    SHOOT_DIAL_CONTINUOUS, // 速度闭环连续供弹。
    SHOOT_DIAL_STUCK_REVERSE, // 堵转后沿反方向退让。
    SHOOT_DIAL_STUCK_RELOAD // 退让后重新追踪原供弹目标。
} ShootDialState_t;

typedef struct
{
    ShootDialState_t state; // 拨盘当前状态枚举。
    uint32_t state_start_ms; // 当前状态进入时间，ms。
    int64_t target; // 拨盘累计编码器目标。
    bool target_synced; // 已根据累计位置建立固定单圈供弹目标。
} ShootDialMotionState_t;

typedef struct
{
    uint32_t block_tick; // 堵转条件连续满足的控制周期数。
    int64_t feed_target; // 退让前保存的供弹目标。
    int8_t motion_direction; // 堵转前方向，取值为 -1 或 1。
    bool continuous; // 堵转前处于连发模式。
} ShootDialRecoveryState_t;

typedef struct
{
    bool holding; // 待机固定单圈目标已锁定。
    bool stopped; // 失联零电流命令已成功入队。
    uint32_t last_stop_ms; // 最近一次零电流命令入队时间，ms。
} ShootDialStopState_t;

typedef struct
{
    bool last_right_up; // 上周期右拨杆上档状态。
    RemoteShoot_t last_mode; // 上周期发射模式枚举。
} ShootRemoteEdgeState_t;

typedef struct
{
    uint32_t single_seen; // 已读取的键鼠短按事件序号。
    bool single_pending; // 暂存下一次短按请求。
    bool single_active; // 单发需保持位控至完成。
    bool single_started; // 拨盘已进入供弹或堵转恢复。
} ShootKeyboardSingleState_t;

typedef struct
{
    uint32_t single; // 已完成的累计单发次数。
    uint32_t stuck; // 累计触发堵转恢复的次数。
} ShootCounterState_t;

typedef enum
{
    SHOOT_FRIC_NORMAL = 0, // 正常速度控制。
    SHOOT_FRIC_BOOST, // 沿出弹方向短时电流恢复。
    SHOOT_FRIC_RECOVERY, // 恢复速度环，暂缓堵转检测。
    SHOOT_FRIC_FAULT // 重试失败停机，关闭后复位。
} ShootFricState_t;

typedef struct
{
    ShootFricState_t state; // 摩擦轮恢复状态。
    bool enabled; // 当前开启周期已开始。
    bool block_timing[2]; // 各轮的连续堵转计时状态。
    uint32_t block_start_ms[2]; // 各轮本次堵转计时起点。
    uint32_t state_start_ms; // 开轮或恢复状态的起始时间。
    uint32_t attempts; // 本次开启已使用的恢复次数。
    uint32_t stuck_count; // 累计触发恢复次数。
} ShootFricRecoveryState_t;

typedef struct
{
    ShootFricRecoveryState_t friction; // 摩擦轮堵转检测和电流脉冲。
    ShootSpeedState speed; // 弹速反馈与摩擦轮自适应目标。
    ShootDialMotionState_t dial; // 拨盘位置目标与状态机。
    ShootDialRecoveryState_t recovery; // 堵转检测与恢复过程。
    ShootDialStopState_t stop; // 待机位置保持与失联零电流重发状态。
    ShootRemoteEdgeState_t remote; // 遥控拨杆边沿和模式记忆。
    ShootKeyboardSingleState_t keyboard; // 键鼠单发事件锁存。
    ShootCounterState_t count; // 调试器可查看的累计计数。
} ShootControlState_t;

extern volatile ShootControlState_t shoot_control_state; // 发射动作、堵转恢复和事件状态。

void ShootControl_Init(void);
// 在线时待机保持配置的单圈位置；断联时输出零电流。
void ShootControl_SetIdleHoldEnabled(bool enabled);

// 随发射任务调用；单发仅在右拨杆进入上档的边沿触发。
void ShootControl_Update(RemoteShoot_t mode, bool right_up);
// 键鼠短按事件锁存至一发完成；长按沿用原连发速度环。
void ShootControl_UpdateKeyboard(RemoteShoot_t mode,
                                 uint32_t single_request_count);
// 退出键鼠模式时取消未完成的键鼠请求，后续恢复遥控拨杆逻辑。
void ShootControl_ResetKeyboard(uint32_t single_request_count);

#ifdef __cplusplus
}
#endif

#endif // SHOOT_CONTROL_H
