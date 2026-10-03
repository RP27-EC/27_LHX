#ifndef LIFT_CONTROL_H
#define LIFT_CONTROL_H

#include "remote_state.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_STOPPED = 0,
    LIFT_DESCENDING,
    LIFT_ASCENDING,
    LIFT_STALLED,
    LIFT_AT_LIMIT,
    LIFT_READY,
    LIFT_CALIBRATING_UP
} LiftControl_State_t;

typedef enum
{
    LIFT_WAIT_NONE = 0,
    LIFT_WAIT_REMOTE,
    LIFT_WAIT_YAW,
    LIFT_WAIT_MOTOR,
    LIFT_WAIT_CHASSIS_LOCK,
    LIFT_WAIT_FAULT,
    LIFT_WAIT_PITCH,
    LIFT_WAIT_SAFETY_STATE
} LiftControl_WaitReason_t;

typedef struct
{
    bool valid; // 是否记录过有效堵转快照。
    LiftControl_State_t direction; // 堵转时的运动方向。
    uint16_t encoder; // 堵转瞬间单圈编码器值。
    int32_t encoder_total; // 堵转瞬间上电累计计数。
    float rotor_turns; // encoder_total / 8192，转子圈数。
    uint32_t time_ms; // 堵转快照记录时间，ms。
} LiftControl_StallSnapshot_t;

typedef struct
{
    float from_top_turns; // 当前编码器距校准机械顶点的转子圈数，向下为正。
    bool position_valid; // 校准完成、2006 在线且累计位置处于行程内。
    bool upper_zone; // 已进入顶端放行区，门槛由升降应用参数设置。
    bool bottom_mode_blocked; // 接近低位时强制机械 Yaw，并通知下板退出跟随/小陀螺。
    bool yaw_home_required; // 主动升降或新请求要求 Yaw 回开机机械零点，到位后解除。
    bool descending; // 收到下降目标或实测持续下行。
    bool special_allowed; // 顶部位置及两轴状态允许调头。
    bool spin_allowed; // 小陀螺额外要求 IMU 可用。
    bool shoot_allowed; // 发射位置许可；仍需单独的手动布防。
    uint32_t update_ms; // 本快照生成时间。
} LiftSafetyState_t;

extern volatile LiftControl_State_t lift_control_state; // 升降当前动作状态。
extern volatile LiftControl_WaitReason_t lift_wait_reason; // 当前等待或停机原因。
extern volatile bool lift_calibrated; // 顶部基准和行程目标是否已建立。
extern volatile int32_t lift_top_encoder_total; // 高位累计位置目标。
extern volatile int32_t lift_top_contact_encoder_total; // 碰顶瞬间的累计编码器物理基准。
extern volatile int32_t lift_bottom_encoder_total; // 低位累计位置目标。
extern volatile bool lift_pitch_nonnegative_required; // Pitch 是否需抬至机械安全下限。
extern volatile LiftSafetyState_t lift_safety_state; // 调试器可直接查看整车安全快照。

void LiftControl_Init(void);
void LiftControl_Update(const RemoteState_t *remote);
void LiftControl_SafetyUpdate(const RemoteState_t *remote); // 云台控制前按同一遥控快照计算。
bool LiftControl_SafetyGet(LiftSafetyState_t *state); // 超时返回 false，调用方须停止特殊动作。
bool LiftControl_GetStallSnapshot(LiftControl_StallSnapshot_t *snapshot);

#endif // LIFT_CONTROL_H
