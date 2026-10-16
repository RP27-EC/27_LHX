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
    LIFT_WAIT_SAFETY_STATE,
    LIFT_WAIT_TRAJECTORY_CONFIG // 轨迹时间或速度参数无效，取消本次动作。
} LiftControl_WaitReason_t;

typedef struct
{
    bool valid; // 是否记录过有效堵转快照。
    LiftControl_State_t direction; // 堵转时的运动方向。
    uint16_t encoder; // 堵转瞬间单圈编码器值。
    int32_t encoder_total; // 堵转瞬间上电累计计数。
    int16_t speed_rpm; // 堵转时实测转速。
    int16_t current_raw; // 堵转时实测电流码。
    LiftControl_State_t requested_direction; // 堵转时的主动请求，不包含保持纠偏。
    int32_t target_encoder_total; // 堵转时正在执行的位置目标。
    int32_t top_contact_encoder_total; // 堵转时采用的机械顶部基准。
    float from_top_turns; // 堵转瞬间距机械顶部，向下为正。
    float rotor_turns; // encoder_total / 8192，转子圈数。
    uint32_t time_ms; // 堵转快照记录时间，ms。
} LiftControl_StallSnapshot_t;

typedef struct
{
    float from_top_turns; // 当前编码器距校准机械顶点的转子圈数，向下为正。
    bool position_valid; // 校准完成、2006 在线且累计位置处于行程内。
    bool upper_zone; // 已进入顶端放行区，门槛由升降应用参数设置。
    bool bottom_mode_blocked; // 接近低位时强制机械 Yaw，并通知下板退出跟随/小陀螺。
    bool yaw_home_required; // 下降、顶部区外上升或新请求要求 Yaw 回机械零点。
    bool descending; // 收到下降目标或实测持续下行。
    bool special_allowed; // 顶部位置及两轴状态允许调头。
    bool spin_allowed; // 小陀螺额外要求 IMU 可用。
    bool shoot_allowed; // 发射位置许可；仍需单独的手动布防。
    uint32_t update_ms; // 本快照生成时间。
} LiftSafetyState_t;

typedef struct
{
    bool active; // 正在执行轨迹或等待终点到位。
    int32_t start_encoder_total; // 本次轨迹的实测起点。
    int32_t end_encoder_total; // 本次轨迹的最终位置。
    int32_t target_encoder_total; // 当前五次曲线位置目标。
    float speed_ff_rad_s; // 曲线导数换算的转子速度前馈。
    float duration_s; // 本次采用的轨迹时间，含速度上限约束。
    float elapsed_s; // 已成功发送的轨迹时间。
    float progress; // 已执行时间占本次时长的比例。
    uint32_t last_update_ms; // 上次轨迹尝试发送时间。
} LiftTrajectoryState_t;
extern volatile LiftTrajectoryState_t lift_trajectory; // 升降五次轨迹观察量。

extern volatile LiftControl_State_t lift_control_state; // 升降当前动作状态。
extern volatile LiftControl_WaitReason_t lift_wait_reason; // 当前等待或停机原因。
extern volatile bool lift_calibrated; // 顶部基准和行程目标是否已建立。
extern volatile int32_t lift_top_encoder_total; // 高位累计位置目标。
extern volatile int32_t lift_top_contact_encoder_total; // 碰顶瞬间的累计编码器物理基准。
extern volatile int32_t lift_bottom_encoder_total; // 低位累计位置目标。
extern volatile bool lift_pitch_nonnegative_required; // Pitch 是否需抬至机械安全下限。
extern volatile LiftSafetyState_t lift_safety_state; // 调试器可直接查看整车安全快照。

void LiftControl_Init(void);
bool LiftControl_CommandBusy(void); // 主动升降、校准或等待回零请求仍在执行。
void LiftControl_FilterRemote(RemoteState_t *remote); // 动作期间锁存升降请求，忽略模式切换。
void LiftControl_Update(const RemoteState_t *remote);
void LiftControl_SafetyUpdate(const RemoteState_t *remote); // 云台控制前按同一遥控快照计算。
bool LiftControl_SafetyGet(LiftSafetyState_t *state); // 超时返回 false，调用方须停止特殊动作。
bool LiftControl_GetStallSnapshot(LiftControl_StallSnapshot_t *snapshot);

#endif // LIFT_CONTROL_H
