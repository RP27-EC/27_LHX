#ifndef UP_APPLICATION_CONFIG_H
#define UP_APPLICATION_CONFIG_H

#include <stdint.h>
#include "shoot_heat.h"

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t period_ticks; // 上板遥控解析任务周期，tick。
} UpperCommunicationTaskConfig;
extern volatile UpperCommunicationTaskConfig upper_communication_task_config; // 上板通信任务周期参数。
void UpperCommunicationTaskConfig_Init(void);

typedef struct
{
    float down_direction; // 下降方向系数，符号决定编码器增减方向。
    float down_speed_rad_s; // 下降转子目标速度，rad/s。
    float up_speed_rad_s; // 上升转子目标速度，rad/s。
    float max_rotor_turns; // 上电零点起，正负方向各允许的最大转子圈数。
    int32_t limit_stop_margin_counts; // 靠近行程边界的停机余量，编码器计数。
    uint32_t stop_retry_ms; // 停机帧重发间隔。
    uint32_t fault_stop_retry_ms; // 故障后零电流帧重发间隔。
} LiftMotionConfig;

typedef struct
{
    float yaw_deadzone_deg; // 升降时 Yaw 偏离开机归中零点的最大角度。
    uint32_t yaw_stable_ms; // Yaw 回到死区后连续稳定的时间，须超过此值。
    float bottom_mode_enter_turns; // 距低位目标不超过此范围时强制机械模式，转子圈。
    float bottom_mode_exit_turns; // 上升离开低位超过此范围后解除联锁，需大于进入范围。
    float special_enter_from_top_turns; // 距机械顶点的特殊动作放行范围，转子圈。
    float special_exit_from_top_turns; // 离开顶部安全区的撤销范围，需大于进入范围。
    int32_t special_down_speed_enter_rpm; // 实测向下转速超过此值时撤销特殊模式。
    uint32_t special_down_motion_confirm_ms; // 无下降指令时，持续下行多久才判定为真实下降。
    int32_t special_down_speed_release_rpm; // 转速回落到此值以下才开始解除下降锁定。
    uint32_t special_down_stop_stable_ms; // 下降结束后解除联锁所需的连续稳定时间，ms。
    uint32_t special_state_timeout_ms; // 上板安全快照超时，超时后一律禁止。
    uint32_t lock_tx_period_ms; // C2 锁车请求发送周期。
    uint32_t chassis_lock_settle_ms; // 发出锁车请求后的等待时间。
    int32_t chassis_release_rpm; // 转子速度低于此值才解除底盘锁车。
    uint32_t offline_release_ms; // 电机回传丢失后维持锁车的时间。
} LiftSafetyConfig;

typedef struct
{
    int32_t stall_progress_counts; // 堵转时间窗内的最小编码器位移。
    int32_t down_stall_current_raw; // 下降堵转电流门槛；另有位移保护。
    int32_t down_stall_speed_rpm; // 下降堵转速度门槛，转子 rpm。
    uint32_t down_stall_time_ms; // 下降堵转判据持续时间。
    int32_t up_stall_current_raw; // 上升堵转电流门槛；另有位移保护。
    int32_t up_stall_speed_rpm; // 上升堵转速度门槛，转子 rpm。
    uint32_t up_stall_time_ms; // 上升堵转判据持续时间。
} LiftStallConfig;

typedef struct
{
    float up_speed_rad_s; // 找顶部时的转子速度，低于正常位控速度。
    uint32_t timeout_ms; // 找顶部的超时时间。
    float travel_turns; // 碰顶基准到低位目标的转子行程，圈。
    float top_hold_offset_turns; // 正常高位距机械顶点向下的偏移，转子圈。
    float top_contact_window_turns; // 原顶部附近允许重新确认碰顶的范围，转子圈。
} LiftCalibrationConfig;

typedef struct
{
    float position_kp_rad_s_per_turn; // 每圈位置误差对应的转子目标速度。
    float position_min_speed_rad_s; // 主动升降克服静摩擦的最小目标速度，保持时不用。
    float hold_speed_rad_s; // 位置保持被外力推开后的最大回位速度。
    int32_t position_tolerance_counts; // 结束主动升降的误差范围，范围内仍做位置保持。
    int32_t top_arrival_tolerance_counts; // 顶部结束主动升降的容差，不清除位置保持误差。
} LiftHoldConfig;

typedef struct
{
    LiftMotionConfig motion; // 基础运动参数。
    LiftSafetyConfig safety; // 升降联锁参数。
    LiftStallConfig stall; // 堵转检测参数。
    LiftCalibrationConfig calibration; // 顶部校准参数。
    LiftHoldConfig hold; // 位置保持参数。
} LiftConfig;
extern volatile LiftConfig lift_config; // 升降行程、动作与联锁参数。
void LiftConfig_Init(void);

typedef struct
{
    int32_t target_speed_rpm; // 两轮共同目标速度幅值，rpm。
    int32_t block_speed_rpm; // 堵转时的低速门槛，转子 rpm。
    int32_t block_current_raw; // 堵转反馈电流门槛，原始码。
    uint32_t block_confirm_ms; // 低速高电流需持续的时间。
    uint32_t startup_grace_ms; // 开轮后的启动检测等待时间。
    int32_t boost_current_raw; // 正向恢复脉冲电流幅值，受驱动限幅。
    uint32_t boost_duration_ms; // 单次大电流脉冲时长。
    uint32_t recovery_wait_ms; // 脉冲后恢复速度环的观察时间。
    uint32_t recovery_max_attempts; // 一次开启期间允许的恢复次数。
} FrictionConfig;

typedef struct
{
    uint16_t hold_encoder; // 拨盘固定保持的单圈编码器位置。
    int64_t feed_direction; // 供弹方向系数，改变符号反转方向。
    int64_t arrived_error_counts; // 单发拨盘到位允许的编码器误差，计数。
    uint32_t single_move_timeout_ms; // 单发未到位时最多等待的时长。
    int32_t block_current_threshold; // 堵转判据：|反馈电流原始码|须大于此值。
    int32_t block_speed_threshold_dps; // 堵转判据：|反馈速度|须小于此值，deg/s。
    uint32_t block_confirm_ticks; // 堵转判据需连续满足的控制周期数。
    uint32_t stuck_reverse_timeout_ms; // 堵转后反向退让的最长时间。
    uint32_t stuck_reload_timeout_ms; // 退让后重试原目标的最长时间。
    uint32_t safe_stop_retry_ms; // 失联零电流帧重发间隔。
} DialFeedConfig;

typedef struct
{
    uint32_t control_period_ticks; // 对应控制任务的执行周期，tick。
    FrictionConfig friction; // 摩擦轮参数。
    DialFeedConfig dial; // 拨盘参数。
    uint32_t mouse_continuous_threshold_ms; // 鼠标按住转为连发的时间门槛，此前松开记为单发，ms。
} ShootConfig;
extern volatile ShootConfig shoot_config; // 发射动作和堵转恢复参数。
void ShootConfig_Init(void);
extern volatile ShootHeatConfig shoot_heat_config; // 热量阈值、射频与失联策略。
void ShootHeatConfig_Init(void);

typedef struct
{
    float tolerance_deg; // 云台两轴机械归中的位置误差范围，度。
    int32_t speed_raw_max; // 两轴速度原始码绝对值须不超过此值。
    uint32_t stable_cycles; // 归中位置与速度需连续合格的控制周期数。
} GimbalHomeConfig;

typedef struct
{
    float kp; // Yaw 角度误差 deg 到目标 deg/s 的比例增益。
    float ki; // 角度外环积分增益。
    float kd; // 角度外环微分增益。
    float integral_limit; // 角度外环积分项绝对值上限。
    float output_limit; // 角度外环输出角速度目标上限，deg/s。
} YawInertialAngleConfig;

typedef struct
{
    float kp; // IMU 角速度误差到 4310 转矩码的比例增益。
    float ki; // IMU 角速度内环积分增益。
    float kd; // IMU 角速度内环微分增益。
    float integral_limit; // 角速度内环积分项限幅；0 不保留积分贡献。
    float output_limit; // 角速度内环输出转矩码上限。
} YawInertialRateConfig;

typedef struct
{
    YawInertialAngleConfig angle; // 角度环参数。
    YawInertialRateConfig rate; // 陀螺仪速度环参数。
} YawInertialConfig;

typedef struct
{
    float kp; // 陀螺仪速度环，角速度误差 °/s -> 转矩码。
    float ki; // 机械 Yaw 陀螺仪速度环积分增益。
    float kd; // 机械 Yaw 陀螺仪速度环微分增益。
    float integral_limit; // 机械 Yaw 速度环积分项限幅。
    float output_limit; // 机械 Yaw 速度环转矩输出限幅，原始码。
} YawMechanicalRateConfig;

typedef struct
{
    float deadzone_deg; // 机械 Yaw 连续位置死区，内部保留速度环阻尼。
    float brake_speed_at_1deg_raw; // 单位角度误差对应的回正速度码上限，按误差平方根缩小；零关闭。
    float command_deg_s_per_raw; // 位置环速度原始码到目标角速度的换算系数，度/s/码。
    float gyro_direction; // IMU Yaw 与编码器正方向同向 +1，反向 -1。
    float chassis_rate_ff_gain; // 底盘角速度到机械 Yaw 目标的前馈增益，负值用于反向，零关闭。
    float chassis_rate_ff_limit_deg_s; // 前馈角速度限幅，避免异常反馈造成目标突变。
    YawMechanicalRateConfig rate; // 陀螺仪速度环参数。
} YawMechanicalConfig;

typedef struct
{
    int32_t wheel_trigger_raw; // 拨轮负向达到此阈值时触发调头，遥控原始值。
    int32_t wheel_rearm_raw; // 拨轮回到复位范围后允许下一次调头，遥控原始值。
    float duration_s; // 调头目标轨迹时长，s；到位后再结束动作。
    float tolerance_deg; // Yaw 距反向车头目标的到位角差。
    int32_t speed_raw_max; // 调头到位允许的反馈速度绝对值，电机原始码。
    uint32_t stable_cycles; // 调头位置与速度需连续合格的控制周期数。
} YawTurnConfig;

typedef struct
{
    float command_rate_deg_s; // 满杆目标角变化率，按输入比例和控制周期累加，度/s。
    float rc_direction; // 摇杆方向系数；调用处已将 ch[0] 取反。
    float home_rad; // Yaw 指向车头的电机单圈角，rad。
    YawInertialConfig inertial; // 惯性模式参数。
    YawMechanicalConfig mechanical; // 机械模式参数。
    YawTurnConfig turn; // 调头参数。
    float front_switch_deg; // 用于选择更接近云台指向的车头或车尾，度。
} YawConfig;

typedef struct
{
    int32_t wheel_trigger_raw; // 拨轮正向越过此值切换小陀螺。
    int32_t wheel_rearm_raw; // 拨轮回中位后才能再次切换。
    uint32_t fault_rearm_ms; // 自旋许可连续丢失超过此时间才锁存重新拨档。
} GimbalSpinConfig;

typedef struct
{
    float kp; // Pitch 惯性位置环比例增益。
    float ki; // Pitch 惯性位置环积分增益。
    float kd; // Pitch 惯性位置环微分增益。
    float integral_limit; // Pitch 惯性位置环积分项限幅。
    float output_limit; // Pitch 位置环目标角速度限幅，度/s。
} PitchInertialAngleConfig;

typedef struct
{
    float kp; // Pitch 陀螺仪速度环比例增益。
    float ki; // Pitch 陀螺仪速度环积分增益。
    float kd; // Pitch 陀螺仪速度环微分增益。
    float integral_limit; // Pitch 速度环积分项限幅。
    float output_limit; // Pitch 速度环转矩输出限幅，原始码。
} PitchInertialRateConfig;

typedef struct
{
    PitchInertialAngleConfig angle; // 角度环参数。
    PitchInertialRateConfig rate; // 陀螺仪速度环参数。
} PitchInertialConfig;

typedef struct
{
    float imu_direction; // IMU Pitch 与电机正方向的对应系数，符号决定方向。
    float rate_filter_alpha; // Pitch 角速度低通的新样本权重，越大响应越快、滤波越弱。
    PitchInertialConfig inertial; // 惯性模式参数。
    float command_rate_deg_s; // Pitch 满杆位置目标变化率，度/s。
    float home_rad; // Pitch 归中时电机单圈角，rad。
    float min_deg; // 相对归中点的 Pitch 下限，度。
    float max_deg; // 相对归中点的 Pitch 上限，度。
    float lift_clearance_deg; // 升降下降/低位时 Pitch 的正角度控制余量。
    float target_lead_deg; // 目标允许领先实际位置的最大角度，度。
    float gravity_k; // Pitch 重力前馈系数，N·m；前馈=k*cos(相对机械零点角)。
} PitchConfig;

typedef struct
{
    uint32_t control_period_ticks; // 对应控制任务的执行周期，tick。
    int32_t rc_speed_enter; // 摇杆输入死区，原始通道值。
    GimbalHomeConfig home; // 开机归中参数。
    YawConfig yaw; // Yaw参数。
    GimbalSpinConfig spin; // 小陀螺参数。
    PitchConfig pitch; // Pitch参数。
} CloudConfig;
extern volatile CloudConfig cloud_config; // 云台控制与调头参数。
void CloudConfig_Init(void);

typedef struct
{
    int32_t move_raw; // WASD 普通移动的虚拟摇杆幅值。
    int32_t sprint_raw; // Shift 加速移动的虚拟摇杆幅值。
    int32_t slow_raw; // Ctrl 精细移动的虚拟摇杆幅值。
    int32_t mouse_yaw_gain; // 鼠标 X 每计数换算的虚拟 Yaw 通道值。
    int32_t mouse_pitch_gain; // 鼠标 Y 每计数换算的 Pitch 通道值；符号控制俯仰方向。
    int32_t aim_divisor; // 按住鼠标右键时按除数降低视角输入。
} KeyboardSensitivityConfig;
extern volatile KeyboardSensitivityConfig keyboard_sensitivity_config; // 键鼠移动和瞄准灵敏度参数。
void KeyboardSensitivityConfig_Init(void);

// 分类默认值，可单独恢复某一组参数。
void LiftMotionConfig_Init(volatile LiftMotionConfig *config);
void LiftSafetyConfig_Init(volatile LiftSafetyConfig *config);
void LiftStallConfig_Init(volatile LiftStallConfig *config);
void LiftCalibrationConfig_Init(volatile LiftCalibrationConfig *config);
void LiftHoldConfig_Init(volatile LiftHoldConfig *config);
void FrictionConfig_Init(volatile FrictionConfig *config);
void DialFeedConfig_Init(volatile DialFeedConfig *config);
void GimbalHomeConfig_Init(volatile GimbalHomeConfig *config);
void YawInertialAngleConfig_Init(volatile YawInertialAngleConfig *config);
void YawInertialRateConfig_Init(volatile YawInertialRateConfig *config);
void YawInertialConfig_Init(volatile YawInertialConfig *config);
void YawMechanicalRateConfig_Init(volatile YawMechanicalRateConfig *config);
void YawMechanicalConfig_Init(volatile YawMechanicalConfig *config);
void YawTurnConfig_Init(volatile YawTurnConfig *config);
void YawConfig_Init(volatile YawConfig *config);
void GimbalSpinConfig_Init(volatile GimbalSpinConfig *config);
void PitchInertialAngleConfig_Init(volatile PitchInertialAngleConfig *config);
void PitchInertialRateConfig_Init(volatile PitchInertialRateConfig *config);
void PitchInertialConfig_Init(volatile PitchInertialConfig *config);
void PitchConfig_Init(volatile PitchConfig *config);

void UpperApplicationConfig_InitAll(void);

#endif // UP_APPLICATION_CONFIG_H
