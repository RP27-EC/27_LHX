#ifndef UP_APPLICATION_CONFIG_H
#define UP_APPLICATION_CONFIG_H

#include <stdint.h>

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t period_ticks; // 上板接收解析板间遥控数据的任务周期，默认 4 ms。
} UpperCommunicationTaskConfig;
extern volatile UpperCommunicationTaskConfig upper_communication_task_config;
void UpperCommunicationTaskConfig_Init(void);

typedef struct
{
    float down_direction; // 当前下降方向；反向改为 +1。
    float down_speed_rad_s; // 下降转子目标速度，rad/s。
    float up_speed_rad_s; // 上升转子目标速度，rad/s。
    float max_rotor_turns; // 上电零点起，正负方向各允许的最大转子圈数。
    int32_t limit_stop_margin_counts; // 靠近行程边界的停机余量，编码器计数。
    float yaw_deadzone_deg; // 升降时 Yaw 偏离开机归中零点的最大角度。
    uint32_t yaw_stable_ms; // Yaw 回到死区后连续稳定的时间，须超过此值。
    uint32_t stop_retry_ms; // 停机帧重发间隔。
    uint32_t fault_stop_retry_ms; // 故障后零电流帧重发间隔。
    int32_t stall_progress_counts; // 堵转时间窗内的最小编码器位移。
    int32_t down_stall_current_raw; // 下降堵转电流门槛；另有位移保护。
    int32_t down_stall_speed_rpm; // 下降堵转速度门槛，转子 rpm。
    uint32_t down_stall_time_ms; // 下降堵转判据持续时间。
    int32_t up_stall_current_raw; // 上升堵转电流门槛；另有位移保护。
    int32_t up_stall_speed_rpm; // 上升堵转速度门槛，转子 rpm。
    uint32_t up_stall_time_ms; // 上升堵转判据持续时间。
    float calibrate_up_speed_rad_s; // 找顶部时的转子速度，低于正常位控速度。
    uint32_t calibrate_timeout_ms; // 找顶部的超时时间。
    float top_clearance_turns; // 碰顶点到后续高位目标的预留圈数；校准时不回退。
    float travel_turns; // 碰顶点到低位目标的转子圈数。
    float position_kp_rad_s_per_turn; // 每圈位置误差对应的转子目标速度。
    float position_min_speed_rad_s; // 未到位时克服静摩擦的最小目标速度。
    float hold_speed_rad_s; // 位置保持被外力推开后的最大回位速度。
    int32_t position_tolerance_counts; // 位控到位允许的编码器误差。
    uint32_t lock_tx_period_ms; // C2 锁车请求发送周期。
    uint32_t chassis_lock_settle_ms; // 发出锁车请求后的等待时间。
    int32_t chassis_release_rpm; // 转子速度低于此值才解除底盘锁车。
    uint32_t offline_release_ms; // 电机回传丢失后维持锁车的时间。
} LiftConfig;
extern volatile LiftConfig lift_config;
void LiftConfig_Init(void);

typedef struct
{
    uint32_t control_period_ticks; // 发射任务控制周期，当前 4 ms。
    int32_t fric_target_speed_rpm; // 两轮共同目标速度幅值，rpm。
    int64_t dial_feed_direction; // 正值代表逆时针上弹；改为 -1 反转。
    float dial_continuous_rounds_per_s; // 连发拨盘目标圈速，圈/秒。
    int64_t dial_arrived_error_counts; // |目标-反馈|≤500 计数视为到位，约 2.75°。
    uint32_t dial_single_move_timeout_ms; // 单发未到位时最多等待的时长。
    int32_t dial_block_current_threshold; // 堵转判据：|反馈电流原始码|须大于此值。
    int32_t dial_block_speed_threshold_dps; // 堵转判据：|反馈速度|须小于此值，deg/s。
    uint32_t dial_block_confirm_ticks; // 堵转判据连续满足的控制周期数；4 ms×50=200 ms。
    uint32_t dial_stuck_reverse_timeout_ms; // 堵转后反向退让的最长时间。
    uint32_t dial_stuck_reload_timeout_ms; // 退让后重试原目标的最长时间。
    uint32_t dial_safe_stop_retry_ms; // 安全态零电流帧重发间隔。
    uint32_t mouse_continuous_threshold_ms; // 键鼠左键按住超过 200 ms 才切入连发；此前松开为单发。
} ShootConfig;
extern volatile ShootConfig shoot_config;
void ShootConfig_Init(void);

typedef struct
{
    uint32_t control_period_ticks; // 云台与升降任务控制周期，当前 4 ms。
    int32_t rc_speed_enter; // 静止转运动的摇杆阈值，原始通道值。
    int32_t rc_speed_exit; // 运动转保持的较小阈值，构成滞回。
    int32_t pitch_max_speed_raw; // Pitch 满杆速度目标，电机速度原始码。
    float yaw_command_rate_deg_s; // 满杆目标角变化率；每周期增量=ch/660×设定角速度×控制周期。
    float yaw_rc_direction; // 摇杆方向系数；调用处已将 ch[0] 取反。
    float yaw_angle_kp; // Yaw 角度误差 deg 到目标 deg/s 的比例增益。
    float yaw_angle_ki; // 角度外环积分增益。
    float yaw_angle_kd; // 角度外环微分增益。
    float yaw_angle_integral_limit; // 角度外环积分项绝对值上限。
    float yaw_rate_target_limit_deg_s; // 角度外环输出角速度目标上限，deg/s。
    float yaw_rate_kp; // IMU 角速度误差到 4310 转矩码的比例增益。
    float yaw_rate_ki; // IMU 角速度内环积分增益。
    float yaw_rate_kd; // IMU 角速度内环微分增益。
    float yaw_rate_integral_limit; // 角速度内环积分项限幅；0 不保留积分贡献。
    float yaw_torque_limit_raw; // 角速度内环输出转矩码上限。
    float mechanical_yaw_near_deg; // 机械模式目标误差小于此角度时使用近点 PID。
    float mechanical_yaw_deadzone_deg; // 机械模式距前/后目标不超过此角度时停止位置纠偏。
    float yaw_home_rad; // Yaw 指向车头的电机单圈角，rad。
    float pitch_home_rad; // Pitch 归中时电机单圈角，rad。
    float home_tolerance_deg; // 两轴位置需落在归中目标 ±1°。
    int32_t home_speed_raw_max; // 两轴速度原始码绝对值须不超过此值。
    uint32_t home_stable_cycles; // 连续合格周期数；4 ms×5=20 ms。
    float pitch_min_deg; // 相对归中点的 Pitch 下限，度。
    float lift_pitch_clearance_deg; // 升降下降/低位时 Pitch 的正角度控制余量。
    float pitch_max_deg; // 相对归中点的 Pitch 上限，度。
    float pitch_limit_slow_deg; // 距限位不足 5° 时按剩余距离线性减速。
    int32_t turn_wheel_trigger_raw; // ch[4]≤-200 视为向上拨到触发位。
    int32_t turn_wheel_rearm_raw; // ch[4]>-50 时重新允许下一次触发。
    int32_t spin_wheel_trigger_raw; // 拨轮正向越过此值切换小陀螺。
    int32_t spin_wheel_rearm_raw; // 拨轮回中位后才能再次切换。
    float front_switch_deg; // |机械 Yaw 角|≥90° 时车尾更接近云台指向。
    float turn_tolerance_deg; // Yaw 距反向车头目标的到位角差。
    int32_t turn_speed_raw_max; // 到位还要求 |Yaw 电机速度原始码|≤20。
    uint32_t turn_stable_cycles; // 连续到位周期数；4 ms×5=20 ms。
    float pitch_gravity_center_rad; // 余弦重力曲线中心角，rad。
    float pitch_gravity_k; // 余弦项幅值，N·m。
    float pitch_gravity_b; // 恒定转矩偏置，N·m。
    float pitch_gravity_scale; // 总体前馈比例；减小可降低整条重力曲线幅值。
} CloudConfig;
extern volatile CloudConfig cloud_config;
void CloudConfig_Init(void);

typedef struct
{
    int32_t move_raw; // WASD 普通移动的虚拟摇杆幅值。
    int32_t sprint_raw; // Shift 加速移动的虚拟摇杆幅值。
    int32_t slow_raw; // Ctrl 精细移动的虚拟摇杆幅值。
    int32_t mouse_yaw_gain; // 鼠标 X 每计数换算的虚拟 Yaw 通道值。
    int32_t mouse_pitch_gain; // 鼠标 Y 每计数换算的 Pitch 通道值；方向已相对上一版反转。
    int32_t aim_divisor; // 按住鼠标右键时视角输入减半。
} KeyboardSensitivityConfig;
extern volatile KeyboardSensitivityConfig keyboard_sensitivity_config;
void KeyboardSensitivityConfig_Init(void);

void UpperApplicationConfig_InitAll(void);

#endif // UP_APPLICATION_CONFIG_H
