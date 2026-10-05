#include "application_config.h"
#include <stddef.h>

volatile UpperCommunicationTaskConfig upper_communication_task_config; // 上板通信任务周期参数。

// 加载上板遥控解析任务的默认执行周期。
void UpperCommunicationTaskConfig_Init(void)
{
    upper_communication_task_config.period_ticks = 4U; // 板间遥控接收解析周期，tick。
}

volatile LiftConfig lift_config; // 升降行程、动作与联锁参数。

// 加载升降方向、速度、行程及停机重发参数。
void LiftMotionConfig_Init(volatile LiftMotionConfig *config)
{
    if (config == NULL) { return; }
    config->down_direction = -1.0f; // 编码器计数减小表示下降。
    config->down_speed_rad_s = 1200.0f; // 正常下降的转子速度上限，rad/s。
    config->up_speed_rad_s = 1200.0f; // 正常上升的转子速度上限，rad/s。
    config->max_rotor_turns = 330.0f; // 上电后正、反向累计转数限制，圈。
    config->limit_stop_margin_counts = 200; // 行程边界前的停机余量，编码器计数。
    config->stop_retry_ms = 50U; // 普通停机命令重发间隔，ms。
    config->fault_stop_retry_ms = 5U; // 故障停机命令重发间隔，ms。
}

// 加载升降归中等待、顶部放行、低位限制及底盘锁车参数。
void LiftSafetyConfig_Init(volatile LiftSafetyConfig *config)
{
    if (config == NULL) { return; }
    config->yaw_deadzone_deg = 2.0f; // 允许升降的 Yaw 归中角度误差，度。
    config->yaw_stable_ms = 50U; // Yaw 连续处于死区内的时间，ms。
    config->bottom_mode_enter_turns = 30.0f; // 距低位目标不超过此范围时强制机械模式，转子圈。
    config->bottom_mode_exit_turns = 40.0f; // 上升离开低位超过此范围后解除联锁，需大于进入范围。
    config->special_enter_from_top_turns = 70.0f; // 距机械顶点的特殊动作放行范围，转子圈。
    config->special_exit_from_top_turns = 75.0f; // 离开顶部安全区的撤销范围，需大于进入范围。
    config->special_down_speed_enter_rpm = 30; // 明显下行才按实测转速撤销。
    config->special_down_motion_confirm_ms = 200U; // 过滤高位保持时的短暂下行纠偏。
    config->special_down_speed_release_rpm = 15; // 停止下行后解除锁定的转速门槛。
    config->special_down_stop_stable_ms = 50U; // 下降结束后解除联锁所需的连续稳定时间，ms。
    config->special_state_timeout_ms = 50U; // 控制任务失去更新时先进入安全态。
    config->lock_tx_period_ms = 10U; // 向下板发送锁车请求的间隔，ms。
    config->chassis_lock_settle_ms = 20U; // 发出锁车请求后的等待时间，ms。
    config->chassis_release_rpm = 10; // 低于该转速时可释放底盘锁定，rpm。
    config->offline_release_ms = 200U; // 电机离线后继续锁车的时间，ms。
}

// 加载升降堵转的电流、速度、位移和持续时间判据。
void LiftStallConfig_Init(volatile LiftStallConfig *config)
{
    if (config == NULL) { return; }
    config->stall_progress_counts = 40; // 堵转时间窗内要求的最小位移，计数。
    config->down_stall_current_raw = 75; // 下降堵转电流阈值，反馈原始码。
    config->down_stall_speed_rpm = 1; // 下降堵转低速阈值，rpm。
    config->down_stall_time_ms = 200U; // 下降堵转确认时间，ms。
    config->up_stall_current_raw = 670; // 上升堵转电流阈值，反馈原始码。
    config->up_stall_speed_rpm = 1; // 上升堵转低速阈值，rpm。
    config->up_stall_time_ms = 450U; // 上升堵转确认时间，ms。
}

// 加载顶部校准速度、行程基准及高位保持偏移。
void LiftCalibrationConfig_Init(volatile LiftCalibrationConfig *config)
{
    if (config == NULL) { return; }
    config->up_speed_rad_s = 500.0f; // 自动找顶部的转子速度，rad/s。
    config->timeout_ms = 900000U; // 向上找顶部的最长时间，ms。
    config->travel_turns = 295.0f; // 碰顶基准到低位目标的转子行程，圈。
    config->top_hold_offset_turns = 5.0f; // 正常上升留出机械顶端余量，校准碰顶不受影响。
    config->top_contact_window_turns = 10.0f; // 原顶部附近允许重新确认碰顶的范围，转子圈。
}

// 加载升降回位速度、位置误差容差和保持参数。
void LiftHoldConfig_Init(volatile LiftHoldConfig *config)
{
    if (config == NULL) { return; }
    config->position_kp_rad_s_per_turn = 5.7f; // 每圈位置误差对应的目标速度，rad/s。
    config->position_min_speed_rad_s = 8.0f; // 主动升降克服静摩擦的最小目标速度，保持时不用，rad/s。
    config->hold_speed_rad_s = 15.0f; // 外力偏离目标后回位的速度上限，rad/s。
    config->position_tolerance_counts = 200; // 结束主动升降的误差范围，范围内仍做位置保持，编码器计数。
    config->top_arrival_tolerance_counts = 20000; // 顶部结束主动升降的容差，不清除位置保持误差，编码器计数。
}

// 依次加载升降运动、安全联锁、堵转、校准和保持默认值。
void LiftConfig_Init(void)
{
    LiftMotionConfig_Init(&lift_config.motion);
    LiftSafetyConfig_Init(&lift_config.safety);
    LiftStallConfig_Init(&lift_config.stall);
    LiftCalibrationConfig_Init(&lift_config.calibration);
    LiftHoldConfig_Init(&lift_config.hold);
}

volatile ShootConfig shoot_config; // 发射动作和堵转恢复参数。

// 加载摩擦轮目标转速、堵转检测及正向脉冲恢复参数。
void FrictionConfig_Init(volatile FrictionConfig *config)
{
    if (config == NULL) { return; }
    config->target_speed_rpm = 1500; // 摩擦轮目标转速幅值，rpm。
    config->block_speed_rpm = 100; // 低于此转速才检查堵转。
    config->block_current_raw = 2000; // 排除低负载的慢速转动。
    config->block_confirm_ms = 200U; // 过滤短暂的出弹掉速。
    config->startup_grace_ms = 500U; // 避开开轮加速阶段。
    config->boost_current_raw = 14000; // 两轮沿出弹方向的恢复电流。
    config->boost_duration_ms = 400U; // 大电流只维持短时间。
    config->recovery_wait_ms = 500U; // 恢复速度环后再检测。
    config->recovery_max_attempts = 3U; // 重试用尽后关闭摩擦轮再重启。
}

// 加载拨盘保持位置、供弹方向、到位判据及堵转退让参数。
void DialFeedConfig_Init(volatile DialFeedConfig *config)
{
    if (config == NULL) { return; }
    config->hold_encoder = 31000U; // 上线和停止供弹后统一回到该单圈位置。
    config->feed_direction = 1LL; // 供弹方向系数，改变符号反转方向。
    config->arrived_error_counts = 150LL; // 单发拨盘到位允许的编码器误差，计数。
    config->single_move_timeout_ms = 500U; // 单发动作超时，ms。
    config->block_current_threshold = 600; // 拨盘堵转电流阈值，原始码。
    config->block_speed_threshold_dps = 10; // 拨盘堵转低速阈值，度/s。
    config->block_confirm_ticks = 50U; // 堵转判据需连续满足的控制周期数。
    config->stuck_reverse_timeout_ms = 200U; // 堵转反向退让超时，ms。
    config->stuck_reload_timeout_ms = 200U; // 退让后重新上弹超时，ms。
    config->safe_stop_retry_ms = 20U; // 失联零电流帧重发间隔，ms。
}

// 加载发射任务周期、鼠标连发门槛及摩擦轮、拨盘默认值。
void ShootConfig_Init(void)
{
    shoot_config.control_period_ticks = 4U; // 发射任务控制周期，tick。
    shoot_config.mouse_continuous_threshold_ms = 200U; // 鼠标按住转为连发的时间门槛，此前松开记为单发，ms。
    FrictionConfig_Init(&shoot_config.friction);
    DialFeedConfig_Init(&shoot_config.dial);
}

volatile CloudConfig cloud_config; // 云台控制与调头参数。

// 加载开机归中的位置、速度判据和稳定等待参数。
void GimbalHomeConfig_Init(volatile GimbalHomeConfig *config)
{
    if (config == NULL) { return; }
    config->tolerance_deg = 1.5f; // 云台两轴机械归中的位置误差范围，度。
    config->speed_raw_max = 20; // 归中到位时的速度原始码上限。
    config->stable_cycles = 5U; // 归中位置与速度需连续合格的控制周期数。
}

// 加载惯性 Yaw 角度外环 PID 和目标角速度限幅。
void YawInertialAngleConfig_Init(volatile YawInertialAngleConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 9.0f; // Yaw 角度外环比例增益。
    config->ki = 0.0f; // Yaw 角度外环积分增益。
    config->kd = 0.0f; // Yaw 角度外环微分增益。
    config->integral_limit = 200.0f; // 角度外环积分项限幅。
    config->output_limit = 600.0f; // 外环目标角速度上限，度/s。
}

// 加载惯性 Yaw 陀螺仪速度内环 PID 和转矩限幅。
void YawInertialRateConfig_Init(volatile YawInertialRateConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 9.0f; // Yaw 角速度内环比例增益。
    config->ki = 0.0f; // Yaw 角速度内环积分增益。
    config->kd = 0.0f; // Yaw 角速度内环微分增益。
    config->integral_limit = 0.0f; // 角速度内环积分项限幅。
    config->output_limit = 2047.0f; // Yaw 输出转矩码上限。
}

// 组合加载惯性 Yaw 的角度外环与速度内环默认值。
void YawInertialConfig_Init(volatile YawInertialConfig *config)
{
    if (config == NULL) { return; }
    YawInertialAngleConfig_Init(&config->angle);
    YawInertialRateConfig_Init(&config->rate);
}

// 加载机械 Yaw 陀螺仪速度环 PID 和转矩限幅。
void YawMechanicalRateConfig_Init(volatile YawMechanicalRateConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 8.0f; // 机械 Yaw 独立陀螺仪速度环。
    config->ki = 0.0f; // 机械 Yaw 陀螺仪速度环积分增益。
    config->kd = 0.0f; // 机械 Yaw 陀螺仪速度环微分增益。
    config->integral_limit = 0.0f; // 机械 Yaw 速度环积分项限幅。
    config->output_limit = 1800.0f; // 机械 Yaw 速度环转矩输出限幅，原始码。
}

// 加载机械 Yaw 死区、回正制动、底盘前馈及陀螺仪速度环参数。
void YawMechanicalConfig_Init(volatile YawMechanicalConfig *config)
{
    if (config == NULL) { return; }
    config->deadzone_deg = 0.4f; // 连续位置死区，内部保留速度环制动。
    config->brake_speed_at_1deg_raw = 60.0f; // 单位角度误差对应的回正速度码上限，按误差平方根缩小；零关闭。
    config->command_deg_s_per_raw = 1.0f; // 位置环速度原始码到目标角速度的换算系数，度/s/码。
    config->gyro_direction = 1.0f; // 与现有惯性 Yaw 使用同一 IMU 正方向。
    config->chassis_rate_ff_gain = 1.0f; // 跟随底盘转速的前馈增益，符号按两板 IMU 方向设置。
    config->chassis_rate_ff_limit_deg_s = 50.0f; // 叠加到速度目标的前馈限幅，度/s。
    YawMechanicalRateConfig_Init(&config->rate);
}

// 加载 Yaw 调头拨轮门槛、轨迹时长和到位稳定判据。
void YawTurnConfig_Init(volatile YawTurnConfig *config)
{
    if (config == NULL) { return; }
    config->wheel_trigger_raw = 200; // 拨轮负向达到此阈值时触发调头，遥控原始值。
    config->wheel_rearm_raw = 50; // 拨轮回到复位范围后允许下一次调头，遥控原始值。
    config->duration_s = 0.78f; // 调头时间，越短目标变化越快。
    config->tolerance_deg = 2.0f; // 调头目标角的到位误差，度。
    config->speed_raw_max = 20; // 调头到位允许的反馈速度绝对值，电机原始码。
    config->stable_cycles = 5U; // 调头位置与速度需连续合格的控制周期数。
}

// 加载 Yaw 输入方向与机械零点，并组合加载各模式默认值。
void YawConfig_Init(volatile YawConfig *config)
{
    if (config == NULL) { return; }
    config->command_rate_deg_s = 200.0f; // Yaw 满杆目标角变化率，度/s。
    config->rc_direction = 1.0f; // Yaw 摇杆输入方向系数。
    config->home_rad = (-0.387884378f); // Yaw 指向车头时的电机单圈角，rad。
    config->front_switch_deg = 90.0f; // 用于选择更接近云台指向的车头或车尾，度。
    YawInertialConfig_Init(&config->inertial);
    YawMechanicalConfig_Init(&config->mechanical);
    YawTurnConfig_Init(&config->turn);
}

// 加载小陀螺拨轮触发、复位及故障后重新布防参数。
void GimbalSpinConfig_Init(volatile GimbalSpinConfig *config)
{
    if (config == NULL) { return; }
    config->wheel_trigger_raw = 200; // 拨轮切换小陀螺的触发阈值。
    config->wheel_rearm_raw = 50; // 拨轮切换小陀螺的复位阈值。
    config->fault_rearm_ms = 100U; // 短暂许可波动立即停转，但不锁存；持续失效需重新拨档。
}

// 加载惯性 Pitch 角度外环 PID 和目标角速度限幅。
void PitchInertialAngleConfig_Init(volatile PitchInertialAngleConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 9.0f; // Pitch 惯性位置环比例增益。
    config->ki = 0.0f; // Pitch 惯性位置环积分增益。
    config->kd = 0.0f; // Pitch 惯性位置环微分增益。
    config->integral_limit = 0.0f; // Pitch 惯性位置环积分项限幅。
    config->output_limit = 150.0f; // Pitch 位置环目标角速度限幅，度/s。
}

// 加载惯性 Pitch 陀螺仪速度内环 PID 和转矩限幅。
void PitchInertialRateConfig_Init(volatile PitchInertialRateConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 8.0f; // Pitch 陀螺仪速度环比例增益。
    config->ki = 0.0f; // Pitch 陀螺仪速度环积分增益。
    config->kd = 0.0f; // Pitch 陀螺仪速度环微分增益。
    config->integral_limit = 0.0f; // Pitch 速度环积分项限幅。
    config->output_limit = 2047.0f; // Pitch 速度环转矩输出限幅，原始码。
}

// 组合加载惯性 Pitch 的角度外环与速度内环默认值。
void PitchInertialConfig_Init(volatile PitchInertialConfig *config)
{
    if (config == NULL) { return; }
    PitchInertialAngleConfig_Init(&config->angle);
    PitchInertialRateConfig_Init(&config->rate);
}

// 加载 Pitch 输入、机械零点、行程、滤波、重力补偿及惯性闭环参数。
void PitchConfig_Init(volatile PitchConfig *config)
{
    if (config == NULL) { return; }
    config->imu_direction = 1.0f; // IMU Pitch 与电机正方向的对应系数，符号决定方向。
    config->rate_filter_alpha = 0.45f; // Pitch 角速度低通的新样本权重，越大响应越快、滤波越弱。
    config->command_rate_deg_s = 150.0f; // Pitch 满杆位置目标变化率，度/s。
    config->home_rad = 2.59309077f; // Pitch 机械归中时的电机单圈角，rad。
    config->min_deg = (-8.5f); // Pitch 相对机械零点的下限，度。
    config->max_deg = 30.0f; // Pitch 相对机械零点的上限，度。
    config->lift_clearance_deg = 1.0f; // 升降低位时 Pitch 的抬起余量，度。
    config->target_lead_deg = 17.0f; // 目标允许领先实际位置的最大角度，度。
    config->gravity_k = 1.35f; // 归中点处约等于重力前馈，N·m。
    PitchInertialConfig_Init(&config->inertial);
}

// 加载云台任务周期、归中判据、Yaw、Pitch 和小陀螺默认值。
void CloudConfig_Init(void)
{
    cloud_config.control_period_ticks = 4U; // 云台与升降任务控制周期，tick。
    cloud_config.rc_speed_enter = 15; // 摇杆输入死区，原始通道值。
    GimbalHomeConfig_Init(&cloud_config.home);
    YawConfig_Init(&cloud_config.yaw);
    GimbalSpinConfig_Init(&cloud_config.spin);
    PitchConfig_Init(&cloud_config.pitch);
}

volatile KeyboardSensitivityConfig keyboard_sensitivity_config; // 键鼠移动和瞄准灵敏度参数。

// 加载键鼠移动、加减速和瞄准灵敏度默认值。
void KeyboardSensitivityConfig_Init(void)
{
    keyboard_sensitivity_config.move_raw = 440; // WASD 普通移动的虚拟通道幅值。
    keyboard_sensitivity_config.sprint_raw = 660; // Shift 加速移动的虚拟通道幅值。
    keyboard_sensitivity_config.slow_raw = 220; // Ctrl 慢速移动的虚拟通道幅值。
    keyboard_sensitivity_config.mouse_yaw_gain = 8; // 鼠标水平每计数对应的 Yaw 输入。
    keyboard_sensitivity_config.mouse_pitch_gain = 8; // 鼠标垂直每计数对应的 Pitch 输入。
    keyboard_sensitivity_config.aim_divisor = 2; // 右键瞄准时输入缩小的除数。
}

// 启动时统一加载上板应用默认参数，供各控制模块初始化使用。
void UpperApplicationConfig_InitAll(void)
{
    UpperCommunicationTaskConfig_Init();
    LiftConfig_Init();
    ShootConfig_Init();
    ShootHeatConfig_Init();
    CloudConfig_Init();
    KeyboardSensitivityConfig_Init();
}

volatile ShootHeatConfig shoot_heat_config;
// 加载本地计热、裁判校准、剩余热量射频档位及离线供弹策略。
void ShootHeatConfig_Init(void)
{
    shoot_heat_config.enabled = true; // 热量限制总开关。
    shoot_heat_config.allow_offline = false; // 脱离裁判调车时才允许打开。
    shoot_heat_config.heat_per_shot = 20.0f; // 每次新供弹预留的热量。
    shoot_heat_config.stop_remaining = 30.0f; // 剩余量到此阈值停止供弹。
    shoot_heat_config.low_remaining = 50.0f; // 低射频档剩余量上界。
    shoot_heat_config.high_remaining = 100.0f; // 高射频档剩余量下界。
    shoot_heat_config.low_rate_hz = 6.0f; // 低余量档射频。
    shoot_heat_config.middle_rate_hz = 10.0f; // 中余量档射频。
    shoot_heat_config.high_rate_hz = 15.0f; // 高余量档射频。
    shoot_heat_config.offline_heat_limit = 100.0f; // 离线调试尚无裁判数据时的上限。
    shoot_heat_config.offline_cooling_per_s = 10.0f; // 离线调试备用冷却速率。
    shoot_heat_config.referee_timeout_ms = 100U; // 板间热量快照有效期。
    shoot_heat_config.calibration_settle_ms = 500U; // 近期供弹热量的反馈保护时间。
}
