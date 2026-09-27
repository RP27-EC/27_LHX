#include "application_config.h"

volatile UpperCommunicationTaskConfig upper_communication_task_config;

void UpperCommunicationTaskConfig_Init(void)
{
    upper_communication_task_config.period_ticks = 4U; // 板间遥控接收解析周期，tick。
}

volatile LiftConfig lift_config;

void LiftConfig_Init(void)
{
    lift_config.down_direction = -1.0f; // 编码器计数减小表示下降。
    lift_config.down_speed_rad_s = 1000.0f; // 正常下降的转子速度上限，rad/s。
    lift_config.up_speed_rad_s = 1000.0f; // 正常上升的转子速度上限，rad/s。
    lift_config.max_rotor_turns = 320.0f; // 上电后正、反向累计转数限制，圈。
    lift_config.limit_stop_margin_counts = 40; // 行程边界前的停机余量，编码器计数。
    lift_config.yaw_deadzone_deg = 2.0f; // 允许升降的 Yaw 归中角度误差，度。
    lift_config.yaw_stable_ms = 100U; // Yaw 连续处于死区内的时间，ms。
    lift_config.stop_retry_ms = 50U; // 普通停机命令重发间隔，ms。
    lift_config.fault_stop_retry_ms = 5U; // 故障停机命令重发间隔，ms。
    lift_config.stall_progress_counts = 40; // 堵转时间窗内要求的最小位移，计数。
    lift_config.down_stall_current_raw = 75; // 下降堵转电流阈值，反馈原始码。
    lift_config.down_stall_speed_rpm = 1; // 下降堵转低速阈值，rpm。
    lift_config.down_stall_time_ms = 200U; // 下降堵转确认时间，ms。
    lift_config.up_stall_current_raw = 520; // 上升堵转电流阈值，反馈原始码。
    lift_config.up_stall_speed_rpm = 1; // 上升堵转低速阈值，rpm。
    lift_config.up_stall_time_ms = 500U; // 上升堵转确认时间，ms。
    lift_config.calibrate_up_speed_rad_s = 300.0f; // 自动找顶部的转子速度，rad/s。
    lift_config.calibrate_timeout_ms = 90000U; // 向上找顶部的最长时间，ms。
    lift_config.top_clearance_turns = 5.0f; // 高位目标距碰顶点向下 5 圈。
    lift_config.travel_turns = 316.0f; // 低位目标距碰顶点向下 316 圈。
    lift_config.position_kp_rad_s_per_turn = 5.5f; // 每圈位置误差对应的目标速度，rad/s。
    lift_config.position_min_speed_rad_s = 10.0f; // 未到位时的最小目标速度，rad/s。
    lift_config.hold_speed_rad_s = 25.0f; // 外力偏离目标后回位的速度上限，rad/s。
    lift_config.position_tolerance_counts = 200; // 位控允许的到位误差，编码器计数。
    lift_config.lock_tx_period_ms = 10U; // 向下板发送锁车请求的间隔，ms。
    lift_config.chassis_lock_settle_ms = 20U; // 发出锁车请求后的等待时间，ms。
    lift_config.chassis_release_rpm = 10; // 低于该转速时可释放底盘锁定，rpm。
    lift_config.offline_release_ms = 200U; // 电机离线后继续锁车的时间，ms。
}

volatile ShootConfig shoot_config;

void ShootConfig_Init(void)
{
    shoot_config.control_period_ticks = 4U; // 发射任务控制周期，tick。
    shoot_config.fric_target_speed_rpm = 1500; // 摩擦轮目标转速幅值，rpm。
    shoot_config.dial_feed_direction = 1LL; // 拨盘上弹方向；负值反转。
    shoot_config.dial_continuous_rounds_per_s = 15.0f; // 连发拨盘速度，圈/s。
    shoot_config.dial_arrived_error_counts = 500LL; // 单发位控到位误差，计数。
    shoot_config.dial_single_move_timeout_ms = 500U; // 单发动作超时，ms。
    shoot_config.dial_block_current_threshold = 600; // 拨盘堵转电流阈值，原始码。
    shoot_config.dial_block_speed_threshold_dps = 10; // 拨盘堵转低速阈值，度/s。
    shoot_config.dial_block_confirm_ticks = 50U; // 4 ms×50 次，堵转确认约 200 ms。
    shoot_config.dial_stuck_reverse_timeout_ms = 200U; // 堵转反向退让超时，ms。
    shoot_config.dial_stuck_reload_timeout_ms = 200U; // 退让后重新上弹超时，ms。
    shoot_config.dial_safe_stop_retry_ms = 20U; // 安全态零电流帧重发间隔，ms。
    shoot_config.mouse_continuous_threshold_ms = 200U; // 鼠标长按转连发的时间，ms。
}

volatile CloudConfig cloud_config;

void CloudConfig_Init(void)
{
    cloud_config.control_period_ticks = 4U; // 云台与升降任务控制周期，tick。
    cloud_config.rc_speed_enter = 15; // 摇杆进入速控的原始值阈值。
    cloud_config.rc_speed_exit = 8; // 摇杆退出速控的原始值阈值。
    cloud_config.pitch_max_speed_raw = 150; // Pitch 满杆速度目标，电机原始码。
    cloud_config.yaw_command_rate_deg_s = 200.0f; // Yaw 满杆目标角变化率，度/s。
    cloud_config.yaw_rc_direction = 1.0f; // Yaw 摇杆输入方向系数。
    cloud_config.yaw_angle_kp = 10.0f; // Yaw 角度外环比例增益。
    cloud_config.yaw_angle_ki = 0.0f; // Yaw 角度外环积分增益。
    cloud_config.yaw_angle_kd = 0.0f; // Yaw 角度外环微分增益。
    cloud_config.yaw_angle_integral_limit = 200.0f; // 角度外环积分项限幅。
    cloud_config.yaw_rate_target_limit_deg_s = 600.0f; // 外环目标角速度上限，度/s。
    cloud_config.yaw_rate_kp = 10.0f; // Yaw 角速度内环比例增益。
    cloud_config.yaw_rate_ki = 0.0f; // Yaw 角速度内环积分增益。
    cloud_config.yaw_rate_kd = 0.0f; // Yaw 角速度内环微分增益。
    cloud_config.yaw_rate_integral_limit = 0.0f; // 角速度内环积分项限幅。
    cloud_config.yaw_torque_limit_raw = 2047.0f; // Yaw 输出转矩码上限。
    cloud_config.mechanical_yaw_near_deg = 3.0f; // 机械模式距当前前/后目标小于 3° 时切换近点 PID。
    cloud_config.mechanical_yaw_deadzone_deg = 1.0f; // 机械模式 Yaw 目标角的 ±1° 死区。
    cloud_config.yaw_home_rad = (-0.387884378f); // Yaw 指向车头时的电机单圈角，rad。
    cloud_config.pitch_home_rad = 2.59309077f; // Pitch 机械归中时的电机单圈角，rad。
    cloud_config.home_tolerance_deg = 1.5f; // 云台归中的位置误差，度。
    cloud_config.home_speed_raw_max = 20; // 归中到位时的速度原始码上限。
    cloud_config.home_stable_cycles = 5U; // 4 ms×5 次，归中连续到位约 20 ms。
    cloud_config.pitch_min_deg = (-7.0f); // Pitch 相对机械零点的下限，度。
    cloud_config.lift_pitch_clearance_deg = 1.0f; // 升降低位时 Pitch 的抬起余量，度。
    cloud_config.pitch_max_deg = 30.0f; // Pitch 相对机械零点的上限，度。
    cloud_config.pitch_limit_slow_deg = 5.0f; // Pitch 靠近机械限位的减速区，度。
    cloud_config.turn_wheel_trigger_raw = 200; // 拨轮调头触发阈值，原始码。
    cloud_config.turn_wheel_rearm_raw = 50; // 拨轮调头重新布防阈值，原始码。
    cloud_config.spin_wheel_trigger_raw = 200; // 拨轮切换小陀螺的触发阈值。
    cloud_config.spin_wheel_rearm_raw = 50; // 拨轮切换小陀螺的复位阈值。
    cloud_config.front_switch_deg = 90.0f; // 选择正反车头的角度分界，度。
    cloud_config.turn_tolerance_deg = 3.0f; // 调头目标角的到位误差，度。
    cloud_config.turn_speed_raw_max = 20; // 调头到位时的速度原始码上限。
    cloud_config.turn_stable_cycles = 5U; // 4 ms×5 次，调头连续到位约 20 ms。
    cloud_config.pitch_gravity_center_rad = 2.678706762f; // Pitch 重力补偿余弦中心角，rad。
    cloud_config.pitch_gravity_k = 4.2072f; // Pitch 重力补偿余弦幅值，N·m。
    cloud_config.pitch_gravity_b = (-2.496f); // Pitch 重力补偿恒定偏置，N·m。
    cloud_config.pitch_gravity_scale = 0.65f; // Pitch 重力补偿整体比例。
}

volatile KeyboardSensitivityConfig keyboard_sensitivity_config;

void KeyboardSensitivityConfig_Init(void)
{
    keyboard_sensitivity_config.move_raw = 440; // WASD 普通移动的虚拟通道幅值。
    keyboard_sensitivity_config.sprint_raw = 660; // Shift 加速移动的虚拟通道幅值。
    keyboard_sensitivity_config.slow_raw = 220; // Ctrl 慢速移动的虚拟通道幅值。
    keyboard_sensitivity_config.mouse_yaw_gain = 8; // 鼠标水平每计数对应的 Yaw 输入。
    keyboard_sensitivity_config.mouse_pitch_gain = 8; // 鼠标垂直每计数对应的 Pitch 输入。
    keyboard_sensitivity_config.aim_divisor = 2; // 右键瞄准时输入缩小的除数。
}

void UpperApplicationConfig_InitAll(void)
{
    UpperCommunicationTaskConfig_Init();
    LiftConfig_Init();
    ShootConfig_Init();
    CloudConfig_Init();
    KeyboardSensitivityConfig_Init();
}
