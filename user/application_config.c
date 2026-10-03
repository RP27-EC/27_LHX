#include "application_config.h"

volatile RemoteConfig remote_config; // 遥控解析与转发周期参数。

void RemoteConfig_Init(void)
{
    remote_config.task_period_ticks = 15U; // 遥控解析任务周期，tick。
    remote_config.period_ticks = 4U; // 遥控 CAN 分帧发送周期，tick。
}

volatile KeyboardSensitivityConfig keyboard_sensitivity_config; // 键鼠移动和瞄准灵敏度参数。

void KeyboardSensitivityConfig_Init(void)
{
    keyboard_sensitivity_config.move_raw = 440; // WASD 普通移动的虚拟通道幅值。
    keyboard_sensitivity_config.sprint_raw = 660; // Shift 加速移动的虚拟通道幅值。
    keyboard_sensitivity_config.slow_raw = 220; // Ctrl 慢速移动的虚拟通道幅值。
    keyboard_sensitivity_config.mouse_yaw_gain = 8; // 鼠标水平每计数对应的 Yaw 输入。
    keyboard_sensitivity_config.mouse_pitch_gain = 8; // 鼠标垂直每计数对应的 Pitch 输入。
    keyboard_sensitivity_config.aim_divisor = 2; // 右键瞄准时输入缩小的除数。
}

volatile ChassisConfig chassis_config; // 底盘解算、跟随与自旋参数。

void ChassisConfig_Init(void)
{
    chassis_config.wheel_speed_tx_period_ms = 10U; // 四轮反馈转速发送周期，ms。
    chassis_config.spin_wheel_trigger_raw = 200; // 拨轮切换小陀螺的触发阈值。
    chassis_config.spin_wheel_rearm_raw = 50; // 拨轮切换小陀螺的复位阈值。
    chassis_config.forward_scale = 5.0f; // 前进通道到目标轮速的比例，rpm/通道值。
    chassis_config.left_scale = -5.0f; // 横移通道到目标轮速的比例，rpm/通道值。
    chassis_config.rotate_scale = -5.0f; // 机械模式转向通道的轮速比例。
    chassis_config.max_motor_rpm = 7000.0f; // 底盘单轮目标速度上限，rpm。
    chassis_config.task_period_ticks = 4U; // 底盘控制任务周期，tick。
    chassis_config.spin_rotate_rpm = 4000.0f; // 小陀螺自旋目标轮速分量，rpm。
    chassis_config.spin_rotate_sign = 1.0f; // 自旋方向系数，改变符号反转方向。
    chassis_config.spin_slew_rpm_per_tick = 80.0f; // 每个底盘控制周期允许的自旋轮速变化量，rpm。
    chassis_config.spin_yaw_angle_sign = 1.0f; // 云台角度转底盘移动坐标的符号。
    chassis_config.spin_fault_rearm_ms = 500U; // 短暂许可波动立即停转，持续失效才要求重新拨档。
    chassis_config.follow_switch_position = 1U; // 底盘跟随云台的左拨杆档位值。
    chassis_config.follow_deadband_deg = 1.0f; // 底盘跟随的位置死区，区内不输出角度纠偏，度。
    chassis_config.follow_kp_rpm_per_deg = 320.0f; // 死区外角度误差到旋转轮速的比例，rpm/度。
    chassis_config.follow_rc_deadband = 15.0f; // Yaw 遥控通道前馈死区。
    chassis_config.follow_ff_rpm_per_rc = 1.5f; // 死区外遥控旋转输入到轮速前馈的比例，rpm/通道值。
    chassis_config.follow_max_rotate_rpm = 5000.0f; // 跟随旋转分量上限，rpm。
    chassis_config.follow_slew_rpm_per_tick = 400.0f; // 每个底盘控制周期允许的跟随轮速变化量，rpm。
    chassis_config.yaw_rate_tx_period_ms = 4U; // 所有模式持续上报底盘角速度，ms。
    chassis_config.follow_rotate_sign = 1.0f; // 跟随旋转方向系数。
    chassis_config.front_switch_deg = 90.0f; // 用于选择更接近云台指向的车头或车尾，度。
    chassis_config.turn_wheel_trigger_raw = 200; // 拨轮负向达到此阈值时触发调头，遥控原始值。
    chassis_config.turn_wheel_rearm_raw = 50; // 拨轮回到复位范围后允许下一次调头，遥控原始值。
    chassis_config.turn_ack_timeout_ms = 300U; // 等待上板开始调头的最长时间，超时解除预锁车，ms。
}

volatile ChassisPowerConfig chassis_power_config;
volatile ChassisPowerControlConfig chassis_power_control_config;

void ChassisPowerConfig_Init(void)
{
    chassis_power_control_config.enabled = true; // 开启超电实际功率反馈限流。
    chassis_power_control_config.offline_limit_w = 80.0f; // 未接裁判时的调试上限
    chassis_power_control_config.reserve_w = 5.0f; // 从上限中扣除的功率余量。
    chassis_power_control_config.deadband_w = 1.0f; // 目标附近停止积分的误差范围。
    chassis_power_control_config.kp = 0.4f; // 相对功率误差的比例增益。
    chassis_power_control_config.ki_per_s = 2.0f; // 电流比例积分的每秒增益。
    chassis_power_control_config.recovery_per_s = 1.0f; // 电流比例恢复的每秒上升限幅。
    chassis_power_control_config.initial_scale = 0.3f; // 上电和反馈恢复时的初始电流比例。
    chassis_power_control_config.offline_current_limit = 1000; // 功率反馈失效时的单轮原始电流限幅。
    chassis_power_config.estimate_gain = 1.0f; // 功率乘积估算的修正倍率，仅影响观察值。
    chassis_power_config.command_a_per_raw = 20.0f / 16384.0f; // C620 指令按协议换算为力矩电流。
    chassis_power_config.feedback_a_per_raw = 20.0f / 16384.0f; // 暂按指令比例解释反馈，保留独立校准入口。
}

void LowerApplicationConfig_InitAll(void)
{
    RemoteConfig_Init();
    KeyboardSensitivityConfig_Init();
    ChassisConfig_Init();
    ChassisPowerConfig_Init();
}
