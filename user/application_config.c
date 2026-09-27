#include "application_config.h"

volatile RemoteConfig remote_config;

void RemoteConfig_Init(void)
{
    remote_config.task_period_ticks = 15U; // 遥控解析任务周期，tick。
    remote_config.period_ticks = 4U; // 遥控 CAN 分帧发送周期，tick。
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

volatile ChassisConfig chassis_config;

void ChassisConfig_Init(void)
{
    chassis_config.wheel_speed_tx_period_ms = 10U; // 四轮反馈转速发送周期，ms。
    chassis_config.spin_wheel_trigger_raw = 200; // 拨轮切换小陀螺的触发阈值。
    chassis_config.spin_wheel_rearm_raw = 50; // 拨轮切换小陀螺的复位阈值。
    chassis_config.forward_scale = 5.0f; // 前进通道到目标轮速的比例，rpm/通道值。
    chassis_config.left_scale = -5.0f; // 横移通道到目标轮速的比例，rpm/通道值。
    chassis_config.rotate_scale = -5.0f; // 机械模式转向通道的轮速比例。
    chassis_config.max_motor_rpm = 7000.0f; // 底盘单轮目标速度上限，rpm。
    chassis_config.task_period_ticks = 4U; // 底盘控制任务周期，4 tick（4 ms）。
    chassis_config.spin_rotate_rpm = 5000.0f; // 小陀螺自旋目标轮速分量，rpm。
    chassis_config.spin_rotate_sign = 1.0f; // 小陀螺自旋方向系数。
    chassis_config.spin_slew_rpm_per_tick = 80.0f; // 自旋轮速每 4 ms 最大变化 80 rpm，保持原加速率。
    chassis_config.spin_yaw_angle_sign = 1.0f; // 云台角度转底盘移动坐标的符号。
    chassis_config.follow_switch_position = 1U; // 底盘跟随云台的左拨杆档位值。
    chassis_config.follow_deadband_deg = 1.0f; // 底盘跟随角度死区，度。
    chassis_config.follow_kp_rpm_per_deg = 320.0f; // 每度角差对应的跟随轮速，rpm/度。
    chassis_config.follow_rc_deadband = 15.0f; // Yaw 遥控通道前馈死区。
    chassis_config.follow_ff_rpm_per_rc = 1.8f; // 前馈轮速与 Yaw 通道值的比例。
    chassis_config.follow_max_rotate_rpm = 5000.0f; // 跟随旋转分量上限，rpm。
    chassis_config.follow_slew_rpm_per_tick = 800.0f; // 跟随轮速每 4 ms 最大变化 800 rpm，保持原加速率。
    chassis_config.follow_rate_tx_period_ms = 10U; // 底盘角速度发送间隔，ms。
    chassis_config.follow_rotate_sign = 1.0f; // 跟随旋转方向系数。
    chassis_config.front_switch_deg = 90.0f; // 正反车头选择的角度分界，度。
    chassis_config.turn_wheel_trigger_raw = 200; // 拨轮调头触发阈值，原始码。
    chassis_config.turn_wheel_rearm_raw = 50; // 拨轮调头重新布防阈值，原始码。
    chassis_config.turn_ack_timeout_ms = 300U; // 上板未开始调头时，最多预锁车 300 ms。
}

void LowerApplicationConfig_InitAll(void)
{
    RemoteConfig_Init();
    KeyboardSensitivityConfig_Init();
    ChassisConfig_Init();
}
