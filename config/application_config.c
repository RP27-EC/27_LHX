#include "application_config.h"
#include <stddef.h>

volatile RemoteConfig remote_config; // 遥控解析与转发周期参数。

// 加载遥控解析、遥控转发及裁判热量转发周期。
void RemoteConfig_Init(void)
{
    remote_config.heat_tx_period_ms = 20U; // 热量和有效标志转发间隔。
    remote_config.task_period_ticks = 15U; // 遥控解析任务周期，tick。
    remote_config.period_ticks = 4U; // 遥控 CAN 分帧发送周期，tick。
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

volatile ChassisConfig chassis_config; // 底盘解算、跟随与自旋参数。

// 加载底盘小陀螺触发、转速、方向、斜坡及重新布防参数。
void ChassisSpinConfig_Init(volatile ChassisSpinConfig *config)
{
    if (config == NULL) { return; }
    config->wheel_trigger_raw = 200; // 拨轮切换小陀螺的触发阈值。
    config->wheel_rearm_raw = 50; // 拨轮切换小陀螺的复位阈值。
    config->rotate_rpm = 4000.0f; // 小陀螺自旋目标轮速分量，rpm。
    config->rotate_sign = 1.0f; // 自旋方向系数，改变符号反转方向。
    config->slew_rpm_per_tick = 80.0f; // 每个底盘控制周期允许的自旋轮速变化量，rpm。
    config->yaw_angle_sign = 1.0f; // 云台角度转底盘移动坐标的符号。
    config->fault_rearm_ms = 500U; // 短暂许可波动立即停转，持续失效才要求重新拨档。
}

// 加载底盘平移、机械转向的输入比例和轮速限幅。
void ChassisMotionConfig_Init(volatile ChassisMotionConfig *config)
{
    if (config == NULL) { return; }
    config->forward_scale = 5.0f; // 前进通道到目标轮速的比例，rpm/通道值。
    config->left_scale = -5.0f; // 横移通道到目标轮速的比例，rpm/通道值。
    config->rotate_scale = -5.0f; // 机械模式转向通道的轮速比例。
    config->max_motor_rpm = 7000.0f; // 底盘单轮目标速度上限，rpm。
}

// 加载底盘跟随的死区、纠偏、输入前馈、限速和斜坡参数。
void ChassisFollowConfig_Init(volatile ChassisFollowConfig *config)
{
    if (config == NULL) { return; }
    config->deadband_deg = 1.0f; // 底盘跟随的位置死区，区内不输出角度纠偏，度。
    config->kp_rpm_per_deg = 320.0f; // 死区外角度误差到旋转轮速的比例，rpm/度。
    config->rc_deadband = 15.0f; // Yaw 遥控通道前馈死区。
    config->ff_rpm_per_rc = 1.5f; // 死区外遥控旋转输入到轮速前馈的比例，rpm/通道值。
    config->max_rotate_rpm = 5000.0f; // 跟随旋转分量上限，rpm。
    config->slew_rpm_per_tick = 400.0f; // 每个底盘控制周期允许的跟随轮速变化量，rpm。
    config->rotate_sign = 1.0f; // 跟随旋转方向系数。
}

// 加载调头拨轮门槛和等待上板确认的超时参数。
void ChassisTurnConfig_Init(volatile ChassisTurnConfig *config)
{
    if (config == NULL) { return; }
    config->wheel_trigger_raw = 200; // 拨轮负向达到此阈值时触发调头，遥控原始值。
    config->wheel_rearm_raw = 50; // 拨轮回到复位范围后允许下一次调头，遥控原始值。
    config->ack_timeout_ms = 300U; // 等待上板开始调头的最长时间，超时解除预锁车，ms。
}

// 加载底盘任务及反馈发送周期，并组合加载各运动模式默认值。
void ChassisConfig_Init(void)
{
    chassis_config.wheel_speed_tx_period_ms = 10U; // 四轮反馈转速发送周期，ms。
    chassis_config.task_period_ticks = 4U; // 底盘控制任务周期，tick。
    chassis_config.yaw_rate_tx_period_ms = 4U; // 所有模式持续上报底盘角速度，ms。
    chassis_config.front_switch_deg = 90.0f; // 用于选择更接近云台指向的车头或车尾，度。
    ChassisSpinConfig_Init(&chassis_config.spin);
    ChassisMotionConfig_Init(&chassis_config.motion);
    ChassisFollowConfig_Init(&chassis_config.follow);
    ChassisTurnConfig_Init(&chassis_config.turn);
}

volatile ChassisPowerModelConfig chassis_power_model_config;
volatile ChassisPowerControlConfig chassis_power_control_config;

// 加载功率余量、实测反馈 PI、恢复速度及失联降级参数。
void ChassisPowerControlConfig_Init(void)
{
    chassis_power_control_config.enabled = true; // 开启模型预测与实测反馈限流。
    chassis_power_control_config.offline_limit_w = 80.0f; // 未接裁判时的调试上限
    chassis_power_control_config.reserve_w = 5.0f; // 从上限中扣除的功率余量。
    chassis_power_control_config.deadband_w = 1.0f; // 目标附近停止积分的误差范围。
    chassis_power_control_config.kp = 0.4f; // 相对功率误差的比例增益。
    chassis_power_control_config.ki_per_s = 2.0f; // 电流比例积分的每秒增益。
    chassis_power_control_config.recovery_per_s = 1.0f; // 电流比例恢复的每秒上升限幅。
    chassis_power_control_config.initial_scale = 0.3f; // 上电和反馈恢复时的初始电流比例。
    chassis_power_control_config.offline_current_limit = 1000; // 功率反馈失效时的单轮原始电流限幅。
}

// 加载各轮电流与转速的功率模型系数，供发送电流前预测功率。
void ChassisPowerModelConfig_Init(void)
{
    // 各轮依次对应电机 ID，项顺序为常数、电流、转速、交叉、电流平方、转速平方。
    const ChassisPowerModelConfig model = {{
        {1.426816374f, 0.00044888211f, 8.49260410e-05f, 1.78188222e-06f, 1.37697922e-07f, 3.54823528e-07f},
        {1.316759451f, -0.00038599261f, -0.00015057505f, 1.56183427e-06f, 1.63447186e-07f, 4.45074719e-07f},
        {1.373260522f, -0.00055127187f, 0.00016888118f, 1.62672879e-06f, 1.53561305e-07f, 3.99971127e-07f},
        {1.373260522f, -0.00055127187f, 0.00016888118f, 1.62672879e-06f, 1.53561305e-07f, 3.99971127e-07f},
    }};
    chassis_power_model_config = model;
}

// 统一加载底盘功率模型和实测功率反馈控制默认值。
void ChassisPowerConfig_Init(void)
{
    ChassisPowerControlConfig_Init();
    ChassisPowerModelConfig_Init();
}

// 启动时统一加载下板应用默认参数，供底盘与遥控任务使用。
void LowerApplicationConfig_InitAll(void)
{
    RemoteConfig_Init();
    KeyboardSensitivityConfig_Init();
    ChassisConfig_Init();
    ChassisPowerConfig_Init();
}
