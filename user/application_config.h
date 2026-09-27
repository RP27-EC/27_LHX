#ifndef DOWN_APPLICATION_CONFIG_H
#define DOWN_APPLICATION_CONFIG_H

#include <stdint.h>

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t task_period_ticks; // 遥控解析任务的执行周期，tick。
    uint32_t period_ticks; // D1~D3 遥控分帧转发任务周期，tick。
} RemoteConfig;
extern volatile RemoteConfig remote_config;
void RemoteConfig_Init(void);

typedef struct
{
    int32_t move_raw; // WASD 普通移动幅值。
    int32_t sprint_raw; // Shift 加速幅值。
    int32_t slow_raw; // Ctrl 慢速幅值。
    int32_t mouse_yaw_gain; // 鼠标 X 灵敏度。
    int32_t mouse_pitch_gain; // 鼠标 Y 灵敏度。
    int32_t aim_divisor; // 右键精细瞄准倍率的除数。
} KeyboardSensitivityConfig;
extern volatile KeyboardSensitivityConfig keyboard_sensitivity_config;
void KeyboardSensitivityConfig_Init(void);

typedef struct
{
    uint32_t wheel_speed_tx_period_ms; // 向上板发送四轮实测转速的周期。
    int32_t spin_wheel_trigger_raw; // 拨轮正向越过此值切换小陀螺。
    int32_t spin_wheel_rearm_raw; // 拨轮回中位后才能再次切换。
    float forward_scale; // 前进输入=ch[3]×此系数，目标轮速 rpm。
    float left_scale; // 横移输入=ch[2]×此系数，符号确定左右方向。
    float rotate_scale; // 机械模式旋转输入=ch[0]×此系数，rpm。
    float max_motor_rpm; // 四轮解算后按最大绝对值等比例缩放到此限幅。
    uint32_t task_period_ticks; // 底盘控制任务周期，当前 4 ms。
    float spin_rotate_rpm; // 小陀螺底盘旋转分量目标幅值，轮速 rpm。
    float spin_rotate_sign; // 旋转分量的方向系数；改为 -1 可反转。
    float spin_slew_rpm_per_tick; // 自旋分量每控制周期的最大变化量，rpm。
    float spin_yaw_angle_sign; // 云台相对车头角进坐标旋转前的符号系数。
    uint32_t follow_switch_position; // DBUS 左拨杆上档值，选择底盘跟随云台。
    float follow_deadband_deg; // 相对所选正方向误差≤5° 时不输出角度跟随分量。
    float follow_kp_rpm_per_deg; // 每超出死区 1°，增加 80 rpm 底盘旋转分量。
    float follow_rc_deadband; // Yaw 遥控通道原始值的前馈死区。
    float follow_ff_rpm_per_rc; // 死区外每 1 通道值增加 1.5 rpm 前馈。
    float follow_max_rotate_rpm; // 跟随旋转分量绝对值上限，轮速 rpm。
    float follow_slew_rpm_per_tick; // 跟随旋转分量每控制周期的最大变化量，rpm。
    uint32_t follow_rate_tx_period_ms; // 向上板发送 D4 实测底盘角速度的最短间隔。
    float follow_rotate_sign; // 跟随旋转最终方向系数，改符号可反转。
    float front_switch_deg; // 正反车头的就近选择分界角。
    int32_t turn_wheel_trigger_raw; // ch[4]≤-200 视为向上拨到触发位。
    int32_t turn_wheel_rearm_raw; // ch[4]>-50 时重新布防。
    uint32_t turn_ack_timeout_ms; // 调头请求发出后等待上板开始的最长时间，ms。
} ChassisConfig;
extern volatile ChassisConfig chassis_config;
void ChassisConfig_Init(void);

void LowerApplicationConfig_InitAll(void);

#endif // DOWN_APPLICATION_CONFIG_H
