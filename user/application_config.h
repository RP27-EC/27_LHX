#ifndef DOWN_APPLICATION_CONFIG_H
#define DOWN_APPLICATION_CONFIG_H

#include <stdint.h>

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t task_period_ticks; // 对应任务的执行周期，tick。
    uint32_t period_ticks; // D1~D3 遥控分帧转发任务周期，tick。
} RemoteConfig;
extern volatile RemoteConfig remote_config; // 遥控解析与转发周期参数。
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
extern volatile KeyboardSensitivityConfig keyboard_sensitivity_config; // 键鼠移动和瞄准灵敏度参数。
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
    uint32_t task_period_ticks; // 对应任务的执行周期，tick。
    float spin_rotate_rpm; // 小陀螺底盘旋转分量目标幅值，轮速 rpm。
    float spin_rotate_sign; // 自旋方向系数，改变符号反转方向。
    float spin_slew_rpm_per_tick; // 每个底盘控制周期允许的自旋轮速变化量，rpm。
    float spin_yaw_angle_sign; // 云台相对车头角进坐标旋转前的符号系数。
    uint32_t spin_fault_rearm_ms; // 自旋许可连续丢失超过此时间才锁存重新拨档。
    uint32_t follow_switch_position; // DBUS 左拨杆上档值，选择底盘跟随云台。
    float follow_deadband_deg; // 底盘跟随的位置死区，区内不输出角度纠偏，度。
    float follow_kp_rpm_per_deg; // 死区外角度误差到旋转轮速的比例，rpm/度。
    float follow_rc_deadband; // Yaw 遥控通道原始值的前馈死区。
    float follow_ff_rpm_per_rc; // 死区外遥控旋转输入到轮速前馈的比例，rpm/通道值。
    float follow_max_rotate_rpm; // 跟随旋转分量绝对值上限，轮速 rpm。
    float follow_slew_rpm_per_tick; // 每个底盘控制周期允许的跟随轮速变化量，rpm。
    uint32_t yaw_rate_tx_period_ms; // 所有模式向上板发送 D4 实测底盘角速度的最短间隔。
    float follow_rotate_sign; // 跟随旋转最终方向系数，改符号可反转。
    float front_switch_deg; // 用于选择更接近云台指向的车头或车尾，度。
    int32_t turn_wheel_trigger_raw; // 拨轮负向达到此阈值时触发调头，遥控原始值。
    int32_t turn_wheel_rearm_raw; // 拨轮回到复位范围后允许下一次调头，遥控原始值。
    uint32_t turn_ack_timeout_ms; // 等待上板开始调头的最长时间，超时解除预锁车，ms。
} ChassisConfig;
extern volatile ChassisConfig chassis_config; // 底盘解算、跟随与自旋参数。
void ChassisConfig_Init(void);

void LowerApplicationConfig_InitAll(void);

#endif // DOWN_APPLICATION_CONFIG_H
