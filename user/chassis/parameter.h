#ifndef PARAMETER_H
#define PARAMETER_H

/* 应用层可调参数。修改数值后需重新编译并烧录。
 * *_MS 使用 HAL 毫秒计时；*_TICKS 使用 RTOS tick，二者不一定相等。
 * CAN ID、DBUS 帧格式、编码器分辨率等协议常量仍由各驱动定义。
 */

/* ==================== 遥控接收与解析 ==================== */
#define RC_TIMEOUT_MS                    100U   /* 距最后一帧有效数据超过此时间即离线。 */
#define RC_TASK_PERIOD_TICKS             15U    /* 遥控解析与在线检测任务周期。 */

/* ==================== 底盘指令与安全 ==================== */
#define CHASSIS_ENABLE_SWITCH_POSITION  2U     /* 左拨杆使能档位：上=1，中=3，下=2。 */
#define CHASSIS_MECHANICAL_SWITCH_POSITION 3U  /* 机械云台模式：底盘控制与下档一致。 */
#define CHASSIS_SPIN_SWITCH_0_POSITION   2U     /* 小陀螺组合：s[0]下档。 */
#define CHASSIS_SPIN_SWITCH_1_POSITION   1U     /* 小陀螺组合：s[1]上档。 */
#define CHASSIS_FORWARD_SCALE           5.0f   /* 遥控前进通道 ch[3] 到轮速指令的倍率。 */
#define CHASSIS_LEFT_SCALE             -5.0f   /* 遥控横移通道 ch[2] 的倍率及方向。 */
#define CHASSIS_ROTATE_SCALE           -5.0f   /* 遥控旋转通道 ch[0] 的倍率及方向。 */
#define CHASSIS_MAX_MOTOR_RPM         7000.0f   /* 麦轮解算后四轮统一缩放的最大目标转速。 */
#define CHASSIS_TASK_PERIOD_TICKS        1U     /* 底盘控制任务周期；应与 PID 时间参数一致。 */

/* ==================== 小陀螺 ==================== */
#define CHASSIS_SPIN_ROTATE_RPM        5000.0f /* 小陀螺四轮旋转速度目标。 */
#define CHASSIS_SPIN_ROTATE_SIGN          1.0f /* 修改正负可切换旋转方向。 */
#define CHASSIS_SPIN_SLEW_RPM_PER_TICK   20.0f /* 每1 ms允许增加的旋转目标。 */
#define CHASSIS_SPIN_YAW_ANGLE_SIGN       1.0f /* 云台机械Yaw角到坐标变换的方向。 */

/* ==================== 云台带动底盘跟随 ==================== */
#define CHASSIS_FOLLOW_SWITCH_POSITION  1U     /* 上档：云台带动底盘跟随。 */
#define CHASSIS_FOLLOW_ANGLE_TIMEOUT_MS 100U
#define CHASSIS_FOLLOW_DEADBAND_DEG     10.0f
#define CHASSIS_FOLLOW_KP_RPM_PER_DEG   80.0f   /* 超出死区后的角度误差 -> 轮电机转速。 */
#define CHASSIS_FOLLOW_RC_DEADBAND       15.0f   /* Yaw摇杆前馈死区，抑制中位噪声。 */
#define CHASSIS_FOLLOW_FF_RPM_PER_RC      1.5f   /* Yaw摇杆每单位对应的底盘旋转速度前馈。 */
#define CHASSIS_FOLLOW_MAX_ROTATE_RPM  5000.0f
#define CHASSIS_FOLLOW_SLEW_RPM_PER_TICK 10.0f   /* 1 ms 控制周期的旋转指令变化上限。 */
#define CHASSIS_FOLLOW_RATE_TX_PERIOD_MS 10U
#define CHASSIS_FOLLOW_ROTATE_SIGN      1.0f

/* ==================== BMI088 与姿态解算 ==================== */
#define IMU_UPDATE_PERIOD_S               0.001f
#define IMU_GYRO_CALIBRATION_SAMPLES       500U /* 上电静止约 0.5 秒。 */
#define IMU_ATTITUDE_KP                     2.0f
#define IMU_ATTITUDE_KI                    0.02f
#define IMU_YAW_RATE_FILTER_ALPHA          0.20f
/* 模板下板 IMU 安装方向：传感器坐标绕底盘 Z 轴旋转 180 度。 */
#define IMU_GYRO_X_SIGN                   (-1.0f)
#define IMU_GYRO_Y_SIGN                   (-1.0f)
#define IMU_GYRO_Z_SIGN                    1.0f
#define IMU_ACCEL_X_SIGN                  (-1.0f)
#define IMU_ACCEL_Y_SIGN                  (-1.0f)
#define IMU_ACCEL_Z_SIGN                   1.0f

/* ==================== 3508 电机保护 ==================== */
#define MOTOR3508_CURRENT_LIMIT       16384    /* C620 电流指令绝对值上限，原始报文单位。 */
#define MOTOR3508_OFFLINE_TIMEOUT_MS    100U   /* 电机反馈超过此时间未更新即视为离线。 */

/* ==================== 3508 串级位置环 PID ==================== */
/* PID_Init 参数顺序：Kp、Ki、Kd、积分限幅、输出限幅、控制周期(秒)。 */
#define MOTOR3508_POSITION_KP            2.5f
#define MOTOR3508_POSITION_KI            2.0f
#define MOTOR3508_POSITION_KD            0.0f
#define MOTOR3508_POSITION_INTEGRAL_LIMIT 500.0f
#define MOTOR3508_POSITION_OUTPUT_LIMIT 3500.0f /* 位置环输出的最大目标转速 rpm。 */

/* ==================== 3508 速度环 PID ==================== */
#define MOTOR3508_SPEED_KP               8.0f
#define MOTOR3508_SPEED_KI               2.0f
#define MOTOR3508_SPEED_KD               0.0f
#define MOTOR3508_SPEED_INTEGRAL_LIMIT 1000.0f
#define MOTOR3508_SPEED_OUTPUT_LIMIT  10000.0f /* 速度环输出限幅，电流指令原始单位。 */
#define MOTOR3508_PID_CONTROL_TIME_S     0.001f /* PID 单次调用周期；当前 1 tick = 1 ms。 */

/* ==================== 上下板通信 ==================== */
#define COMMUNICATION_TASK_PERIOD_TICKS  1U    /* 原始遥控帧经 CAN 转发的任务周期。 */

#endif /* PARAMETER_H */
