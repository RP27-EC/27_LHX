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
#define CHASSIS_FORWARD_SCALE           5.0f   /* 遥控前进通道 ch[3] 到轮速指令的倍率。 */
#define CHASSIS_LEFT_SCALE             -5.0f   /* 遥控横移通道 ch[2] 的倍率及方向。 */
#define CHASSIS_ROTATE_SCALE           -5.0f   /* 遥控旋转通道 ch[0] 的倍率及方向。 */
#define CHASSIS_MAX_MOTOR_RPM         5000.0f   /* 麦轮解算后四轮统一缩放的最大目标转速。 */
#define CHASSIS_TASK_PERIOD_TICKS        1U     /* 底盘控制任务周期；应与 PID 时间参数一致。 */

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
