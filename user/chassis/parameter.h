#ifndef PARAMETER_H
#define PARAMETER_H

/* 下板集中参数，修改后需重新编译烧录。
 * *_MS 使用 HAL 毫秒，*_TICKS 使用 RTOS 节拍；当前控制任务 1 tick = 1 ms。
 * PID 积分限幅限制积分项，输出限幅限制该环输出；协议常量仍由驱动定义。
 */

/* ==================== 遥控接收与解析 ==================== */
#define RC_TIMEOUT_MS                    100U   /* 最后有效 DBUS 帧距今≥100 ms 即判遥控离线。 */
#define RC_TASK_PERIOD_TICKS             15U    /* 遥控解析任务每 15 tick 运行一次。 */

/* 键鼠模式由 V 按键切换；转换后的虚拟通道仍使用 DBUS 的 ±660 量程。 */
#define RC_KEYBOARD_MOVE_RAW             440    /* WASD 普通移动对应的虚拟摇杆幅值。 */
#define RC_KEYBOARD_SPRINT_RAW           660    /* 按住 Shift 时的移动幅值。 */
#define RC_KEYBOARD_SLOW_RAW             220    /* 按住 Ctrl 时的移动幅值。 */
#define RC_KEYBOARD_MOUSE_YAW_GAIN       8      /* 每个鼠标 X 计数换算的 Yaw 虚拟通道值。 */
#define RC_KEYBOARD_MOUSE_PITCH_GAIN     8      /* 每个鼠标 Y 计数换算的 Pitch 虚拟通道值。 */
#define RC_KEYBOARD_AIM_DIVISOR          2      /* 按住鼠标右键时视角输入减半。 */

/* ==================== 底盘指令与安全 ==================== */
#define CHASSIS_MECHANICAL_SWITCH_POSITION 3U  /* DBUS 左拨杆中档值，机械模式由 ch[0] 直接转底盘。 */
#define CHASSIS_SPIN_SWITCH_0_POSITION   2U     /* DBUS 左拨杆下档值，选择小陀螺模式。 */
#define CHASSIS_SPIN_SWITCH_1_POSITION   1U     /* DBUS 右拨杆上档值，小陀螺中才实际自旋。 */
/* 遥控通道约 -660~660；倍率把通道值直接转换为四轮解算输入 rpm。 */
#define CHASSIS_FORWARD_SCALE           5.0f   /* 前进输入=ch[3]×5，满杆约 3300 rpm。 */
#define CHASSIS_LEFT_SCALE             -5.0f   /* 横移输入=ch[2]×(-5)，负号确定左右方向。 */
#define CHASSIS_ROTATE_SCALE           -5.0f   /* 机械模式旋转输入=ch[0]×(-5)，rpm。 */
#define CHASSIS_MAX_MOTOR_RPM         7000.0f   /* 四轮解算后按最大绝对值等比例缩放到此限幅。 */
#define CHASSIS_TASK_PERIOD_TICKS        1U     /* 底盘控制任务周期，当前 1 ms。 */

/* ==================== 小陀螺 ==================== */
#define CHASSIS_SPIN_ROTATE_RPM        5000.0f /* 小陀螺底盘旋转分量目标幅值，轮速 rpm。 */
#define CHASSIS_SPIN_ROTATE_SIGN          1.0f /* 旋转分量的方向系数；改为 -1 可反转。 */
#define CHASSIS_SPIN_SLEW_RPM_PER_TICK   20.0f /* 自旋分量每 1 tick 最多变 20 rpm；1 ms 时约 20 krpm/s。 */
#define CHASSIS_SPIN_YAW_ANGLE_SIGN       1.0f /* 云台相对车头角进坐标旋转前的符号系数。 */

/* ==================== 云台带动底盘跟随 ==================== */
#define CHASSIS_FOLLOW_SWITCH_POSITION  1U     /* DBUS 左拨杆上档值，选择底盘跟随云台。 */
#define CHASSIS_FOLLOW_ANGLE_TIMEOUT_MS 100U   /* 上板 C1 机械 Yaw 角超过此时间未更新则停止跟随。 */
#define CHASSIS_FOLLOW_DEADBAND_DEG     2.0f   /* 相对所选正方向误差≤5° 时不输出角度跟随分量。 */
/* 死区外角误差：正角减 10°，负角加 10°；旋转分量=角误差×KP+摇杆前馈。 */
#define CHASSIS_FOLLOW_KP_RPM_PER_DEG   320.0f   /* 每超出死区 1°，增加 80 rpm 底盘旋转分量。 */
#define CHASSIS_FOLLOW_RC_DEADBAND       15.0f  /* Yaw 遥控通道原始值的前馈死区。 */
#define CHASSIS_FOLLOW_FF_RPM_PER_RC      1.8f   /* 死区外每 1 通道值增加 1.5 rpm 前馈。 */
#define CHASSIS_FOLLOW_MAX_ROTATE_RPM  5000.0f  /* 跟随旋转分量绝对值上限，轮速 rpm。 */
#define CHASSIS_FOLLOW_SLEW_RPM_PER_TICK 200.0f  /* 跟随旋转分量每 tick 最多变 10 rpm。 */
#define CHASSIS_FOLLOW_RATE_TX_PERIOD_MS 10U    /* 向上板发送 D4 实测底盘角速度的最短间隔。 */
#define CHASSIS_FOLLOW_ROTATE_SIGN      1.0f    /* 跟随旋转最终方向系数，改符号可反转。 */
/* 两个相反的车头方向分别对应云台机械 Yaw 相对底盘 0° 和 180°。
 * |角度|<90° 选物理车头，≥90° 选车尾。
 * 拨轮从中位向上跨过负阈值触发调头，回中位附近后才可再次触发。
 */
#define CHASSIS_FRONT_SWITCH_DEG          90.0f /* 正反车头的就近选择分界角。 */
#define CHASSIS_TURN_WHEEL_TRIGGER_RAW     200  /* ch[4]≤-200 视为向上拨到触发位。 */
#define CHASSIS_TURN_WHEEL_REARM_RAW        50  /* ch[4]>-50 时重新布防。 */
#define CHASSIS_TURN_DONE_TOLERANCE_DEG   2.0f /* 收到转完状态后，离目标≤5° 才恢复底盘。 */

/* ==================== BMI088 与姿态解算 ==================== */
#define IMU_UPDATE_PERIOD_S               0.001f /* 姿态积分使用的周期，1 ms。 */
#define IMU_GYRO_CALIBRATION_SAMPLES       500U   /* 静止零偏采样 500 次，每次隔 1 ms，约 0.5 s。 */
#define IMU_ATTITUDE_KP                     2.0f /* 加速度重力方向对姿态的比例校正增益。 */
#define IMU_ATTITUDE_KI                    0.02f /* 姿态误差积分校正增益。 */
#define IMU_YAW_RATE_FILTER_ALPHA          0.20f /* 新角速度样本权重；滤波值+=0.2×(新值-旧值)。 */
/* BMI088 传感器坐标绕底盘 Z 轴旋转 180°：X/Y 取反，Z 不变。 */
#define IMU_GYRO_X_SIGN                   (-1.0f) /* 陀螺 X 轴方向系数。 */
#define IMU_GYRO_Y_SIGN                   (-1.0f) /* 陀螺 Y 轴方向系数。 */
#define IMU_GYRO_Z_SIGN                    1.0f   /* 陀螺 Z 轴方向系数。 */
#define IMU_ACCEL_X_SIGN                  (-1.0f) /* 加速度 X 轴方向系数。 */
#define IMU_ACCEL_Y_SIGN                  (-1.0f) /* 加速度 Y 轴方向系数。 */
#define IMU_ACCEL_Z_SIGN                   1.0f   /* 加速度 Z 轴方向系数。 */

/* ==================== 3508 电机保护 ==================== */
#define MOTOR3508_CURRENT_LIMIT       16384    /* C620 四电机电流命令原始码的绝对值上限。 */
#define MOTOR3508_OFFLINE_TIMEOUT_MS    100U   /* 任一底盘电机反馈≥100 ms 未更新则停车。 */

/* ==================== 3508 串级位置环 PID ==================== */
/* 位置环输入累计编码器计数，输出目标电机转速 rpm；当前底盘主任务走速度模式。 */
#define MOTOR3508_POSITION_KP            2.5f   /* 计数误差到目标 rpm 的比例增益。 */
#define MOTOR3508_POSITION_KI            2.0f   /* 位置误差积分增益。 */
#define MOTOR3508_POSITION_KD            0.0f   /* 位置误差微分增益；0 为关闭。 */
#define MOTOR3508_POSITION_INTEGRAL_LIMIT 500.0f /* 位置环积分项绝对值上限。 */
#define MOTOR3508_POSITION_OUTPUT_LIMIT 3500.0f /* 位置环输出目标速度绝对值上限，rpm。 */

/* ==================== 3508 速度环 PID ==================== */
/* 速度环输入电机 rpm，输出 C620 电流命令原始码。 */
#define MOTOR3508_SPEED_KP               8.0f  /* rpm 误差到电流码的比例增益。 */
#define MOTOR3508_SPEED_KI               2.0f  /* rpm 误差积分增益。 */
#define MOTOR3508_SPEED_KD               0.0f  /* rpm 误差微分增益；0 为关闭。 */
#define MOTOR3508_SPEED_INTEGRAL_LIMIT 1000.0f /* 速度环积分项绝对值上限。 */
#define MOTOR3508_SPEED_OUTPUT_LIMIT  10000.0f /* PID 电流码输出上限，另受 CURRENT_LIMIT 约束。 */
#define MOTOR3508_PID_CONTROL_TIME_S     0.001f /* PID 每次调用间隔，1 ms。 */

/* ==================== 上下板通信 ==================== */
#define COMMUNICATION_TASK_PERIOD_TICKS  1U    /* D1~D3 遥控分帧转发任务周期，当前 1 ms。 */
#define COMMUNICATION_BUS_OFF_RETRY_MS    100U  /* CAN2 Bus-Off 后两次 Stop/Start 尝试至少间隔 100 ms。 */

#endif /* PARAMETER_H */
