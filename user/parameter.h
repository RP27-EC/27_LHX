#ifndef PARAMETER_H
#define PARAMETER_H

/* 上板集中参数；修改后需重新编译烧录。*_MS 是 HAL 毫秒，*_TICKS 是 RTOS 节拍。
 * PID 积分限幅限制积分项，输出限幅限制该环输出，不等于机构机械限位。
 */

/* DM4310：Pitch 在 CAN1、Yaw 在 CAN2；CONTROL 为发送 ID，FEEDBACK 为回传 ID。 */
#define MOTOR4310_PITCH_CONTROL_CAN_ID      0x001U /* Pitch 控制标准 ID。 */
#define MOTOR4310_PITCH_FEEDBACK_CAN_ID     0x011U /* Pitch 反馈标准 ID。 */
#define MOTOR4310_CONTROL_CAN_ID            0x002U /* Yaw 控制标准 ID。 */
#define MOTOR4310_FEEDBACK_CAN_ID           0x012U /* Yaw 反馈标准 ID。 */
#define MOTOR4310_OFFLINE_TIMEOUT_MS        100U   /* 两轴任一反馈超过此时长未更新，即判离线。 */
#define MOTOR4310_DISABLE_RETRY_MS          50U    /* 保险状态下周期重发失能命令的间隔。 */
#define COMM_RC_TIMEOUT_MS                  100U   /* D1~D3 有效遥控帧超过此时长未更新，即判断控。 */

/* 4310 串级控制：位置误差用累计编码器计数，位置环输出速度原始码目标；
 * 速度环读取电机回传速度原始码，输出转矩原始码（约 -2048~2047）。
 */
#define MOTOR4310_SPEED_KP                  2.6f    /* 速度误差到转矩码的比例增益。 */
#define MOTOR4310_SPEED_KI                  0.0f    /* 速度环积分增益，积分按控制周期累积。 */
#define MOTOR4310_SPEED_KD                  0.004f    /* 速度环微分增益；0 为关闭。 */
#define MOTOR4310_SPEED_INTEGRAL_LIMIT      200.0f  /* 速度环积分项绝对值上限。 */
#define MOTOR4310_SPEED_OUTPUT_LIMIT        2047.0f /* 速度环输出转矩原始码上限。 */
#define MOTOR4310_POSITION_KP               0.32f    /* 位置计数误差到速度目标的比例增益。 */
#define MOTOR4310_POSITION_KI               0.0f    /* 位置环积分增益。 */
#define MOTOR4310_POSITION_KD               0.008f    /* 位置环微分增益；0 为关闭。 */
#define MOTOR4310_POSITION_INTEGRAL_LIMIT   100.0f  /* 位置环积分项绝对值上限。 */
#define MOTOR4310_POSITION_OUTPUT_LIMIT     700.0f  /* 位置环速度原始码目标上限。 */
#define MOTOR4310_CONTROL_PERIOD_S          0.001f  /* PID 单次调用周期，1 ms = 0.001 s。 */

/* 摩擦轮 3508：回传速度为 rpm，速度环输出为 C620 电流命令原始码。 */
#define MOTOR3508_CURRENT_LIMIT             16384   /* 发给 C620 的电流码绝对值上限。 */
#define MOTOR3508_MAX_SPEED_RPM             2000.0f /* 目标速度绝对值上限，rpm。 */
#define MOTOR3508_LEFT_DIRECTION            1.0f    /* 左轮目标速度方向系数。 */
#define MOTOR3508_RIGHT_DIRECTION          (-1.0f)  /* 右轮反转系数，和左轮等大反向。 */
#define MOTOR3508_OFFLINE_TIMEOUT_MS        100U    /* 反馈超时判离线的阈值。 */
#define MOTOR3508_SPEED_KP                  2.0f    /* rpm 误差到电流码的比例增益。 */
#define MOTOR3508_SPEED_KI                  1.0f    /* rpm 误差积分增益。 */
#define MOTOR3508_SPEED_KD                  0.0f    /* rpm 误差微分增益。 */
#define MOTOR3508_SPEED_INTEGRAL_LIMIT      500.0f  /* 速度环积分项绝对值上限。 */
#define MOTOR3508_SPEED_OUTPUT_LIMIT        5000.0f /* PID 电流码输出上限，另受 CURRENT_LIMIT 约束。 */
#define MOTOR3508_PID_CONTROL_TIME_S        0.001f  /* 摩擦轮 PID 调用周期，1 ms。 */

/* M2006：CAN1 0x204 回传，0x200 第 4 槽发送。 */
#define MOTOR2006_CURRENT_LIMIT             10000  /* C610 电流原始码绝对值限幅。 */
#define MOTOR2006_OFFLINE_TIMEOUT_MS        100U   /* 回传超过 100 ms 即离线。 */
#define MOTOR2006_COMMAND_TIMEOUT_MS        100U   /* 非零电流命令超过 100 ms 未更新则自动清零。 */
#define MOTOR2006_REDUCTION_RATIO           36.0f  /* 转子与减速箱输出轴转数比，36:1。 */
#define MOTOR2006_TORQUE_CONSTANT           0.18f  /* 转矩电流换算系数。 */
#define MOTOR2006_SPEED_KP                  7.5f   /* 速度比例增益。 */
#define MOTOR2006_SPEED_KI                  0.0f   /* 速度积分增益。 */
#define MOTOR2006_SPEED_KD                  0.0f   /* 速度微分增益。 */
#define MOTOR2006_SPEED_INTEGRAL_LIMIT      0.0f   /* 积分限幅。 */
#define MOTOR2006_SPEED_TORQUE_OUTPUT_LIMIT 600.0f /* 速度环输出限幅。 */
#define MOTOR2006_PID_CONTROL_TIME_S        0.001f /* 速度环单次调用周期，1 ms。 */

/* 升降：转子累计圈数以上电首帧为零；方向由实车安装决定。 */
#define LIFT_DOWN_DIRECTION                 -1.0f   /* 当前下降方向；反向改为 +1。 */
#define LIFT_DOWN_SPEED_RAD_S               500.0f   /* 下降转子目标速度，rad/s。 */
#define LIFT_UP_SPEED_RAD_S                 500.0f   /* 上升转子目标速度，rad/s。 */
#define LIFT_MAX_ROTOR_TURNS                330.0f  /* 上电零点起，正负方向各最多 340 转。 */
#define LIFT_LIMIT_SLOW_TURNS               1.0f   /* 临近行程边界的减速区，转子圈数。 */
#define LIFT_LIMIT_STOP_MARGIN_COUNTS       40     /* 距边界不足 40 计数时停机。 */
#define LIFT_YAW_DEADZONE_DEG               1.0f   /* 云台偏离当前正方向超过此角度即停。 */
#define LIFT_STOP_RETRY_MS                   50U    /* 停机帧每 50 ms 重发。 */
#define LIFT_FAULT_STOP_RETRY_MS             5U     /* 堵转、限位后每 5 ms 重发零电流。 */
#define LIFT_STALL_PROGRESS_COUNTS           40     /* 堵转时间窗内少于 40 计数视为未前进。 */
#define LIFT_DOWN_STALL_CURRENT_RAW         75    /* 下降堵转电流门槛；另有位移保护。 */
#define LIFT_DOWN_STALL_SPEED_RPM           1      /* 下降堵转速度门槛，转子 rpm。 */
#define LIFT_DOWN_STALL_TIME_MS             200U   /* 下降堵转判据持续时间。 */
#define LIFT_UP_STALL_CURRENT_RAW           520    /* 上升堵转电流门槛；另有位移保护。 */
#define LIFT_UP_STALL_SPEED_RPM             1      /* 上升堵转速度门槛，转子 rpm。 */
#define LIFT_UP_STALL_TIME_MS               500U   /* 上升堵转判据持续时间。 */
#define LIFT_CALIBRATE_UP_SPEED_RAD_S       180.0f  /* 找顶部时的转子速度，低于正常位控速度。 */
#define LIFT_CALIBRATE_TIMEOUT_MS           90000U /* 90 秒仍未找到顶部则停机。 */
#define LIFT_BACKOFF_TIMEOUT_MS             10000U /* 顶部回退 10 秒未到位则停机。 */
#define LIFT_TOP_BACKOFF_TURNS              5.0f   /* 顶部堵转点往下 5 圈作为上顶点。 */
#define LIFT_TRAVEL_TURNS                   316.0f /* 上顶点到下目标点的转子圈数。 */
#define LIFT_POSITION_KP_RAD_S_PER_TURN     5.5f   /* 每圈位置误差对应的转子目标速度。 */
#define LIFT_POSITION_MIN_SPEED_RAD_S       20.0f   /* 未到位时克服静摩擦的最小目标速度。 */
#define LIFT_BACKOFF_SPEED_RAD_S            5.0f  /* 顶部回退校准的最大转子速度。 */
#define LIFT_HOLD_SPEED_RAD_S               5.0f  /* 位置保持被外力推开后的最大回位速度。 */
#define LIFT_POSITION_TOLERANCE_COUNTS      80     /* 目标误差不超过 80 计数视为到位。 */
#define LIFT_POSITION_SETTLED_RPM           60     /* 顶部回退到位时的最大转子转速。 */
#define LIFT_CALIBRATION_SETTLE_MS          100U   /* 回退到位需稳定 100 ms。 */
#define LIFT_LOCK_TX_PERIOD_MS              10U    /* C2 锁车请求发送周期。 */
#define LIFT_CHASSIS_SPEED_TIMEOUT_MS       50U    /* D5 四轮转速超过 50 ms 未更新则禁止升降。 */
#define LIFT_CHASSIS_STOP_SPEED_RPM         200     /* 四轮转子转速均不超过此值才允许升降。 */
#define LIFT_CHASSIS_LOCK_SETTLE_MS         20U     /* 发出锁车请求后至少等待 20 ms 的新转速反馈。 */
#define LIFT_CHASSIS_RELEASE_RPM             10     /* 转子速度低于此值才解除底盘锁车。 */
#define LIFT_OFFLINE_RELEASE_MS              200U   /* 电机回传丢失后保持锁车 200 ms。 */

/* LK4005 拨盘：位置环输入累计 16 位编码器计数，输出目标速度 deg/s；
 * 速度环输入 deg/s，输出 0xA1 电流命令原始码；连发走独立速度环。
 */
#define DIAL_MOTOR_CAN_ID                   0x141U  /* 拨盘电机标准 CAN ID。 */
#define DIAL_MOTOR_CURRENT_LIMIT            2000    /* 0xA1 电流命令的最终绝对值限幅。 */
#define DIAL_MOTOR_OFFLINE_TIMEOUT_MS       100U    /* 拨盘反馈超时判离线的阈值。 */
#define DIAL_MOTOR_POSITION_KP              0.2f    /* 位置计数误差到目标 deg/s 的比例增益。 */
#define DIAL_MOTOR_POSITION_KI              0.0f    /* 位置环积分增益；0 为关闭。 */
#define DIAL_MOTOR_POSITION_KD              0.0f    /* 位置环微分增益；0 为关闭。 */
#define DIAL_MOTOR_POSITION_INTEGRAL_LIMIT  0.0f    /* 位置环积分项限幅；0 不保留积分贡献。 */
#define DIAL_MOTOR_POSITION_SPEED_LIMIT_DPS 7000.0f /* 位置环输出目标速度上限，deg/s。 */
#define DIAL_MOTOR_SPEED_KP                 0.2f    /* 单发位置内环速度比例增益。 */
#define DIAL_MOTOR_SPEED_KI                 0.0f    /* 单发位置内环速度积分增益。 */
#define DIAL_MOTOR_SPEED_KD                 0.005f    /* 单发位置内环速度微分增益。 */
#define DIAL_MOTOR_SPEED_INTEGRAL_LIMIT     500.0f    /* 单发、连发速度环共用的积分项限幅。 */
#define DIAL_MOTOR_SPEED_OUTPUT_LIMIT       1500.0f /* 两个速度环输出电流码上限。 */
#define DIAL_MOTOR_CONTINUOUS_SPEED_KP      3.0f   /* 连发独立速度环比例增益。 */
#define DIAL_MOTOR_CONTINUOUS_SPEED_KI      5.0f    /* 连发独立速度环积分增益。 */
#define DIAL_MOTOR_CONTINUOUS_SPEED_KD      0.005f  /* 连发独立速度环微分增益。 */
#define DIAL_MOTOR_PID_CONTROL_TIME_S       0.001f  /* 拨盘 PID 调用周期，1 ms。 */

/* 发射任务：两个摩擦轮和拨盘都需在线；3508 编号为驱动接口编号。 */
#define SHOOT_CONTROL_PERIOD_TICKS          1U    /* 发射控制任务周期，当前 1 tick = 1 ms。 */
#define SHOOT_LEFT_FRIC_MOTOR_ID            1U    /* 左摩擦轮驱动编号。 */
#define SHOOT_RIGHT_FRIC_MOTOR_ID           2U    /* 右摩擦轮驱动编号。 */
#define SHOOT_FRIC_TARGET_SPEED_RPM         1500  /* 两轮共同目标速度幅值，rpm。 */
/* 拨盘一圈按 2^16=65536 计数，一圈上弹一发；单发目标在旧累计目标上递增。
 * 连发目标速度 = 圈/秒 × 360 deg/s，现 15 圈/秒即 5400 deg/s。
 */
#define SHOOT_DIAL_ONE_BULLET_COUNTS        65536LL /* 一发的编码器累计增量；堵转退让也走一圈。 */
#define SHOOT_DIAL_FEED_DIRECTION           1LL     /* 正值代表逆时针上弹；改为 -1 反转。 */
#define SHOOT_DIAL_CONTINUOUS_ROUNDS_PER_S   15.0f   /* 连发拨盘目标圈速，圈/秒。 */
#define SHOOT_DIAL_ARRIVED_ERROR_COUNTS     500LL   /* |目标-反馈|≤500 计数视为到位，约 2.75°。 */
#define SHOOT_DIAL_SINGLE_MOVE_TIMEOUT_MS  500U    /* 单发未到位时最多等待的时长。 */
#define SHOOT_DIAL_BLOCK_CURRENT_THRESHOLD 600     /* 堵转判据：|反馈电流原始码|须大于此值。 */
#define SHOOT_DIAL_BLOCK_SPEED_THRESHOLD_DPS 10    /* 堵转判据：|反馈速度|须小于此值，deg/s。 */
#define SHOOT_DIAL_BLOCK_CONFIRM_TICKS      200U    /* 两个堵转判据连续满足 200 次，约 200 ms。 */
#define SHOOT_DIAL_STUCK_REVERSE_TIMEOUT_MS 200U    /* 堵转后反向退让的最长时间。 */
#define SHOOT_DIAL_STUCK_RELOAD_TIMEOUT_MS  200U    /* 退让后重试原目标的最长时间。 */
#define SHOOT_DIAL_SAFE_STOP_RETRY_MS       50U     /* 保险状态下停止帧重发间隔。 */
#define SHOOT_MOUSE_CONTINUOUS_THRESHOLD_MS 200U   /* 键鼠左键按住超过 200 ms 才切入连发；此前松开为单发。 */

/* 云台任务与摇杆：DBUS 通道约 -660~660，Pitch 摇杆目标速度原始码
 * = 通道值 / 660 × CLOUD_PITCH_MAX_SPEED_RAW。
 */
#define CLOUD_CONTROL_PERIOD_TICKS          1U     /* 云台任务周期，当前 1 tick = 1 ms。 */
#define CLOUD_TASK_STACK_BYTES              1024U  /* 云台任务栈大小，字节。 */
#define CLOUD_RC_MAX_VALUE                  660.0f /* DBUS 摇杆归一化分母。 */
#define RC_KEYBOARD_MOVE_RAW                440    /* WASD 普通移动的虚拟摇杆幅值。 */
#define RC_KEYBOARD_SPRINT_RAW              660    /* Shift 加速移动的虚拟摇杆幅值。 */
#define RC_KEYBOARD_SLOW_RAW                220    /* Ctrl 精细移动的虚拟摇杆幅值。 */
#define RC_KEYBOARD_MOUSE_YAW_GAIN          8      /* 鼠标 X 每计数换算的虚拟 Yaw 通道值。 */
#define RC_KEYBOARD_MOUSE_PITCH_GAIN        8      /* 鼠标 Y 每计数换算的 Pitch 通道值；方向已相对上一版反转。 */
#define RC_KEYBOARD_AIM_DIVISOR             2      /* 按住鼠标右键时视角输入减半。 */
#define CLOUD_RC_SPEED_ENTER                15     /* 静止转运动的摇杆阈值，原始通道值。 */
#define CLOUD_RC_SPEED_EXIT                 8      /* 运动转保持的较小阈值，构成滞回。 */
#define CLOUD_PITCH_MAX_SPEED_RAW           150    /* Pitch 满杆速度目标，电机速度原始码。 */
#define CLOUD_FOLLOW_RATE_TIMEOUT_MS        100U   /* D4 角速度快照有效期；当前 Yaw 闭环未用 D4。 */

/* BMI088 传感器到车体坐标绕 Z 轴转 180°：X/Y 取反、Z 不变。 */
#define GIMBAL_IMU_UPDATE_PERIOD_S          0.001f /* 姿态积分周期，1 ms。 */
#define GIMBAL_IMU_CALIBRATION_SAMPLES      1000U  /* 静止陀螺零偏采样数，每次隔 1 ms，约 1 s。 */
#define GIMBAL_IMU_ATTITUDE_KP              2.0f   /* 加速度重力方向修正姿态的比例增益。 */
#define GIMBAL_IMU_ATTITUDE_KI              0.02f  /* 姿态误差积分修正增益。 */
#define GIMBAL_IMU_YAW_RATE_FILTER_ALPHA    1.0f   /* 角速度低通新样本权重；1 为直接采用新值。 */

/* 惯性系 Yaw：摇杆积分成角度目标，再经角度环→deg/s 目标→速度环→转矩码。
 * 10 N·m 对应约 2047 转矩码，即约 204.7 码/N·m；0.04 N·m/(deg/s)
 * 换算约 8.19 码/(deg/s)。下方 Kp=9.0 是该量级上的实际调定值。
 */
#define CLOUD_YAW_COMMAND_RATE_DEG_S        200.0f /* 满杆目标角变化率；每周期增量=ch/660×200×0.001 度。 */
#define CLOUD_YAW_RC_DIRECTION              1.0f   /* 摇杆方向系数；调用处已将 ch[0] 取反。 */
#define CLOUD_YAW_ANGLE_KP                  8.0f  /* Yaw 角度误差 deg 到目标 deg/s 的比例增益。 */
#define CLOUD_YAW_ANGLE_KI                  0.0f   /* 角度外环积分增益。 */
#define CLOUD_YAW_ANGLE_KD                  0.0f   /* 角度外环微分增益。 */
#define CLOUD_YAW_ANGLE_INTEGRAL_LIMIT      200.0f /* 角度外环积分项绝对值上限。 */
#define CLOUD_YAW_RATE_TARGET_LIMIT_DEG_S   500.0f /* 角度外环输出角速度目标上限，deg/s。 */
#define CLOUD_YAW_RATE_KP                   9.0f   /* IMU 角速度误差到 4310 转矩码的比例增益。 */
#define CLOUD_YAW_RATE_KI                   0.0f   /* IMU 角速度内环积分增益。 */
#define CLOUD_YAW_RATE_KD                   0.0f   /* IMU 角速度内环微分增益。 */
#define CLOUD_YAW_RATE_INTEGRAL_LIMIT       0.0f   /* 角速度内环积分项限幅；0 不保留积分贡献。 */
#define CLOUD_YAW_TORQUE_LIMIT_RAW          2047.0f /* 角速度内环输出转矩码上限。 */

/* 电机单圈位置 0~65535 对应 -π~π rad；归中点换算成计数：
 * (HOME_RAD+π)×65535/(2π)，再选距当前累计角最近的一圈。
 * Pitch 机械限位以归中点为零，不是绝对编码器角。
 */
#define CLOUD_YAW_HOME_RAD                  (-0.387884378f) /* Yaw 指向车头的电机单圈角，rad。 */
#define CLOUD_PITCH_HOME_RAD                2.59309077f    /* Pitch 归中时电机单圈角，rad。 */
#define CLOUD_HOME_TOLERANCE_DEG            1.0f           /* 两轴位置需落在归中目标 ±1°。 */
#define CLOUD_HOME_SPEED_RAW_MAX            20             /* 两轴速度原始码绝对值须不超过此值。 */
#define CLOUD_HOME_STABLE_CYCLES            20U            /* 连续合格周期数；1 ms 周期约 20 ms。 */
#define CLOUD_PITCH_MIN_DEG                 (-7.0f)        /* 相对归中点的 Pitch 下限，度。 */
#define CLOUD_PITCH_MAX_DEG                 30.0f          /* 相对归中点的 Pitch 上限，度。 */
#define CLOUD_PITCH_LIMIT_SLOW_DEG          5.0f           /* 距限位不足 5° 时按剩余距离线性减速。 */
/* 拨轮从中位向上跨过负阈值触发一次调头；回到中位附近才重新布防。
 * 两阈值形成滞回，避免拨轮噪声重复触发。调头期间只让 Yaw 转向，底盘停车。
 */
#define CLOUD_TURN_WHEEL_TRIGGER_RAW        200            /* ch[4]≤-200 视为向上拨到触发位。 */
#define CLOUD_TURN_WHEEL_REARM_RAW          50             /* ch[4]>-50 时重新允许下一次触发。 */
#define CLOUD_SPIN_WHEEL_TRIGGER_RAW        200            /* 拨轮正向越过此值切换小陀螺。 */
#define CLOUD_SPIN_WHEEL_REARM_RAW          50             /* 拨轮回中位后才能再次切换。 */
#define CLOUD_FRONT_SWITCH_DEG              90.0f          /* |机械 Yaw 角|≥90° 时车尾更接近云台指向。 */
#define CLOUD_TURN_TOLERANCE_DEG            3.0f           /* Yaw 距反向车头目标的到位角差。 */
#define CLOUD_TURN_SPEED_RAW_MAX            20             /* 到位还要求 |Yaw 电机速度原始码|≤20。 */
#define CLOUD_TURN_STABLE_CYCLES            20U            /* 连续到位周期数，1 ms 周期约 20 ms。 */
/* 重力前馈：N·m=×SCALE；
 * 电机角=angle×2π/65535-π；转矩码=N·m×4095/(2×MAX_NM)。
 */
#define CLOUD_PITCH_GRAVITY_CENTER_RAD      2.678706762f /* 余弦重力曲线中心角，rad。 */
#define CLOUD_PITCH_GRAVITY_K               4.2072f      /* 余弦项幅值，N·m。 */
#define CLOUD_PITCH_GRAVITY_B               (-2.496f)    /* 恒定转矩偏置，N·m。 */
#define CLOUD_PITCH_GRAVITY_SCALE           0.6f         /* 总体前馈比例；减小可降低整条重力曲线幅值。 */
#define CLOUD_MOTOR_TORQUE_MAX_NM           10.0f        /* 转矩码换算采用的电机满量程，N·m。 */

#endif /* PARAMETER_H */
