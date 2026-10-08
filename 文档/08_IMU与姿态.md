# 下板 IMU与姿态

## 文件和数据流

`bottom_driven/IMU/imu.[ch]` 封装BMI088。SPI2和加速度/陀螺两个片选由外设层配置。Init校验传感器并设置量程；周期Update读取数据，按安装轴映射，启动静止采样减零偏，送入四元数EKF，生成欧拉角、累计Yaw与角速度。Get复制快照并返回已校准且在线的有效性。

## 参数与判断

参数位于 `imu_config`：启动采样、轴方向、量程换算、滤波、超时与 `attitude_ekf`。先核对轴映射再调滤波；加速度修正只用于姿态，不能消除Z轴航向长期漂移。`ekf_accel_used=false` 不一定离线，运动加速度可能被拒绝；需结合online和update_count判断。

## 移植与排查

适用于同类SPI BMI088与HAL平台。配置SPI、两个片选和延时接口，复制设备层+EKF，修改句柄/引脚/安装方向后Init，固定周期Update，控制调用Get。不要直接将原始计数送入控制环；两板角速度前馈必须统一方向。初始化错误查SPI与片选，标定不完成查启动静止条件，抖动还需检查机构松动与振动。

## 接口与排查补充

世界系线加速度已扣除重力，当前平移控制使用轮速目标。板间 D4 使用底盘 IMU 的 Yaw 角速度。

## 常用观察变量

加速度数组有效下标为 0～2，四元数为 0～3。当前工程发布去重力加速度，尚未据此建立平移速度闭环。

| Watch 表达式 | 单位 / 类型 | 含义 |
| --- | --- | --- |
| `chassis_imu.online` | bool | 驱动判定的传感器在线状态。 |
| `chassis_imu.calibrated` | bool | 启动静止零偏标定已完成。 |
| `chassis_imu.update_count` | 次 | 成功更新姿态的累计次数。 |
| `chassis_imu.init_error` | 次 | 初始化检测错误累计数。 |
| `chassis_imu.roll_deg` | deg | 横滚角。 |
| `chassis_imu.pitch_deg` | deg | 俯仰角。 |
| `chassis_imu.yaw_deg` | deg | 单圈航向角。 |
| `chassis_imu.yaw_total_deg` | deg | 跨圈累计航向角。 |
| `chassis_imu.yaw_rate_deg_s` | deg/s | Yaw 滤波后的角速度。 |
| `chassis_imu.gyro_rad_s[0]` | rad/s | 机体系 X 轴角速度；[1] Y 轴，[2] Z 轴。 |
| `chassis_imu.accel_m_s2[0]` | m/s² | 机体系 X 轴加速度，含重力；[1] Y 轴，[2] Z 轴。 |
| `chassis_imu.quaternion[0]` | 无量纲 | 姿态四元数 w；[1] x，[2] y，[3] z。 |
| `chassis_imu.ekf_bias_rad_s[0]` | rad/s | EKF 剩余零偏估计 X 分量；[1] Y 分量，[2] Z 分量。 |
| `chassis_imu.ekf_chi_square` | 残差指标 | 加速度量测残差检验值。 |
| `chassis_imu.ekf_accel_used` | bool | 本周期姿态更新接受加速度修正。 |
| `chassis_imu.temperature_c` | ℃ | BMI088 温度。 |
| `chassis_imu.linear_accel_world_m_s2[0]` | m/s² | 世界系去重力 X 轴线加速度；[1] Y 轴，[2] Z 轴。 |

完整速查与观察方法见 [本板观察变量速查](00_观察变量速查.md)。

[返回工程首页](../README.md)
