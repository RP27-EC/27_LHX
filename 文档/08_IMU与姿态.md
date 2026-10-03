# 下板 IMU与姿态

## 文件和数据流

`bottom_driven/IMU/imu.[ch]` 封装BMI088。SPI2和加速度/陀螺两个片选由外设层配置。Init校验传感器并设置量程；周期Update读取数据，按安装轴映射，启动静止采样减零偏，送入四元数EKF，生成欧拉角、累计Yaw与角速度。Get复制快照并返回已校准且在线的有效性。

## 变量

| `chassis_imu` 成员 | 单位及作用 |
| --- | --- |
| `gyro_rad_s[3]` | 启动标定和在线零偏修正后的机体系角速度，rad/s |
| `accel_m_s2[3]` | 机体系加速度，m/s²，静止时包含重力 |
| `quaternion[4]` | w,x,y,z姿态，不是欧拉角 |
| `roll_deg/pitch_deg/yaw_deg` | 欧拉角，deg；Yaw约−180～180 |
| `yaw_total_deg` | 跨圈累计Yaw，deg |
| `yaw_rate_deg_s` | Yaw控制角速度，deg/s，受Yaw低通参数影响 |
| `ekf_bias_rad_s/ekf_chi_square/ekf_accel_used` | 剩余零偏、残差检验、是否使用加速度修正 |
| `calibrated/online/update_count/init_error` | 启动标定、在线、成功更新次数和初始化错误 |

下板另有 `linear_accel_world_m_s2[3]`：世界系去重力加速度，当前没有据此积分平移速度做闭环。D4使用yaw_rate_deg_s。

## 参数与判断

参数位于 `imu_config`：启动采样、轴方向、量程换算、滤波、超时与 `attitude_ekf`。先核对轴映射再调滤波；加速度修正只用于姿态，不能消除Z轴航向长期漂移。`ekf_accel_used=false` 不一定离线，运动加速度可能被拒绝；需结合online和update_count判断。

## 移植与排查

适用于同类SPI BMI088与HAL平台。配置SPI、两个片选和延时接口，复制设备层+EKF，修改句柄/引脚/安装方向后Init，固定周期Update，控制调用Get。不要直接将原始计数送入控制环；两板角速度前馈必须统一方向。初始化错误查SPI与片选，标定不完成查启动静止条件，抖动还需检查机构松动与振动。

[返回工程首页](../README.md)
