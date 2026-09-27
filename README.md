# 双板步兵工程 27_LHX

本仓库采用上下板分支分别维护固件。`main` 只放工程总览；完整源码、Keil 工程和板级说明分别位于以下分支。

| 分支 | 控制器与职责 | 从这里开始 |
| --- | --- | --- |
| [`infantry_up`](https://github.com/RP27-EC/27_LHX/tree/infantry_up) | STM32F4 上板：云台 Yaw/Pitch、升降、摩擦轮与拨盘发射、上板 IMU | [上板 README](https://github.com/RP27-EC/27_LHX/blob/infantry_up/README.md) · [Keil 工程](https://github.com/RP27-EC/27_LHX/tree/infantry_up/MDK-ARM) |
| [`infantry_down`](https://github.com/RP27-EC/27_LHX/tree/infantry_down) | STM32H7 下板：DBUS 接收、四轮麦轮底盘、下板 IMU、超级电容与无线充通信 | [下板 README](https://github.com/RP27-EC/27_LHX/blob/infantry_down/README.md) · [Keil 工程](https://github.com/RP27-EC/27_LHX/tree/infantry_down/MDK-ARM) |

## 整体控制链路

| 环节 | 数据与处理 |
| --- | --- |
| 输入 | 下板从 UART/DMA 接收并校验 18 字节 DBUS，解析遥控拨杆、拨轮与键鼠；原始帧分三条 CAN 帧转发给上板。两板各自维护模式状态机和边沿保险。 |
| 云台与发射 | 上板结合电机反馈及 BMI088 解算控制 Yaw/Pitch；独立任务处理升降校准、高低位及摩擦轮、单发/连发。 |
| 底盘 | 下板依据上板回传的机械归中 Yaw 角，执行机械、底盘跟随云台、小陀螺及调头协同；麦轮逆解后以四路 3508 速度 PID 控制。 |
| 安全 | 遥控/反馈超时进入停车或失能；发射和物理小陀螺要求拨杆变化后重新布防；升降上电找顶校准期间使用短时锁车请求。 |

## 两板通信速览

板间链路为经典 CAN、11 位标准 ID、8 字节帧；下板使用 FDCAN2，上板使用 CAN2。多字节量按小端，具体位定义、缩放和超时以两分支 README 与当前代码为准。

| 方向 | ID | 用途 |
| --- | --- | --- |
| 下板 → 上板 | `0xD1`～`0xD3` | 18 字节原始 DBUS，按 `[0..7]`、`[8..15]`、`[16..17]` 分帧。 |
| 下板 → 上板 | `0xD4`、`0xD5` | 底盘 IMU Yaw 角速度、四轮实测转子速度。 |
| 上板 → 下板 | `0xC1` | 云台相对机械零点的 Yaw 角及调头标志。 |
| 上板 → 下板 | `0xC2` | 升降上电校准期间的底盘锁车请求。 |

## 开发与验证

1. 分别切换到目标分支，在 `MDK-ARM/` 打开对应 `.uvprojx`；不要把上板工程和下板工程当作同一份 Keil 目标混编。两板的目录与外设初始化、控制任务、参数入口见各自 README。
2. 应用策略参数放 `user/application_config.[ch]`，电机/IMU/CAN 等底层参数放 `bottom_driven/peripheral_config.[ch]`；默认值在上电时初始化一次，便于在线调参。
3. 任何板间协议、CAN 滤波、任务周期、坐标符号或失联策略修改，都要核对**两边代码**并分别编译、成对烧录。修改周期时同步检查 PID 时间基准、心跳、堵转确认周期及超时。
4. 上板前先核对电源、机械限位、电机转向和反馈在线；在台架上依次验证遥控断联、调头锁车释放、升降校准/保护及发射保险。`HAL_OK` 只代表 CAN 帧入队，不代表电机执行。

当前功能以两个开发分支的实际代码为准；本总览用于快速定位，具体操作映射、故障排查和控制参数请阅读对应板的 README。
