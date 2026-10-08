# 双板步兵工程 27_LHX

本仓库采用上下板分支分别维护固件。`main` 存放整车总览、模块文档索引和工程复盘；完整源码、Keil 工程及板级说明位于对应分支。

## 目录

- [工程入口](#工程入口)
- [工程复盘](#工程复盘)
- [模块文档总索引](#模块文档总索引)
- [常用观察变量](#常用观察变量)
- [整体控制链路](#整体控制链路)
- [两板通信速览](#两板通信速览)
- [开发与验证](#开发与验证)

## 工程入口

| 分支 | 控制器与职责 | 入口 |
| --- | --- | --- |
| [infantry_up](https://github.com/RP27-EC/27_LHX/tree/infantry_up) | STM32F407 上板：Yaw/Pitch、升降、发射、热量与弹速自适应 | [上板 README](https://github.com/RP27-EC/27_LHX/blob/infantry_up/README.md) · [Keil 工程](https://github.com/RP27-EC/27_LHX/tree/infantry_up/MDK-ARM) |
| [infantry_down](https://github.com/RP27-EC/27_LHX/tree/infantry_down) | STM32H723 下板：DBUS、四轮底盘、功率、裁判、超电与无线充 | [下板 README](https://github.com/RP27-EC/27_LHX/blob/infantry_down/README.md) · [Keil 工程](https://github.com/RP27-EC/27_LHX/tree/infantry_down/MDK-ARM) |

## 工程复盘

[RP步兵工程复盘焚决 Word](RP%E6%AD%A5%E5%85%B5%E5%B7%A5%E7%A8%8B%E5%A4%8D%E7%9B%98%E7%84%9A%E5%86%B3.docx)

记录 20 个调车与工程案例，按“遇到的问题 → 解决办法 → 根本原因 → 学到的教训”整理，包含目录、页码及常用观察入口。内容覆盖云台、调头与归中、升降联锁、发射与热量、功率与超电、协议时效、外设配置和调试观察。尚未定位的问题保留待验证原因。

## 模块文档总索引

| 模块 | 上板 | 下板 |
| --- | --- | --- |
| 启动与任务 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/01_%E5%90%AF%E5%8A%A8%E4%B8%8E%E4%BB%BB%E5%8A%A1.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/01_%E5%90%AF%E5%8A%A8%E4%B8%8E%E4%BB%BB%E5%8A%A1.md) |
| 遥控与模式 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/02_%E9%81%A5%E6%8E%A7%E4%B8%8E%E6%A8%A1%E5%BC%8F.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/02_%E9%81%A5%E6%8E%A7%E4%B8%8E%E6%A8%A1%E5%BC%8F.md) |
| 云台与调头 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/03_%E4%BA%91%E5%8F%B0%E4%B8%8E%E8%B0%83%E5%A4%B4.md) | — |
| 升降与安全联锁 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/04_%E5%8D%87%E9%99%8D%E4%B8%8E%E5%AE%89%E5%85%A8%E8%81%94%E9%94%81.md) | — |
| 发射与弹速自适应 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/05_%E5%8F%91%E5%B0%84%E6%9C%BA%E6%9E%84.md) | — |
| 热量控制 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/11_%E7%83%AD%E9%87%8F%E6%8E%A7%E5%88%B6.md) | — |
| 底盘与麦轮 | — | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/03_%E5%BA%95%E7%9B%98%E4%B8%8E%E9%BA%A6%E8%BD%AE.md) |
| 功率控制与在线辨识 | — | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/04_%E5%8A%9F%E7%8E%87%E6%8E%A7%E5%88%B6.md) |
| 裁判系统 | — | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/05_%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F.md) |
| 超电与无线充 | — | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/06_%E8%B6%85%E7%94%B5%E4%B8%8E%E6%97%A0%E7%BA%BF%E5%85%85.md) |
| 电机驱动 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/06_%E7%94%B5%E6%9C%BA%E9%A9%B1%E5%8A%A8.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/07_%E7%94%B5%E6%9C%BA%E9%A9%B1%E5%8A%A8.md) |
| IMU 与姿态 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/07_IMU%E4%B8%8E%E5%A7%BF%E6%80%81.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/08_IMU%E4%B8%8E%E5%A7%BF%E6%80%81.md) |
| 通信与总线 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/08_%E9%80%9A%E4%BF%A1%E4%B8%8E%E6%80%BB%E7%BA%BF.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/09_%E9%80%9A%E4%BF%A1%E4%B8%8E%E6%80%BB%E7%BA%BF.md) |
| 算法库 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/09_%E7%AE%97%E6%B3%95%E5%BA%93.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/10_%E7%AE%97%E6%B3%95%E5%BA%93.md) |
| 配置与调参 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/10_%E9%85%8D%E7%BD%AE%E4%B8%8E%E8%B0%83%E5%8F%82.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/11_%E9%85%8D%E7%BD%AE%E4%B8%8E%E8%B0%83%E5%8F%82.md) |
| 模块接口与移植 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/12_%E6%A8%A1%E5%9D%97%E6%8E%A5%E5%8F%A3%E4%B8%8E%E7%9B%AE%E5%BD%95.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/12_%E6%A8%A1%E5%9D%97%E6%8E%A5%E5%8F%A3%E4%B8%8E%E7%9B%AE%E5%BD%95.md) |
| 耗时与优化检查 | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/13_%E4%BB%BB%E5%8A%A1%E8%80%97%E6%97%B6%E4%B8%8E%E4%BC%98%E5%8C%96%E6%A3%80%E6%9F%A5.md) | [模块说明与观察表](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/13_%E4%BB%BB%E5%8A%A1%E8%80%97%E6%97%B6%E4%B8%8E%E4%BC%98%E5%8C%96%E6%A3%80%E6%9F%A5.md) |

## 常用观察变量

- [上板观察变量速查](https://github.com/RP27-EC/27_LHX/blob/infantry_up/%E6%96%87%E6%A1%A3/00_%E8%A7%82%E5%AF%9F%E5%8F%98%E9%87%8F%E9%80%9F%E6%9F%A5.md)：云台、升降、发射、热量、电机、IMU、板间通信与任务诊断。
- [下板观察变量速查](https://github.com/RP27-EC/27_LHX/blob/infantry_down/%E6%96%87%E6%A1%A3/00_%E8%A7%82%E5%AF%9F%E5%8F%98%E9%87%8F%E9%80%9F%E6%9F%A5.md)：底盘、功率预测、RLS、裁判、缓冲、超电、电机与总线。

各模块观察表列出完整 Watch 路径、单位、数组下标和有效条件。两板分别加载对应 AXF；重新编译后同步更新调试符号和采样地址。参数默认值以配置初始化文件及运行值为准。

## 整体控制链路

| 环节 | 数据与处理 |
| --- | --- |
| 输入 | 下板接收、校验 DBUS 并转发 D1～D3；两板维护遥控模式、输入边沿和重新布防。 |
| 云台与升降 | 上板先更新 IMU 和升降安全快照，再执行归中、云台控制和升降动作。机械 Yaw 使用编码器位置外环与 IMU 速度内环，底盘角速度参与前馈。 |
| 发射 | 上板检查遥控、升降与电机许可，执行拨盘固定相位保持、单发和连发；本地热量由裁判校准，摩擦轮按枪管测速调整。 |
| 底盘与功率 | 四轮独立 PID 和固定扭矩前馈产生电流请求；RLS 辨识损耗、模型预测与超电实测反馈共同分配功率，动态上限取裁判系统。 |
| 安全 | 遥控、设备和消息分别检查超时；升降校准请求锁车，顶部和低位区域影响特殊动作许可；离线轮持续零电流，其余轮继续控制。 |

## 两板通信速览

板间链路为经典 CAN、11 位标准 ID、8 字节帧；下板使用 FDCAN2，上板使用 CAN2。多字节量按小端，详细位定义、缩放和超时见两板通信文档。

| 方向 | ID | 用途 |
| --- | --- | --- |
| 下板 → 上板 | 0xD1～0xD3 | 原始 18 字节 DBUS 分片，整组检查顺序与时效。 |
| 下板 → 上板 | 0xD4、0xD5 | 底盘 Yaw 角速度、四轮实测转子速度。 |
| 下板 → 上板 | 0xD6 | 裁判热量、热量上限、冷却量、发射电源许可与热量序号。 |
| 下板 → 上板 | 0xD7 | 枪管弹速、弹速上限、测速序号、有效标志与样本年龄。 |
| 上板 → 下板 | 0xC1 | 机械相对 Yaw 角、调头状态、自旋与低位模式许可。 |
| 上板 → 下板 | 0xC2 | 升降上电校准锁车请求及请求序号。 |

## 开发与验证

1. 在对应分支的 `MDK-ARM` 打开 Keil 工程。两板各自编译、下载，流程和移植步骤见板级 README。
2. 应用与底层驱动的配置文件统一位于各板根目录 `config/`。参数按部件、模式和功能分类，默认值在上电初始化时加载；运行时调参确认后回填初始化函数。
3. 修改板间协议、过滤器、任务周期、坐标方向和失联策略时，核对两板并成对编译、烧录。周期变化后同步检查 PID 时间、心跳、堵转计数与通信超时。
4. PC 验证检查软件状态和恢复逻辑；实车验证电机方向、机构行程、轨迹跟踪、功率和发射效果。耗时文档中的预算与实际 MCU 测量分别记录。

文档同步日期：2026年10月8日。模块功能与可调数值以对应固件分支的当前代码为准。
