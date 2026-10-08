# 裁判模块说明

详细说明已统一到 [裁判系统模块文档](../../文档/05_裁判系统.md)，含外设、V2.0兼容、诊断变量与移植。工程总索引见 [README](../../README.md)。

## 常用观察变量

| Watch 表达式 | 单位 / 类型 | 含义 |
| --- | --- | --- |
| `referee_state.diagnostics.parsed_frame_count` | 帧 | 全部支持命令的解析成功次数。 |
| `referee_state.message[7].fresh` | bool | 热量与缓冲消息新鲜度。 |
| `referee_state.info.power_heat_data.buffer_energy` | J | 裁判缓冲能量。 |

完整变量、单位和有效条件见 [模块观察表](../../文档/05_裁判系统.md#常用观察变量)。
