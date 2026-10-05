#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "peripheral_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// 下板发送给上板的标准帧 ID。
#define COMM_CAN_RX_ID_D1  0x0D1U
#define COMM_CAN_RX_ID_D2  0x0D2U
#define COMM_CAN_RX_ID_D3  0x0D3U
#define COMM_CAN_RX_ID_D4  0x0D4U
#define COMM_CAN_RX_ID_D5  0x0D5U // 四个底盘电机的实时转速。

#define COMM_CAN_RX_ID_D6  0x0D6U // 裁判热量与发射许可。

// 上板发送给下板的标准帧 ID。
#define COMM_CAN_TX_ID_C1  0x0C1U
#define COMM_CAN_TX_ID_C2  0x0C2U
#define COMM_LIFT_LOCK_MAGIC 0xA6U

#define COMM_CAN_FRAME_SIZE  8U

// 下板传来的 DJI DBUS 原始遥控帧长度。
#define COMM_RC_FRAME_SIZE   18U

// 三档拨杆原始编码：上 1，中 3，下 2。
#define COMM_RC_SW_UP        1U
#define COMM_RC_SW_MID       3U
#define COMM_RC_SW_DOWN      2U

// DBUS 键盘位图。
#define COMM_RC_KEY_W      (1U << 0)
#define COMM_RC_KEY_S      (1U << 1)
#define COMM_RC_KEY_A      (1U << 2)
#define COMM_RC_KEY_D      (1U << 3)
#define COMM_RC_KEY_SHIFT  (1U << 4)
#define COMM_RC_KEY_CTRL   (1U << 5)
#define COMM_RC_KEY_Q      (1U << 6)
#define COMM_RC_KEY_E      (1U << 7)
#define COMM_RC_KEY_R      (1U << 8)
#define COMM_RC_KEY_F      (1U << 9)
#define COMM_RC_KEY_G      (1U << 10)
#define COMM_RC_KEY_Z      (1U << 11)
#define COMM_RC_KEY_X      (1U << 12)
#define COMM_RC_KEY_C      (1U << 13)
#define COMM_RC_KEY_V      (1U << 14)
#define COMM_RC_KEY_B      (1U << 15)

// 每个接收 ID 的最新快照。
typedef struct
{
    uint8_t data[COMM_CAN_FRAME_SIZE]; // 该 CAN ID 最近一次收到的 8 字节原始数据。
    uint32_t rx_count; // 该 CAN ID 累计收到的有效帧数。
    uint32_t last_rx_ms; // 最近一次收帧的 HAL 毫秒时间戳。
    bool received; // 上电后是否至少收到过一帧。
} Communication_CanRxFrame_t;

// 与下板遥控器解析结果保持一致，通道值范围通常为 -660~660。
typedef struct
{
    struct
    {
        int16_t ch[5]; // 五路遥控通道，均已减去中心值 1024。
        uint8_t s[2]; // 左、右三档拨杆的原始档位编码。
    } rc; // 与 DJI DBUS 数据布局对应的遥控数据。
    struct
    {
        int16_t x; // 鼠标向右位移为正。
        int16_t y; // 鼠标向下位移为正。
        int16_t z; // 鼠标滚轮值。
        bool left; // 鼠标左键。
        bool right; // 鼠标右键。
    } mouse;
    uint16_t key; // DBUS 键盘按下位图。
} Communication_RcControl_t;

// 遥控链路状态；任务通过 Communication_RC_Get 读取。
extern Communication_RcControl_t communication_rc; // 当前解析完成的遥控数据。
extern volatile bool communication_rc_online; // 遥控链路当前在线标志。
extern volatile uint32_t communication_rc_valid_count; // 有效遥控帧累计数。
extern volatile uint32_t communication_rc_last_valid_ms; // 最近有效帧时间。
extern volatile uint32_t communication_rc_assembly_error_count; // 拼帧错误数。

// 在 MX_CAN2_Init() 之后调用：配置精确 ID 过滤器、启动 CAN2 和接收中断。
HAL_StatusTypeDef Communication_CAN_Init(void);

// 发送固定 8 字节标准帧；std_id 只允许 0xC1 或 0xC2。
HAL_StatusTypeDef Communication_CAN_Send(uint16_t std_id,
                                         const uint8_t data[COMM_CAN_FRAME_SIZE]);
HAL_StatusTypeDef Communication_CAN_SendC1(
    const uint8_t data[COMM_CAN_FRAME_SIZE]);
HAL_StatusTypeDef Communication_CAN_SendC2(
    const uint8_t data[COMM_CAN_FRAME_SIZE]);
// C2: [0]=标识，[1] bit0=锁车，[2]=请求序号。
HAL_StatusTypeDef Communication_CAN_SendLiftLock(bool hold, uint8_t sequence);
// C1: Yaw 角(0.01°)及调头标志；D4: 底盘角速度(0.01°/s)；D5: 四轮 rpm。
HAL_StatusTypeDef Communication_CAN_SendYawAngle(float angle_deg);
// C1 byte[2] bit0=Yaw角有效，bit1=调头中，bit2=允许调头，
// bit3=允许自旋，bit4=上板已选小陀螺，bit5=低位禁止云台模式；模式位与角度有效位独立。
HAL_StatusTypeDef Communication_CAN_SendYawState(float angle_deg, bool turning,
                                                  bool allow_turn,
                                                  bool allow_spin,
                                                  bool spin_selected,
                                                  bool angle_valid,
                                                  bool bottom_mode_blocked);
bool Communication_CAN_GetChassisYawRate(float *rate_deg_s);
bool Communication_CAN_GetChassisYawRateState(float *rate_deg_s, uint32_t *sample_ms);

// 获取指定 D1~D6 的最新完整快照；尚未收到或参数错误时返回 false。
bool Communication_CAN_GetLatest(uint16_t std_id,
                                 Communication_CanRxFrame_t *frame);

typedef struct {
    uint16_t heat, limit, cooling; // 当前热量、上限、每秒冷却量。
    uint8_t sequence; // 原裁判热量消息更新序号。
    bool valid, output_allowed; // 下板数据新鲜度和发射许可。
    uint32_t last_rx_ms; // 板间热量帧到达时刻。
} Communication_HeatSnapshot_t;
bool Communication_GetHeatSnapshot(Communication_HeatSnapshot_t *heat);

// 处理 18 字节遥控帧并检测断联，建议每 1~10 ms 调用。
void Communication_Process(void);

// 原子复制当前遥控结果；返回值表示遥控链路当前是否在线。
bool Communication_RC_Get(Communication_RcControl_t *control);
bool Communication_RC_IsOnline(void);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const volatile Communication_CanRxFrame_t *frames; // D1~D6 原始帧；控制读取用 ops.can_get_latest。
    const Communication_RcControl_t *remote; // 拼帧完成后的遥控解析结果。
} BoardLinkModuleDataRefs;

typedef struct
{
    const volatile bool *remote_online; // 遥控链路在线状态。
    const volatile uint32_t *valid_count; // 有效遥控帧累计数。
    const volatile uint32_t *last_valid_ms; // 最近有效遥控帧时刻。
    const volatile uint32_t *assembly_errors; // 遥控拼帧错误数。
} BoardLinkModuleDiagnosticsRefs;

typedef struct
{
    volatile CommunicationConfig *config; // 当前可调驱动参数。
    BoardLinkModuleDataRefs data; // 反馈与解析数据引用。
    BoardLinkModuleDiagnosticsRefs diagnostics; // 通信诊断引用。

    // 初始化。
    HAL_StatusTypeDef (*can_init)(void); // 初始化板间 CAN 接收。

    // 数据读取与在线检查。
    bool (*can_get_chassis_yaw_rate)(float *rate_deg_s); // 读取下板 Yaw 角速度。
    bool (*can_get_chassis_yaw_rate_state)(float *rate_deg_s, uint32_t *sample_ms); // 读取下板角速度及采样时刻。
    bool (*can_get_latest)(uint16_t std_id, Communication_CanRxFrame_t *frame); // 复制指定 ID 的原始接收帧。
    bool (*get_heat_snapshot)(Communication_HeatSnapshot_t *heat); // 复制热量与供弹许可快照。
    bool (*rc_get)(Communication_RcControl_t *control); // 复制已解析遥控数据。
    bool (*rc_is_online)(void); // 读取遥控链路在线状态。

    // 控制与发送。
    HAL_StatusTypeDef (*can_send)(uint16_t std_id, const uint8_t data[COMM_CAN_FRAME_SIZE]); // 发送板间标准帧。
    HAL_StatusTypeDef (*can_send_c1)(const uint8_t data[COMM_CAN_FRAME_SIZE]); // 发送云台状态帧。
    HAL_StatusTypeDef (*can_send_c2)(const uint8_t data[COMM_CAN_FRAME_SIZE]); // 发送升降锁车帧。
    HAL_StatusTypeDef (*can_send_lift_lock)(bool hold, uint8_t sequence); // 编码并发送升降锁车状态。
    HAL_StatusTypeDef (*can_send_yaw_angle)(float angle_deg); // 编码并发送云台角度。
    HAL_StatusTypeDef (*can_send_yaw_state)(
        float angle_deg, bool turning, bool allow_turn, bool allow_spin, bool spin_selected,
        bool angle_valid, bool bottom_mode_blocked); // 编码并发送云台角度与模式许可。

    // 状态维护。
    void (*process)(void); // 周期处理接收数据和在线状态。
} BoardLinkModule;

extern const BoardLinkModule board_link; // 模块统一访问入口。

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_H
