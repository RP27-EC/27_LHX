#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "peripheral_config.h"

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

// 两板通信使用 FDCAN2、11 位标准 ID、经典 CAN、8 字节数据帧。
#define COMMUNICATION_FRAME_SIZE       8U

// 下板发送给上板的报文 ID。
#define COMMUNICATION_TX_ID_D1         0x0D1U
#define COMMUNICATION_TX_ID_D2         0x0D2U
#define COMMUNICATION_TX_ID_D3         0x0D3U
#define COMMUNICATION_TX_ID_D4         0x0D4U
#define COMMUNICATION_TX_ID_D5         0x0D5U // 四轮转子转速。

#define COMMUNICATION_TX_ID_D6         0x0D6U // 裁判热量与发射许可。
#define COMMUNICATION_TX_ID_D7         0x0D7U // 枪管测速、弹速上限和样本年龄。

// 上板发送给下板的报文 ID，也是下板滤波器唯一放行的两个 ID。
#define COMMUNICATION_RX_ID_C1         0x0C1U
#define COMMUNICATION_RX_ID_C2         0x0C2U
#define COMMUNICATION_LIFT_LOCK_MAGIC  0xA6U

typedef struct
{
    uint8_t data[COMMUNICATION_FRAME_SIZE]; // 最近一帧的原始 8 字节数据。
    uint32_t last_rx_ms; // 最近一次接收时的 HAL 毫秒时间。
    uint32_t rx_count; // 该 ID 累计收到的有效帧数。
    bool received; // 上电后是否至少收到过一帧。
} Communication_RxFrame;

// 板间通信状态。
extern volatile uint32_t communication_rx_count;
extern volatile uint32_t communication_last_rx_id;
extern volatile uint32_t communication_bus_off_count; // CAN2 Bus-Off 恢复尝试次数。
extern volatile uint32_t communication_restart_count; // CAN2 成功重新启动次数。

// 在 MX_FDCAN2_Init() 之后调用：配置 C1/C2 滤波器、启动 FDCAN2 并开启 FIFO0 中断。
HAL_StatusTypeDef Communication_Init(void);

// 通信任务周期调用：CAN2 进入 Bus-Off 时限频重启，使硬件自动重发恢复工作。
void Communication_Service(void);

// 发送一帧到上板。std_id 只允许 D1~D7，data 必须指向 8 字节数据。
HAL_StatusTypeDef Communication_Send(uint32_t std_id,
                                     const uint8_t data[COMMUNICATION_FRAME_SIZE]);

// 原子复制 C1 或 C2 的最近接收结果；尚未收到或 ID 非法时返回 false。
bool Communication_GetRxFrame(uint32_t std_id, Communication_RxFrame *frame);

// C1: Yaw 角(0.01°)、调头状态及允许标志；D4: 底盘角速度(0.01°/s)。
// C2: 升降锁车；D5: 四轮转速。
bool Communication_GetYawAngle(float *angle_deg);
// 读取 C1 的机械角、调头中及允许调头标志；帧超时/无效时返回 false。
bool Communication_GetYawState(float *angle_deg, bool *turning,
                               bool *turn_allowed);
bool Communication_GetSpinState(bool *upper_selected, bool *spin_allowed);
// C1 bit5：上板确认升降接近低位；帧超时则返回 false。
bool Communication_GetBottomModeBlocked(void);
bool Communication_GetLiftLock(uint8_t *sequence);
HAL_StatusTypeDef Communication_SendChassisYawRate(float rate_deg_s);
HAL_StatusTypeDef Communication_SendChassisYawRateState(float rate_deg_s, bool valid);
// D5 每个电机占两个字节，int16 小端，单位 rpm。
HAL_StatusTypeDef Communication_SendChassisWheelSpeeds(const int16_t speed_rpm[4]);

// D6：热量/上限/冷却量各 uint16 小端，许可标志、热量更新序号。
HAL_StatusTypeDef Communication_SendHeatState(uint16_t heat, uint16_t limit,
    uint16_t cooling, bool valid, bool output_allowed, uint8_t sequence);
// D7：两项弹速均为 0.01m/s，小端；序号、有效位及样本年龄。
HAL_StatusTypeDef Communication_SendShotState(float speed_m_s, float limit_m_s,
    bool valid, bool limit_valid, uint16_t sequence, uint32_t age_ms);

// 由统一 HAL FDCAN FIFO0 回调调用，不应由任务代码直接调用。
void Communication_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t interrupts);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const Communication_RxFrame *c1; // 云台角度和模式许可原始帧。
    const Communication_RxFrame *c2; // 升降锁车原始帧。
} BoardLinkModuleDataRefs;

typedef struct
{
    const volatile uint32_t *rx_count; // 板间接收帧累计数。
    const volatile uint32_t *last_rx_id; // 最近接收 ID。
    const volatile uint32_t *bus_off_count; // 总线故障恢复尝试数。
    const volatile uint32_t *restart_count; // 总线恢复成功数。
} BoardLinkModuleDiagnosticsRefs;

typedef struct
{
    volatile CommunicationConfig *config; // 当前可调驱动参数。
    BoardLinkModuleDataRefs data; // 反馈与解析数据引用。
    BoardLinkModuleDiagnosticsRefs diagnostics; // 通信诊断引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get_rx_frame)(uint32_t std_id, Communication_RxFrame *frame); // 复制指定 ID 的原始接收帧。
    bool (*get_yaw_angle)(float *angle_deg); // 读取云台机械角。
    bool (*get_yaw_state)(float *angle_deg, bool *turning, bool *turn_allowed); // 读取云台机械角及调头状态。
    bool (*get_spin_state)(bool *upper_selected, bool *spin_allowed); // 读取小陀螺选择与许可。
    bool (*get_bottom_mode_blocked)(void); // 读取低位云台模式限制。
    bool (*get_lift_lock)(uint8_t *sequence); // 读取升降锁车请求与序号。

    // 控制与发送。
    HAL_StatusTypeDef (*send)(uint32_t std_id, const uint8_t data[COMMUNICATION_FRAME_SIZE]); // 发送标准数据帧。
    HAL_StatusTypeDef (*send_chassis_yaw_rate)(float rate_deg_s); // 发送底盘 Yaw 角速度。
    HAL_StatusTypeDef (*send_chassis_yaw_rate_state)(float rate_deg_s, bool valid); // 发送底盘角速度和有效标志。
    HAL_StatusTypeDef (*send_chassis_wheel_speeds)(const int16_t speed_rpm[4]); // 发送四轮反馈转速。
    HAL_StatusTypeDef (*send_heat_state)(
        uint16_t heat, uint16_t limit, uint16_t cooling, bool valid, bool output_allowed,
        uint8_t sequence); // 发送裁判热量与供弹许可。
    HAL_StatusTypeDef (*send_shot_state)(float speed_m_s, float limit_m_s,
        bool valid, bool limit_valid, uint16_t sequence, uint32_t age_ms); // 发送主枪管测速快照。

    // 状态维护。
    void (*service)(void); // 周期维护模块通信状态。

    // 接收与解析。
    void (*fdcan_rx_fifo0_callback)(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts); // 处理板间 FDCAN 接收中断。
} BoardLinkModule;

extern const BoardLinkModule board_link; // 模块统一访问入口。

#endif
