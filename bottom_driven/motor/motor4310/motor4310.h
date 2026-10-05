#ifndef MOTOR4310_H
#define MOTOR4310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "peripheral_config.h"
#include <stdbool.h>
#include <stdint.h>

// MIT 协议固定量；可调驱动参数见 peripheral_config.h。
#define MOTOR4310_PITCH_CONTROL_CAN_ID  0x001U
#define MOTOR4310_PITCH_FEEDBACK_CAN_ID 0x011U
#define MOTOR4310_CONTROL_CAN_ID        0x002U
#define MOTOR4310_FEEDBACK_CAN_ID       0x012U
#define MOTOR4310_CAN_FRAME_SIZE         8U // 经典 CAN 数据帧长度，字节。
#define MOTOR4310_PMAX                   3.14159265359f // 单圈位置范围为 -π~π rad。
// 位置回传 16 位，-π~π 映射到 0~65535：一圈计数=2π×65535/(2×PMAX)，约 65535。
#define MOTOR4310_ECD_PER_ROUND          \
    (6.28318530718f * 65535.0f / (2.0f * MOTOR4310_PMAX))

typedef enum
{
    MOTOR4310_PITCH = 0, // Pitch 轴电机数组下标。
    MOTOR4310_YAW = 1, // Yaw 轴电机数组下标。
    MOTOR4310_COUNT = 2 // 云台 4310 电机总数。
} Motor4310_Id_t;

typedef struct
{
    uint16_t angle; // MIT 反馈的单圈位置原始值。
    int16_t speed; // MIT 反馈的速度原始值。
    int16_t torque; // MIT 反馈的转矩原始值。
    uint8_t temperature; // 电机温度，单位摄氏度。
    uint8_t state; // MIT 反馈帧中的电机状态码。
    int32_t total_angle; // 以编码器数值 0 为零点的累计计数。
    uint16_t last_angle; // 上一帧单圈位置，用于累计圈数。
    float angle_degree; // 当前累计位置换算得到的角度。
    volatile bool initialized; // 是否已用首帧建立累计位置。
    volatile uint32_t rx_count; // 累计接收的有效反馈帧数。
    volatile uint32_t last_rx_ms; // 最近一次反馈的毫秒时间戳。
    volatile bool online; // 心跳检测得到的当前在线状态。
    volatile bool enabled; // 最近一次成功入队的使能/失能命令状态。
    uint32_t last_enable_ms; // 最近一次发送使能命令的时间。
    volatile bool disable_command_sent; // 上次失能命令的入队标志，断联时周期重发。
    uint32_t last_disable_ms; // 最近一次发送失能命令的时间。
} Motor4310_Data_t;

extern Motor4310_Data_t motor4310_data[MOTOR4310_COUNT];
extern PID_Controller_t motor4310_speed_pids[MOTOR4310_COUNT];
extern PID_Controller_t motor4310_position_pids[MOTOR4310_COUNT];
// 分别配置 CAN1/2 的反馈过滤器；CAN2 与板间通信共用接收回调。
HAL_StatusTypeDef Motor4310_Init(void);
// 两轴独立指定 id 控制；位置/速度闭环要求两轴同时在线。
bool Motor4310_OnlineCheck(Motor4310_Id_t id);
bool Motor4310_AllOnline(void);
bool Motor4310_GetFeedback(Motor4310_Id_t id, Motor4310_Data_t *feedback);
void Motor4310_Heartbeat(void);
void Motor4310_ResetControl(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_EnableMotor(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_DisableMotor(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_PositionControlMotor(Motor4310_Id_t id,
                                                 int32_t target_position);
HAL_StatusTypeDef Motor4310_PositionControlWithFeedforward(
    Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw);
HAL_StatusTypeDef Motor4310_PositionControlWithProfile(
    Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw,
    const Motor4310_PidProfile_t *profile, bool enable_yaw_feedforward);
HAL_StatusTypeDef Motor4310_SpeedControlMotor(Motor4310_Id_t id,
                                              int16_t target_speed);
HAL_StatusTypeDef Motor4310_SpeedControlWithFeedforward(
    Motor4310_Id_t id, int16_t target_speed, int16_t feedforward_raw);
HAL_StatusTypeDef Motor4310_SetTorqueRawMotor(Motor4310_Id_t id,
                                              int16_t torque);
void Motor4310_ParseFeedbackMotor(Motor4310_Id_t id,
                                  const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);
void Motor4310_ProcessCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                               const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);

// MIT 模式特殊命令与纯转矩输出。
HAL_StatusTypeDef Motor4310_Enable(void);
HAL_StatusTypeDef Motor4310_Disable(void);
HAL_StatusTypeDef Motor4310_SetTorqueRaw(int16_t torque);

// 速度环与位置-速度串级环。
HAL_StatusTypeDef Motor4310_SpeedControl(int16_t target_speed);
HAL_StatusTypeDef Motor4310_PositionControl(int32_t target_position);
int32_t Motor4310_PositionToEcd(float rounds, float degree);

void Motor4310_ParseFeedback(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);
void Motor4310_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const Motor4310_Data_t *feedback; // 按 Pitch、Yaw 枚举下标观察反馈。
} Motor4310ModuleDataRefs;

typedef struct
{
    const PID_Controller_t *position_pid; // 各轴位置环状态。
    const PID_Controller_t *speed_pid; // 各轴电机速度环状态。
} Motor4310ModuleControlRefs;

typedef struct
{
    volatile Motor4310Config *config; // 当前可调驱动参数。
    Motor4310ModuleDataRefs data; // 反馈与解析数据引用。
    Motor4310ModuleControlRefs control; // 驱动闭环状态引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*online_check)(Motor4310_Id_t id); // 检查反馈在线状态。
    bool (*all_online)(void); // 检查全部电机在线状态。
    bool (*get_feedback)(Motor4310_Id_t id, Motor4310_Data_t *feedback); // 复制指定电机反馈。

    // 控制与发送。
    HAL_StatusTypeDef (*enable_motor)(Motor4310_Id_t id); // 发送指定轴使能命令。
    HAL_StatusTypeDef (*disable_motor)(Motor4310_Id_t id); // 发送指定轴失能命令。
    HAL_StatusTypeDef (*position_control_motor)(Motor4310_Id_t id, int32_t target_position); // 执行指定轴位置闭环。
    HAL_StatusTypeDef (*position_control_with_feedforward)(
        Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw); // 位置闭环叠加转矩前馈。
    HAL_StatusTypeDef (*position_control_with_profile)(
        Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw,
        const Motor4310_PidProfile_t *profile, bool enable_yaw_feedforward); // 按指定 PID 参数执行位置闭环。
    HAL_StatusTypeDef (*speed_control_motor)(Motor4310_Id_t id, int16_t target_speed); // 执行指定轴速度闭环。
    HAL_StatusTypeDef (*speed_control_with_feedforward)(
        Motor4310_Id_t id, int16_t target_speed, int16_t feedforward_raw); // 速度闭环叠加转矩前馈。
    HAL_StatusTypeDef (*set_torque_raw_motor)(Motor4310_Id_t id, int16_t torque); // 发送指定轴转矩码。
    HAL_StatusTypeDef (*enable)(void); // 发送 Yaw 使能命令。
    HAL_StatusTypeDef (*disable)(void); // 发送 Yaw 失能命令。
    HAL_StatusTypeDef (*set_torque_raw)(int16_t torque); // 发送 Yaw 转矩码。
    HAL_StatusTypeDef (*speed_control)(int16_t target_speed); // 执行速度闭环。
    HAL_StatusTypeDef (*position_control)(int32_t target_position); // 执行位置闭环。
    int32_t (*position_to_ecd)(float rounds, float degree); // 将圈数和角度换成编码器目标。

    // 状态维护。
    void (*heartbeat)(void); // 更新反馈在线状态。
    void (*reset_control)(Motor4310_Id_t id); // 清空闭环状态。

    // 接收与解析。
    void (*parse_feedback_motor)(Motor4310_Id_t id, const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]); // 解析指定轴反馈。
    void (*process_can_frame)(
        CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]); // 分发并解析 CAN 反馈。
    void (*parse_feedback)(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]); // 解析 Yaw 电机反馈。
    void (*can_rx_fifo0_callback)(CAN_HandleTypeDef *hcan); // 处理 CAN 接收中断。
} Motor4310Module;

extern const Motor4310Module motor4310; // 模块统一访问入口。

#ifdef __cplusplus
}
#endif

#endif // MOTOR4310_H
