#ifndef DIAL_MOTOR_H
#define DIAL_MOTOR_H

#include "peripheral_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

#define DIAL_MOTOR_FRAME_SIZE  8U
#define DIAL_MOTOR_CAN_ID      0x141U
#define DIAL_MOTOR_ENCODER_COUNTS_PER_REV  65536.0f

typedef enum
{
    DIAL_MOTOR_CMD_CLOSE = 0x80U, // 关闭电机命令。
    DIAL_MOTOR_CMD_STOP = 0x81U, // 停止电机命令。
    DIAL_MOTOR_CMD_RUN = 0x88U, // 运行电机命令。
    DIAL_MOTOR_CMD_TORQUE = 0xA1U // 转矩电流闭环命令。
} DialMotor_Command_t;

// LK4005 基础反馈；速度单位 1 deg/s，累计角度以首帧为零点。
typedef struct
{
    uint8_t response_command; // 最近反馈帧对应的命令字。
    int8_t temperature; // 电机温度，单位摄氏度。
    int16_t current_raw; // 反馈的转矩电流原始值。
    int16_t speed_dps; // 电机轴速度，单位度每秒。
    uint16_t encoder; // 当前单圈编码器值。
    uint16_t last_encoder; // 上一帧编码器值，用于跨圈累计。
    int64_t encoder_total; // 相对上电首帧的累计编码器计数。
    float position_deg; // 累计计数换算后的电机轴角度。
    volatile bool initialized; // 是否已经用首帧建立累计零点。
    volatile bool received; // 上电后是否至少收到过一帧反馈。
    volatile bool online; // 心跳检测得到的当前在线状态。
    volatile uint32_t last_rx_ms; // 最近一次反馈的毫秒时间戳。
    volatile uint32_t rx_count; // 累计接收的有效反馈帧数。
} DialMotor_Feedback_t;

typedef struct
{
    uint32_t queued_count; // 成功放入 CAN1 发送邮箱的帧数。
    uint32_t guard_busy_count; // 收发间隔未满足而跳过的次数。
    uint32_t mailbox_busy_count; // CAN1 发送邮箱全满的次数。
    uint32_t send_error_count; // HAL 发送失败次数。
    uint8_t last_command; // 最近成功入队的命令字。
    int16_t last_current_raw; // 最近成功入队的 A1 电流命令。
} DialMotor_TxDiagnostics_t;

extern DialMotor_Feedback_t dial_motor_feedback;
extern volatile DialMotor_TxDiagnostics_t dial_motor_tx_diagnostics;
extern PID_Controller_t dial_motor_position_pid;
extern PID_Controller_t dial_motor_speed_pid;
extern PID_Controller_t dial_motor_continuous_speed_pid; // 连发独立速度环。

// 配置 CAN1 的 0x141 精确过滤器。
HAL_StatusTypeDef DialMotor_Init(void);

// 基础状态命令。
HAL_StatusTypeDef DialMotor_Run(void);
HAL_StatusTypeDef DialMotor_Stop(void);
HAL_StatusTypeDef DialMotor_Close(void);

// 0xA1 转矩电流闭环，指令会按 dial_motor_config.current_limit 限幅。
HAL_StatusTypeDef DialMotor_SetTorqueCurrent(int16_t current);

// 累计编码器位控；以上电首帧为零点，每圈 65536 计数，随发射任务调用。
HAL_StatusTypeDef DialMotor_PositionControl(int64_t target_encoder_total);
// 位置闭环附加速度上限，用于热量临界时完成已预留的一发。
HAL_StatusTypeDef DialMotor_PositionControlLimited(int64_t target_encoder_total,
                                                  float speed_limit_dps);
// 拨盘速度闭环；目标和反馈均为电机轴度每秒，随发射任务调用。
HAL_StatusTypeDef DialMotor_SpeedControl(float target_speed_dps);
void DialMotor_ResetControl(void);

bool DialMotor_GetFeedback(DialMotor_Feedback_t *feedback);
bool DialMotor_OnlineCheck(void);
void DialMotor_Heartbeat(void);

// 由工程统一 HAL CAN 回调分发，本函数不主动读 FIFO。
void DialMotor_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[DIAL_MOTOR_FRAME_SIZE]);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const DialMotor_Feedback_t *feedback; // 拨盘反馈与累计位置。
    const volatile DialMotor_TxDiagnostics_t *tx; // 拨盘命令发送统计。
} DialMotorModuleDataRefs;

typedef struct
{
    const PID_Controller_t *position_pid; // 拨盘位置环状态。
    const PID_Controller_t *speed_pid; // 单发速度环状态。
    const PID_Controller_t *continuous_speed_pid; // 连发速度环状态。
} DialMotorModuleControlRefs;

typedef struct
{
    volatile DialMotorConfig *config; // 当前可调驱动参数。
    DialMotorModuleDataRefs data; // 反馈与解析数据引用。
    DialMotorModuleControlRefs control; // 驱动闭环状态引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get_feedback)(DialMotor_Feedback_t *feedback); // 复制指定电机反馈。
    bool (*online_check)(void); // 检查反馈在线状态。

    // 控制与发送。
    HAL_StatusTypeDef (*run)(void); // 发送拨盘运行命令。
    HAL_StatusTypeDef (*stop)(void); // 发送停止命令。
    HAL_StatusTypeDef (*close)(void); // 发送拨盘关闭命令。
    HAL_StatusTypeDef (*set_torque_current)(int16_t current); // 发送拨盘转矩电流命令。
    HAL_StatusTypeDef (*position_control)(int64_t target_encoder_total); // 执行位置闭环。
    HAL_StatusTypeDef (*position_control_limited)(int64_t target_encoder_total, float speed_limit_dps); // 按附加速度上限执行拨盘位控。
    HAL_StatusTypeDef (*speed_control)(float target_speed_dps); // 执行速度闭环。

    // 状态维护。
    void (*reset_control)(void); // 清空闭环状态。
    void (*heartbeat)(void); // 更新反馈在线状态。

    // 接收与解析。
    void (*process_can_frame)(
        CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t data[DIAL_MOTOR_FRAME_SIZE]); // 分发并解析 CAN 反馈。
} DialMotorModule;

extern const DialMotorModule dial_motor; // 模块统一访问入口。

#ifdef __cplusplus
}
#endif

#endif // DIAL_MOTOR_H
