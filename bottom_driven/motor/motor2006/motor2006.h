#ifndef MOTOR2006_H
#define MOTOR2006_H

#include "peripheral_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

// M2006：CAN1 0x204 回传，0x200 第 4 槽发送。
#define MOTOR2006_FEEDBACK_CAN_ID  0x204U
#define MOTOR2006_COMMAND_CAN_ID   0x200U
#define MOTOR2006_FRAME_SIZE       8U
#define MOTOR2006_REDUCTION_RATIO  36.0f

typedef struct
{
    uint16_t encoder; // 电机转子单圈编码器，0~8191。
    uint16_t last_encoder; // 上一帧编码器，用于跨圈累计。
    int32_t encoder_total; // 以上电首帧为零点的转子累计计数。
    int16_t speed_rpm; // 电机转子速度，rpm。
    int16_t current_raw; // C610 回传电流原始码。
    uint8_t temperature; // 电机温度，摄氏度。
    volatile bool received; // 是否收到过有效反馈。
    volatile bool online; // 反馈是否在配置的在线超时范围内。
    volatile uint32_t last_rx_ms; // 最近反馈时间。
    volatile uint32_t rx_count; // 有效反馈累计帧数。
} Motor2006_Feedback_t;

extern Motor2006_Feedback_t motor2006_feedback;
extern PID_Controller_t motor2006_speed_pid;

// 配置 CAN1 精确过滤器；不会主动给电机非零电流。
HAL_StatusTypeDef Motor2006_Init(void);
// 与摩擦轮共用 0x200 帧；非零命令超时自动归零。
HAL_StatusTypeDef Motor2006_SetCurrent(int16_t current_raw);
HAL_StatusTypeDef Motor2006_Stop(void);
// 速度目标为转子 rad/s，调用周期与速度 PID 配置一致。
HAL_StatusTypeDef Motor2006_SpeedControl(float target_rotor_rad_s);
void Motor2006_ResetSpeedPID(void);

bool Motor2006_GetFeedback(Motor2006_Feedback_t *feedback);
bool Motor2006_OnlineCheck(void);
void Motor2006_Heartbeat(void);
float Motor2006_GetOutputAngleDeg(void); // 36:1 减速箱输出轴相对上电角度。

// 0x200 群组帧由本驱动统一拼接：第 1/2 槽摩擦轮，第 3 槽零，第 4 槽 2006。
HAL_StatusTypeDef Motor2006_SendFrictionCurrents(int16_t left_raw,
                                                  int16_t right_raw);
// 由统一 CAN 接收回调分发，不主动读取 FIFO。
void Motor2006_ProcessCanFrame(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               const uint8_t data[MOTOR2006_FRAME_SIZE]);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const Motor2006_Feedback_t *feedback; // 升降电机反馈与累计转子位置。
} Motor2006ModuleDataRefs;

typedef struct
{
    const PID_Controller_t *speed_pid; // 升降速度环状态。
} Motor2006ModuleControlRefs;

typedef struct
{
    volatile Motor2006Config *config; // 当前可调驱动参数。
    Motor2006ModuleDataRefs data; // 反馈与解析数据引用。
    Motor2006ModuleControlRefs control; // 驱动闭环状态引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get_feedback)(Motor2006_Feedback_t *feedback); // 复制指定电机反馈。
    bool (*online_check)(void); // 检查反馈在线状态。
    float (*get_output_angle_deg)(void); // 读取升降输出轴相对角度。

    // 控制与发送。
    HAL_StatusTypeDef (*set_current)(int16_t current_raw); // 设置升降电流命令。
    HAL_StatusTypeDef (*stop)(void); // 发送停止命令。
    HAL_StatusTypeDef (*speed_control)(float target_rotor_rad_s); // 执行速度闭环。
    HAL_StatusTypeDef (*send_friction_currents)(int16_t left_raw, int16_t right_raw); // 拼接摩擦轮与升降的群组电流帧。

    // 状态维护。
    void (*reset_speed_pid)(void); // 清空速度环状态。
    void (*heartbeat)(void); // 更新反馈在线状态。

    // 接收与解析。
    void (*process_can_frame)(
        CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t data[MOTOR2006_FRAME_SIZE]); // 分发并解析 CAN 反馈。
} Motor2006Module;

extern const Motor2006Module motor2006; // 模块统一访问入口。

#ifdef __cplusplus
}
#endif

#endif // MOTOR2006_H
