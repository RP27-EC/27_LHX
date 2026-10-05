#ifndef MOTOR3508_H
#define MOTOR3508_H

#include "peripheral_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR3508_COUNT          2U
#define MOTOR3508_FEEDBACK_BASE  0x201U
#define MOTOR3508_COMMAND_ID     0x200U
#define MOTOR3508_FRAME_SIZE     8U

// C620 反馈原始数据；速度单位为电机转子 rpm。
typedef struct
{
    uint16_t encoder; // 转子单圈编码器原始值，范围 0~8191。
    int16_t speed_rpm; // 转子反馈转速，单位 rpm。
    int16_t current_raw; // C620 反馈的实际电流原始值。
    uint8_t temperature; // 电机温度，单位摄氏度。
    volatile bool received; // 上电后是否至少收到过一帧反馈。
    volatile bool online; // 心跳检测得到的当前在线状态。
    volatile uint32_t last_rx_ms; // 最近一次反馈的毫秒时间戳。
    volatile uint32_t rx_count; // 累计接收的有效反馈帧数。
} Motor3508_Feedback_t;

extern Motor3508_Feedback_t motor3508_feedback[MOTOR3508_COUNT];
extern PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT];

// 配置两路摩擦轮反馈过滤器，并初始化两路速度 PID。
HAL_StatusTypeDef Motor3508_Init(void);

// 发送 0x200 群组电流帧；ID1/ID2 为摩擦轮，ID3 为零，ID4 保留 M2006 电流。
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t current_1,
                                       int16_t current_2);

// 双摩擦轮等速反向控制；目标限速，调用周期与 PID 周期一致。
HAL_StatusTypeDef Motor3508_SpeedControl(int16_t target_speed_rpm);
HAL_StatusTypeDef Motor3508_Stop(void);
void Motor3508_ResetSpeedPID(void);

// motor_id 范围为 1~2。
bool Motor3508_GetFeedback(uint8_t motor_id,
                           Motor3508_Feedback_t *feedback);
bool Motor3508_OnlineCheck(uint8_t motor_id);
bool Motor3508_AllOnline(void);
void Motor3508_Heartbeat(void);

// 由工程统一 HAL CAN 回调分发，本函数不主动读 FIFO。
void Motor3508_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[MOTOR3508_FRAME_SIZE]);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const Motor3508_Feedback_t *feedback; // 按电机 ID 顺序观察摩擦轮反馈。
} Motor3508ModuleDataRefs;

typedef struct
{
    const PID_Controller_t *speed_pid; // 两路摩擦轮速度环状态。
} Motor3508ModuleControlRefs;

typedef struct
{
    volatile Motor3508Config *config; // 当前可调驱动参数。
    Motor3508ModuleDataRefs data; // 反馈与解析数据引用。
    Motor3508ModuleControlRefs control; // 驱动闭环状态引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get_feedback)(uint8_t motor_id, Motor3508_Feedback_t *feedback); // 复制指定电机反馈。
    bool (*online_check)(uint8_t motor_id); // 检查反馈在线状态。
    bool (*all_online)(void); // 检查全部电机在线状态。

    // 控制与发送。
    HAL_StatusTypeDef (*send_current)(int16_t current_1, int16_t current_2); // 发送群组电流命令。
    HAL_StatusTypeDef (*speed_control)(int16_t target_speed_rpm); // 执行速度闭环。
    HAL_StatusTypeDef (*stop)(void); // 发送停止命令。

    // 状态维护。
    void (*reset_speed_pid)(void); // 清空速度环状态。
    void (*heartbeat)(void); // 更新反馈在线状态。

    // 接收与解析。
    void (*process_can_frame)(
        CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t data[MOTOR3508_FRAME_SIZE]); // 分发并解析 CAN 反馈。
} Motor3508Module;

extern const Motor3508Module motor3508; // 模块统一访问入口。

#ifdef __cplusplus
}
#endif

#endif // MOTOR3508_H
