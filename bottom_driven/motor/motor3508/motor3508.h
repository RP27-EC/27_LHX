#ifndef MOTOR3508_H
#define MOTOR3508_H

#include "peripheral_config.h"
#include "PID.h"

#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define MOTOR3508_COUNT          4U
#define MOTOR3508_FEEDBACK_BASE  0x201U
#define MOTOR3508_COMMAND_ID     0x200U

// 反馈值均来自 C620 原始报文，不包含 PID 或底盘解算。
typedef struct
{
    uint16_t encoder; // 电机转子单圈编码器：0~8191。
    int64_t encoder_total; // 相对首帧的累计转子编码器计数。
    float position_deg; // 转子累计角度，首帧为 0，未除减速比。
    int16_t speed_rpm; // 转子转速 rpm，尚未除减速比。
    int16_t current_raw; // 有符号反馈电流原始值。
    uint8_t temperature; // 温度，摄氏度。
    bool received; // 上电后是否收到过有效反馈。
    uint32_t last_rx_ms; // 最近一次反馈的 HAL 毫秒时间。
    uint32_t rx_count; // 该电机接收帧数。
} Motor3508_Feedback;

//控制总和，mode切换控制模式
HAL_StatusTypeDef Motor3508_control(uint8_t mode,int16_t id1,int16_t id2,int16_t id3,int16_t id4);

// 初始化电机反馈和 PID；底盘 CAN 接口单独初始化。
HAL_StatusTypeDef Motor3508_Init(void);

// 发送 ID1~4 电流命令并按配置限幅；HAL_OK 时电流帧已入队。
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2,
                                      int16_t id3, int16_t id4);
// 转速PID控制
HAL_StatusTypeDef Motor_3508_speed_control(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4);

// 四电机串级位控，目标为相对首帧的转子累计角度，单位度。
// 任一反馈超时则停止；调用周期须与 PID 周期一致。
HAL_StatusTypeDef Motor3508_PositionControl(float angle_1_deg,
                                           float angle_2_deg,
                                           float angle_3_deg,
                                           float angle_4_deg);

// 发送一帧四电机零电流指令
HAL_StatusTypeDef Motor3508_Stop(void);

// 电机在线检查回调函数
bool Motor3508_OnlineCheck(void);

// motor_id 为 1~4；原子复制反馈，未收到反馈或参数错误返回 false。
bool Motor3508_GetFeedback(uint8_t motor_id, Motor3508_Feedback *feedback);

// 统一底盘 CAN 接口分发已校验的 8 字节电机反馈。
void Motor3508_ProcessCanFrame(uint32_t std_id, const uint8_t data[8]);


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const Motor3508_Feedback *feedback; // 按电机 ID 顺序观察四轮反馈。
} Motor3508ModuleDataRefs;

typedef struct
{
    const PID_Controller_t *position_pid; // 四轮位置环状态。
    const PID_Controller_t *speed_pid; // 四轮速度环状态。
} Motor3508ModuleControlRefs;

typedef struct
{
    volatile Motor3508Config *config; // 当前可调驱动参数。
    Motor3508ModuleDataRefs data; // 反馈与解析数据引用。
    Motor3508ModuleControlRefs control; // 驱动闭环状态引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*online_check)(void); // 检查反馈在线状态。
    bool (*get_feedback)(uint8_t motor_id, Motor3508_Feedback *feedback); // 复制指定电机反馈。

    // 控制与发送。
    HAL_StatusTypeDef (*execute)(uint8_t mode,int16_t id1,int16_t id2,int16_t id3,int16_t id4); // 按控制模式执行四轮命令。
    HAL_StatusTypeDef (*send_current)(int16_t id1, int16_t id2, int16_t id3, int16_t id4); // 发送群组电流命令。
    HAL_StatusTypeDef (*speed_control)(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4); // 执行速度闭环。
    HAL_StatusTypeDef (*position_control)(
        float angle_1_deg, float angle_2_deg, float angle_3_deg, float angle_4_deg); // 执行位置闭环。
    HAL_StatusTypeDef (*stop)(void); // 发送停止命令。

    // 接收与解析。
    void (*process_can_frame)(uint32_t std_id, const uint8_t data[8]); // 分发并解析 CAN 反馈。
} Motor3508Module;

extern const Motor3508Module motor3508; // 模块统一访问入口。

#endif
