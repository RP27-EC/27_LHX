#include "motor3508.h"
#include "fdcan.h"
#include <string.h>
#include "PID.h"
#include <float.h>

static Motor3508_Feedback motor_feedback[MOTOR3508_COUNT]; /* 四个底盘电机反馈。 */
static PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT]; /* 四路速度环 PID。 */
static PID_Controller_t motor3508_position_pid[MOTOR3508_COUNT]; /* 四路位置外环 PID。 */
static uint8_t control_mode = 0U; /* 0 停止，1 速度，2 位置。 */

//模式切换清零积分
static void select_control_mode(uint8_t mode)
{
    uint32_t i;
    if (control_mode != mode)
    {
        for (i = 0U; i < MOTOR3508_COUNT; i++)
        {
            PID_Reset(&motor3508_speed_pid[i]);
            PID_Reset(&motor3508_position_pid[i]);
        }
        control_mode = mode;
    }
}

HAL_StatusTypeDef Motor3508_control(uint8_t mode,int16_t id1,int16_t id2,int16_t id3,int16_t id4)
{
    switch (mode) {
        case 0:
        return Motor3508_Stop();

        case 1:
        select_control_mode(mode);
        return Motor_3508_speed_control(id1,id2,id3,id4);

        case 2:
        select_control_mode(mode);
        return Motor3508_PositionControl(id1,id2,id3,id4);

        default:
        return Motor3508_Stop();
    }
}



//can以及PID参数初始化
HAL_StatusTypeDef Motor3508_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    uint32_t i;
    HAL_StatusTypeDef status;

    /* 当前 FDCAN1 专用于四个底盘电机：只接收 0x201~0x204。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = MOTOR3508_FEEDBACK_BASE;
    filter.FilterID2 = MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT - 1U;
    status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,
                                         FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) { return status; }

    memset(motor_feedback, 0, sizeof(motor_feedback));
    control_mode = 0U;
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        PID_Init(&motor3508_position_pid[i], MOTOR3508_POSITION_KP,
                 MOTOR3508_POSITION_KI, MOTOR3508_POSITION_KD,
                 MOTOR3508_POSITION_INTEGRAL_LIMIT,
                 MOTOR3508_POSITION_OUTPUT_LIMIT, MOTOR3508_PID_CONTROL_TIME_S);
    }
    status = HAL_FDCAN_Start(&hfdcan1);
    if (status != HAL_OK) { return status; }

    /* 使用已经配置的 FDCAN1 IT0；接收回调不使用 RTOS 接口。 */
    status = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    if (status != HAL_OK) { (void)HAL_FDCAN_Stop(&hfdcan1); }
        
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        PID_Init(&motor3508_speed_pid[i], MOTOR3508_SPEED_KP,
                 MOTOR3508_SPEED_KI, MOTOR3508_SPEED_KD,
                 MOTOR3508_SPEED_INTEGRAL_LIMIT,
                 MOTOR3508_SPEED_OUTPUT_LIMIT, MOTOR3508_PID_CONTROL_TIME_S);
    }
    return status;
}

//限幅
static int16_t limit_current(int16_t value)
{
    if (value > MOTOR3508_CURRENT_LIMIT) { return MOTOR3508_CURRENT_LIMIT; }
    if (value < -MOTOR3508_CURRENT_LIMIT) { return -MOTOR3508_CURRENT_LIMIT; }
    return value;
}

//4电机力矩控制
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2,int16_t id3, int16_t id4)
{
    FDCAN_TxHeaderTypeDef header = {0};
    int16_t values[4] = {id1, id2, id3, id4};
    uint8_t data[8];
    uint32_t i;

    /* C620：标准 ID 0x200，经典 CAN，四个大端有符号 16 位指令。 */
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        uint16_t raw = (uint16_t)limit_current(values[i]);
        data[2U * i] = (uint8_t)(raw >> 8);
        data[2U * i + 1U] = (uint8_t)raw;
    }
    header.Identifier = MOTOR3508_COMMAND_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    /* FIFO 满、总线未启动等情况交由调用方处理，不忙等。 */
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

//PID控速
HAL_StatusTypeDef Motor_3508_speed_control(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4){
    select_control_mode(1U);
    int16_t id1 = PID_Calc(&motor3508_speed_pid[0],speed_1,motor_feedback[0].speed_rpm);
    int16_t id2 = PID_Calc(&motor3508_speed_pid[1],speed_2,motor_feedback[1].speed_rpm);
    int16_t id3 = PID_Calc(&motor3508_speed_pid[2],speed_3,motor_feedback[2].speed_rpm);
    int16_t id4 = PID_Calc(&motor3508_speed_pid[3],speed_4,motor_feedback[3].speed_rpm);
    return Motor3508_SendCurrent(id1,id2,id3,id4);
}

//4电机强制泄力
HAL_StatusTypeDef Motor3508_Stop(void)
{
    uint32_t i;
    control_mode = 0U;
    for (i = 0U; i < MOTOR3508_COUNT; i++) { PID_Reset(&motor3508_position_pid[i]); }
    for (i = 0U; i < MOTOR3508_COUNT; i++) { PID_Reset(&motor3508_speed_pid[i]); }
    return Motor3508_SendCurrent(0, 0, 0, 0);
}

//4电机控角度
HAL_StatusTypeDef Motor3508_PositionControl(float angle_1_deg,float angle_2_deg,float angle_3_deg,float angle_4_deg)
{
    const float target_angles[MOTOR3508_COUNT] = {
        angle_1_deg, angle_2_deg, angle_3_deg, angle_4_deg
    };
    Motor3508_Feedback feedback[MOTOR3508_COUNT];
    int16_t currents[MOTOR3508_COUNT] = {0, 0, 0, 0};
    uint32_t index;
    float target_speed;
    float current;

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        if (!(target_angles[index] >= -FLT_MAX &&
              target_angles[index] <= FLT_MAX) ||
            !Motor3508_GetFeedback((uint8_t)(index + 1U), &feedback[index]))
        {
            (void)Motor3508_Stop();
            return HAL_ERROR;
        }
    }

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        target_speed = PID_Calc(&motor3508_position_pid[index], target_angles[index],
                                feedback[index].position_deg);
        current = PID_Calc(&motor3508_speed_pid[index], target_speed,
                           (float)feedback[index].speed_rpm);

        if (current > MOTOR3508_CURRENT_LIMIT) { current = MOTOR3508_CURRENT_LIMIT; }
        if (current < -MOTOR3508_CURRENT_LIMIT) { current = -MOTOR3508_CURRENT_LIMIT; }
        currents[index] = (int16_t)current;
    }

    return Motor3508_SendCurrent(currents[0], currents[1],currents[2], currents[3]);
}

//反馈报文获取
bool Motor3508_GetFeedback(uint8_t motor_id, Motor3508_Feedback *feedback)
{
    uint32_t saved_primask;
    if ((feedback == NULL) || (motor_id < 1U) || (motor_id > MOTOR3508_COUNT))
    {
        return false;
    }
    /* 避免中断更新过程中读取到不同帧的混合字段。 */
    saved_primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor_feedback[motor_id - 1U];
    __set_PRIMASK(saved_primask);
    return feedback->received;
}

bool Motor3508_OnlineCheck(void){
    Motor3508_Feedback feedback;
    uint32_t now;
    uint8_t motor_id;

    now = HAL_GetTick();

    for (motor_id = 1U; motor_id <= MOTOR3508_COUNT; motor_id++)
    {
        if (!Motor3508_GetFeedback(motor_id, &feedback))
        {
            return false;
        }
        if ((uint32_t)(now - feedback.last_rx_ms) >=
            MOTOR3508_OFFLINE_TIMEOUT_MS)
        {
            return false;
        }
    }
    return true;
}

void Motor3508_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    /* 用 64 字节暂存区，防止意外 FD 帧在校验前超出数组容量。 */
    uint8_t data[64];
    Motor3508_Feedback *motor;
    uint16_t encoder;
    int32_t delta;

    if ((hfdcan != &hfdcan1) ||
        ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) { return; }

    /* 一次中断排空已到达的报文，避免只读一帧留下积压。 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (header.DataLength != FDCAN_DLC_BYTES_8) ||
            (header.Identifier < MOTOR3508_FEEDBACK_BASE) ||
            (header.Identifier >= MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT))
        {
            continue;
        }
        encoder = ((uint16_t)data[0] << 8) | data[1];
        if (encoder > 8191U) { continue; }
        motor = &motor_feedback[header.Identifier - MOTOR3508_FEEDBACK_BASE];
        /* 编码器跨圈累计：8192 个计数为转子一圈。
         * 相邻两次有效反馈运动须小于半圈，否则方向/圈数存在歧义。
         * 首帧建立相对零点，不能提供断电保持的绝对位置。
         */
        if (motor->received)
        {
            delta = (int32_t)encoder - (int32_t)motor->encoder;
            if (delta > 4096) { delta -= 8192; }
            else if (delta < -4096) { delta += 8192; }
            motor->encoder_total += delta;
        }
        else
        {
            motor->encoder_total = 0;
        }
        motor->position_deg = (float)motor->encoder_total * (360.0f / 8192.0f);
        motor->encoder = encoder;
        motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        motor->current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        motor->temperature = data[6];
        motor->last_rx_ms = HAL_GetTick();
        motor->rx_count++;
        motor->received = true;
    }
}
