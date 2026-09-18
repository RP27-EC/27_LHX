#include "motor3508.h"
#include "fdcan.h"
#include <string.h>
#include "PID.h"

/* 仅复用模板 RM_motor/drv_can 的 C620 报文逻辑，不引入其设备对象。 */
static Motor3508_Feedback motor_feedback[MOTOR3508_COUNT];
PID_Controller_t motor3508_1_pid;
PID_Controller_t motor3508_2_pid;
PID_Controller_t motor3508_3_pid;
PID_Controller_t motor3508_4_pid;

float kp=8,ki=2;

HAL_StatusTypeDef Motor3508_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
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
    status = HAL_FDCAN_Start(&hfdcan1);
    if (status != HAL_OK) { return status; }

    /* 使用已经配置的 FDCAN1 IT0；接收回调不使用 RTOS 接口。 */
    status = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    if (status != HAL_OK) { (void)HAL_FDCAN_Stop(&hfdcan1); }
    PID_Init(&motor3508_1_pid,kp,ki,0.0f,1000.0f,10000.0f,0.001f);
    PID_Init(&motor3508_2_pid,kp,ki,0.0f,1000.0f,10000.0f,0.001f);
    PID_Init(&motor3508_3_pid,kp,ki,0.0f,1000.0f,10000.0f,0.001f);
    PID_Init(&motor3508_4_pid,kp,ki,0.0f,1000.0f,10000.0f,0.001f);
    return status;
}

static int16_t limit_current(int16_t value)
{
    if (value > MOTOR3508_CURRENT_LIMIT) { return MOTOR3508_CURRENT_LIMIT; }
    if (value < -MOTOR3508_CURRENT_LIMIT) { return -MOTOR3508_CURRENT_LIMIT; }
    return value;
}

HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2,
                                      int16_t id3, int16_t id4)
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

    HAL_StatusTypeDef Motor_3508_speed_control(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4){
    int16_t id1 = PID_Calc(&motor3508_1_pid,speed_1,motor_feedback[0].speed_rpm);
    int16_t id2 = PID_Calc(&motor3508_2_pid,speed_2,motor_feedback[1].speed_rpm);
    int16_t id3 = PID_Calc(&motor3508_3_pid,speed_3,motor_feedback[2].speed_rpm);
    int16_t id4 = PID_Calc(&motor3508_4_pid,speed_4*10,motor_feedback[3].speed_rpm);
    return Motor3508_SendCurrent(id1,id2,id3,id4);
}

HAL_StatusTypeDef Motor3508_Stop(void)
{
    PID_Reset(&motor3508_1_pid);
    PID_Reset(&motor3508_2_pid);
    PID_Reset(&motor3508_3_pid);
    PID_Reset(&motor3508_4_pid);
    return Motor3508_SendCurrent(0, 0, 0, 0);
}

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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    /* 用 64 字节暂存区，防止意外 FD 帧在校验前超出数组容量。 */
    uint8_t data[64];
    Motor3508_Feedback *motor;
    uint16_t encoder;

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
        motor->encoder = encoder;
        motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        motor->current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        motor->temperature = data[6];
        motor->last_rx_ms = HAL_GetTick();
        motor->rx_count++;
        motor->received = true;
    }
}
