#include "chassis_can.h"
#include "fdcan.h"
#include "motor3508.h"
#include "power_communication.h"

volatile ChassisCanState chassis_can_state;

HAL_StatusTypeDef ChassisCan_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    chassis_can_state = (ChassisCanState){0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = MOTOR3508_FEEDBACK_BASE;
    filter.FilterID2 = MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT - 1U;
    status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    if (status != HAL_OK) { return status; }

    filter.FilterIndex = 1U;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterID1 = POWER_CAPACITOR_STATUS_ID;
    filter.FilterID2 = POWER_WIRELESS_STATUS_ID;
    status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,
                                          FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_Start(&hfdcan1);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                             FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    if (status != HAL_OK) { (void)HAL_FDCAN_Stop(&hfdcan1); }
    return status;
}

HAL_StatusTypeDef ChassisCan_Send(uint32_t std_id,
                                 const uint8_t data[CHASSIS_CAN_FRAME_SIZE])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;
    uint32_t primask;

    if (data == NULL || std_id > 0x7FFU) { return HAL_ERROR; }
    header.Identifier = std_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, (uint8_t *)data);
    if (status == HAL_OK)
    {
        chassis_can_state.last_tx_id = std_id;
        chassis_can_state.tx_count++;
    }
    else
    {
        chassis_can_state.tx_error_count++;
    }
    __set_PRIMASK(primask);
    return status;
}

void ChassisCan_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    // HAL 读取消息前尚未校验 DLC，暂存区按最大 CAN FD 帧分配。
    uint8_t data[64];

    if (hfdcan != &hfdcan1 ||
        (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) { return; }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        // 超电 0x211 允许 7 字节，末字节补零供统一状态结构保存。
        data[7] = 0U;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            chassis_can_state.rx_error_count++;
            break;
        }
        if (header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN ||
            (header.DataLength != FDCAN_DLC_BYTES_8 &&
             !(header.Identifier == POWER_CAPACITOR_STATUS_ID &&
               header.DataLength == FDCAN_DLC_BYTES_7)))
        {
            chassis_can_state.rx_error_count++;
            continue;
        }

        chassis_can_state.last_rx_id = header.Identifier;
        chassis_can_state.rx_count++;
        if (header.Identifier >= MOTOR3508_FEEDBACK_BASE &&
            header.Identifier < MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT)
        {
            Motor3508_ProcessCanFrame(header.Identifier, data);
        }
        else if (header.Identifier == POWER_CAPACITOR_STATUS_ID ||
                 header.Identifier == POWER_WIRELESS_STATUS_ID)
        {
            PowerCommunication_ProcessCanFrame(header.Identifier, data);
        }
    }
}
