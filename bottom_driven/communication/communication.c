#include "communication.h"

#include "fdcan.h"
#include "motor3508.h"
#include <string.h>

static Communication_RxFrame communication_rx_c1;
static Communication_RxFrame communication_rx_c2;

volatile uint32_t communication_rx_count = 0U;
volatile uint32_t communication_last_rx_id = 0U;

HAL_StatusTypeDef Communication_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&communication_rx_c1, 0, sizeof(communication_rx_c1));
    memset(&communication_rx_c2, 0, sizeof(communication_rx_c2));
    communication_rx_count = 0U;
    communication_last_rx_id = 0U;

    /* 一个双 ID 标准滤波器精确放行 0xC1 和 0xC2，并送入 FIFO0。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = COMMUNICATION_RX_ID_C1;
    filter.FilterID2 = COMMUNICATION_RX_ID_C2;
    status = HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
    if (status != HAL_OK) { return status; }

    /* 未命中过滤器的标准帧、扩展帧以及所有远程帧全部丢弃。 */
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_Start(&hfdcan2);
    if (status != HAL_OK) { return status; }

    status = HAL_FDCAN_ActivateNotification(&hfdcan2,
                                            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                            0U);
    if (status != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(&hfdcan2);
    }
    return status;
}

HAL_StatusTypeDef Communication_Send(uint32_t std_id,
                                     const uint8_t data[COMMUNICATION_FRAME_SIZE])
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((data == NULL) ||
        (std_id < COMMUNICATION_TX_ID_D1) ||
        (std_id > COMMUNICATION_TX_ID_D4))
    {
        return HAL_ERROR;
    }

    header.Identifier = std_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, (uint8_t *)data);
}

bool Communication_GetRxFrame(uint32_t std_id, Communication_RxFrame *frame)
{
    const Communication_RxFrame *source;
    uint32_t saved_primask;

    if (frame == NULL) { return false; }

    if (std_id == COMMUNICATION_RX_ID_C1)
    {
        source = &communication_rx_c1;
    }
    else if (std_id == COMMUNICATION_RX_ID_C2)
    {
        source = &communication_rx_c2;
    }
    else
    {
        return false;
    }

    /* 防止中断更新到一半时，任务读到由两帧数据拼成的结构体。 */
    saved_primask = __get_PRIMASK();
    __disable_irq();
    *frame = *source;
    __set_PRIMASK(saved_primask);
    return frame->received;
}

void Communication_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[COMMUNICATION_FRAME_SIZE];
    Communication_RxFrame *destination;

    if ((hfdcan != &hfdcan2) ||
        ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    /* 一次中断排空 FIFO，避免连续到帧时遗留积压。 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        if (header.Identifier == COMMUNICATION_RX_ID_C1)
        {
            destination = &communication_rx_c1;
        }
        else if (header.Identifier == COMMUNICATION_RX_ID_C2)
        {
            destination = &communication_rx_c2;
        }
        else
        {
            continue;
        }

        memcpy(destination->data, data, COMMUNICATION_FRAME_SIZE);
        destination->last_rx_ms = HAL_GetTick();
        destination->rx_count++;
        destination->received = true;
        communication_last_rx_id = header.Identifier;
        communication_rx_count++;

        Communication_RxFrameCallback(header.Identifier, data);
    }
}

__weak void Communication_RxFrameCallback(
    uint32_t std_id, const uint8_t data[COMMUNICATION_FRAME_SIZE])
{
    (void)std_id;
    (void)data;
}

/* HAL 只允许存在一个同名 FIFO0 回调，因此在这里统一分发两个 FDCAN 实例。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    if (hfdcan == &hfdcan1)
    {
        Motor3508_FDCANRxFifo0Callback(hfdcan, interrupts);
    }
    else if (hfdcan == &hfdcan2)
    {
        Communication_FDCANRxFifo0Callback(hfdcan, interrupts);
    }
}
