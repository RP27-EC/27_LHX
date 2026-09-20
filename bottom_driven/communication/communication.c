#include "communication.h"
#include "can.h"
#include <string.h>

#define COMM_CAN_FILTER_BANK        14U
#define COMM_CAN_SLAVE_START_BANK   14U
#define COMM_CAN_STD_ID_TO_FILTER(id) ((uint32_t)(id) << 5U)
#define COMM_CAN_RX_FRAME_COUNT     4U

static volatile Communication_CanRxFrame_t
    communication_rx_frames[COMM_CAN_RX_FRAME_COUNT];

static int32_t Communication_CAN_RxIndex(uint16_t std_id)
{
    if ((std_id >= COMM_CAN_RX_ID_D1) && (std_id <= COMM_CAN_RX_ID_D4))
    {
        return (int32_t)(std_id - COMM_CAN_RX_ID_D1);
    }

    return -1;
}

HAL_StatusTypeDef Communication_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    /*
     * bxCAN 16 位列表模式一组正好容纳 4 个标准 ID，因而只接收
     * 0xD1、0xD2、0xD3、0xD4，不会额外放行相邻 ID。
     * CAN2 使用从过滤器组 14 开始的共享过滤器区域。
     */
    filter.FilterBank = COMM_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D1);
    filter.FilterIdLow = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D2);
    filter.FilterMaskIdHigh = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D3);
    filter.FilterMaskIdLow = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D4);
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = COMM_CAN_SLAVE_START_BANK;

    status = HAL_CAN_ConfigFilter(&hcan2, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

    memset((void *)communication_rx_frames, 0,
           sizeof(communication_rx_frames));

    status = HAL_CAN_Start(&hcan2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_CAN_ActivateNotification(&hcan2,
                                          CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK)
    {
        (void)HAL_CAN_Stop(&hcan2);
    }

    return status;
}

HAL_StatusTypeDef Communication_CAN_Send(
    uint16_t std_id,
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if ((data == NULL) ||
        ((std_id != COMM_CAN_TX_ID_C1) &&
         (std_id != COMM_CAN_TX_ID_C2)))
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U)
    {
        return HAL_BUSY;
    }

    header.StdId = std_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = COMM_CAN_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan2, &header, (uint8_t *)data, &mailbox);
}

HAL_StatusTypeDef Communication_CAN_SendC1(
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    return Communication_CAN_Send(COMM_CAN_TX_ID_C1, data);
}

HAL_StatusTypeDef Communication_CAN_SendC2(
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    return Communication_CAN_Send(COMM_CAN_TX_ID_C2, data);
}

bool Communication_CAN_GetLatest(uint16_t std_id,
                                 Communication_CanRxFrame_t *frame)
{
    int32_t index;
    uint32_t saved_primask;

    if (frame == NULL)
    {
        return false;
    }

    index = Communication_CAN_RxIndex(std_id);
    if (index < 0)
    {
        return false;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();
    memcpy(frame, (const void *)&communication_rx_frames[index],
           sizeof(*frame));
    __set_PRIMASK(saved_primask);

    return frame->received;
}

__weak void Communication_CAN_OnReceive(
    uint16_t std_id,
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    (void)std_id;
    (void)data;
}

/* CAN2_RX0_IRQHandler 已由 CubeMX 生成，并会进入此 HAL 回调。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[COMM_CAN_FRAME_SIZE];
    int32_t index;

    if ((hcan == NULL) || (hcan->Instance != CAN2))
    {
        return;
    }

    /* 一次中断排空 FIFO0，避免高频报文在队列中积压。 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        if ((header.IDE != CAN_ID_STD) ||
            (header.RTR != CAN_RTR_DATA) ||
            (header.DLC != COMM_CAN_FRAME_SIZE))
        {
            continue;
        }

        index = Communication_CAN_RxIndex((uint16_t)header.StdId);
        if (index < 0)
        {
            continue;
        }

        memcpy((void *)communication_rx_frames[index].data,
               data, COMM_CAN_FRAME_SIZE);
        communication_rx_frames[index].last_rx_ms = HAL_GetTick();
        communication_rx_frames[index].rx_count++;
        communication_rx_frames[index].received = true;

        Communication_CAN_OnReceive((uint16_t)header.StdId, data);
    }
}
