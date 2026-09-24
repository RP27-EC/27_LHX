#include "communication.h"

#include "fdcan.h"
#include "motor3508.h"
#include "parameter.h"
#include <float.h>
#include <string.h>

static Communication_RxFrame communication_rx_c1; /* 上板 C1 报文的最新快照。 */
static Communication_RxFrame communication_rx_c2; /* 上板 C2 报文的最新快照。 */

volatile uint32_t communication_rx_count = 0U; /* C1/C2 有效报文累计接收数。 */
volatile uint32_t communication_last_rx_id = 0U; /* 最近收到的板间通信标准 ID。 */
volatile uint32_t communication_bus_off_count = 0U; /* CAN2 Bus-Off 恢复尝试次数。 */
volatile uint32_t communication_restart_count = 0U; /* CAN2 成功重新启动次数。 */
static uint32_t communication_last_restart_ms; /* 避免持续断线时频繁重启外设。 */

HAL_StatusTypeDef Communication_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&communication_rx_c1, 0, sizeof(communication_rx_c1));
    memset(&communication_rx_c2, 0, sizeof(communication_rx_c2));
    communication_rx_count = 0U;
    communication_last_rx_id = 0U;
    communication_bus_off_count = 0U;
    communication_restart_count = 0U;
    communication_last_restart_ms = 0U;

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

void Communication_Service(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;
    uint32_t now_ms;

    /* 遥控帧超时会自动恢复；此处处理的是控制器自身进入 Bus-Off。 */
    if (HAL_FDCAN_GetState(&hfdcan2) != HAL_FDCAN_STATE_BUSY ||
        HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol_status) != HAL_OK ||
        protocol_status.BusOff == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if (communication_bus_off_count != 0U &&
        (uint32_t)(now_ms - communication_last_restart_ms) <
            COMMUNICATION_BUS_OFF_RETRY_MS)
    {
        return;
    }

    communication_last_restart_ms = now_ms;
    communication_bus_off_count++;
    /* HAL Start 只接受 READY 状态；先 Stop 再 Start 清除 Bus-Off 的 INIT。 */
    if (HAL_FDCAN_Stop(&hfdcan2) != HAL_OK)
    {
        return;
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        return;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan2,
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                        0U) == HAL_OK)
    {
        communication_restart_count++;
    }
}

HAL_StatusTypeDef Communication_Send(uint32_t std_id,
                                     const uint8_t data[COMMUNICATION_FRAME_SIZE])
{
    FDCAN_TxHeaderTypeDef header = {0};
    FDCAN_ProtocolStatusTypeDef protocol_status;

    if ((data == NULL) ||
        (std_id < COMMUNICATION_TX_ID_D1) ||
        (std_id > COMMUNICATION_TX_ID_D4))
    {
        return HAL_ERROR;
    }

    /* Bus-Off 时不继续塞入旧遥控帧；恢复任务重启后发送当前最新帧。 */
    if (HAL_FDCAN_GetState(&hfdcan2) != HAL_FDCAN_STATE_BUSY ||
        HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol_status) != HAL_OK ||
        protocol_status.BusOff != 0U)
    {
        return HAL_BUSY;
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

bool Communication_GetYawAngle(float *angle_deg)
{
    Communication_RxFrame frame;
    int16_t encoded;

    if (angle_deg == NULL ||
        !Communication_GetRxFrame(COMMUNICATION_RX_ID_C1, &frame) ||
        (uint32_t)(HAL_GetTick() - frame.last_rx_ms) >=
            CHASSIS_FOLLOW_ANGLE_TIMEOUT_MS ||
        (frame.data[2] & 0x01U) == 0U)
    {
        return false;
    }
    encoded = (int16_t)((uint16_t)frame.data[0] |
                        ((uint16_t)frame.data[1] << 8));
    *angle_deg = (float)encoded * 0.01f;
    return true;
}

HAL_StatusTypeDef Communication_SendChassisYawRate(float rate_deg_s)
{
    uint8_t data[COMMUNICATION_FRAME_SIZE] = {0};
    int16_t encoded;

    if (!(rate_deg_s >= -FLT_MAX && rate_deg_s <= FLT_MAX))
    { return HAL_ERROR; }
    if (rate_deg_s > 327.67f) { rate_deg_s = 327.67f; }
    else if (rate_deg_s < -327.68f) { rate_deg_s = -327.68f; }
    encoded = (int16_t)(rate_deg_s * 100.0f);
    data[0] = (uint8_t)(uint16_t)encoded;
    data[1] = (uint8_t)((uint16_t)encoded >> 8);
    data[2] = 0x01U;
    return Communication_Send(COMMUNICATION_TX_ID_D4, data);
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
