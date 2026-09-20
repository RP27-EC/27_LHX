#include "communication.h"
#include "can.h"
#include "motor4310.h"
#include <string.h>

#define COMM_CAN_FILTER_BANK        14U
#define COMM_CAN_SLAVE_START_BANK   14U
#define COMM_CAN_STD_ID_TO_FILTER(id) ((uint32_t)(id) << 5U)
#define COMM_CAN_RX_FRAME_COUNT     4U
#define COMM_RC_PART_D1             0x01U
#define COMM_RC_PART_D2             0x02U

static volatile Communication_CanRxFrame_t
    communication_rx_frames[COMM_CAN_RX_FRAME_COUNT];
static uint8_t communication_rc_assembly[COMM_RC_FRAME_SIZE];
static volatile uint8_t communication_rc_assembly_mask;
static uint8_t communication_rc_snapshot[COMM_RC_FRAME_SIZE];
static volatile bool communication_rc_snapshot_ready;
static volatile uint32_t communication_rc_snapshot_ms;

Communication_RcControl_t communication_rc;
volatile bool communication_rc_online = false;
volatile uint32_t communication_rc_valid_count = 0U;
volatile uint32_t communication_rc_last_valid_ms = 0U;
volatile uint32_t communication_rc_assembly_error_count = 0U;

static void Communication_RC_SetSafe(void)
{
    uint32_t saved_primask;

    saved_primask = __get_PRIMASK();
    __disable_irq();
    memset(&communication_rc, 0, sizeof(communication_rc));
    communication_rc.rc.s[0] = COMM_RC_SW_MID;
    communication_rc.rc.s[1] = COMM_RC_SW_MID;
    communication_rc_online = false;
    __set_PRIMASK(saved_primask);
}

static bool Communication_RC_D3PaddingIsZero(
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    uint32_t index;

    for (index = 2U; index < COMM_CAN_FRAME_SIZE; index++)
    {
        if (data[index] != 0U)
        {
            return false;
        }
    }

    return true;
}

/* 中断中只做分包拼接和快照提交，不在这里进行业务解析。 */
static void Communication_RC_AcceptFragment(
    uint16_t std_id,
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    if (std_id == COMM_CAN_RX_ID_D1)
    {
        memcpy(&communication_rc_assembly[0], data, COMM_CAN_FRAME_SIZE);
        communication_rc_assembly_mask = COMM_RC_PART_D1;
        return;
    }

    if (std_id == COMM_CAN_RX_ID_D2)
    {
        if (communication_rc_assembly_mask != COMM_RC_PART_D1)
        {
            communication_rc_assembly_mask = 0U;
            communication_rc_assembly_error_count++;
            return;
        }

        memcpy(&communication_rc_assembly[8], data, COMM_CAN_FRAME_SIZE);
        communication_rc_assembly_mask |= COMM_RC_PART_D2;
        return;
    }

    if (std_id == COMM_CAN_RX_ID_D3)
    {
        if ((communication_rc_assembly_mask !=
             (COMM_RC_PART_D1 | COMM_RC_PART_D2)) ||
            !Communication_RC_D3PaddingIsZero(data))
        {
            communication_rc_assembly_mask = 0U;
            communication_rc_assembly_error_count++;
            return;
        }

        communication_rc_assembly[16] = data[0];
        communication_rc_assembly[17] = data[1];
        memcpy(communication_rc_snapshot, communication_rc_assembly,
               COMM_RC_FRAME_SIZE);
        communication_rc_snapshot_ms = HAL_GetTick();
        communication_rc_snapshot_ready = true;
        communication_rc_assembly_mask = 0U;
    }

    /* D4 暂未使用，仍由通用 CAN 快照接口保留原始数据。 */
}

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
    memset(communication_rc_assembly, 0, sizeof(communication_rc_assembly));
    memset(communication_rc_snapshot, 0, sizeof(communication_rc_snapshot));
    communication_rc_assembly_mask = 0U;
    communication_rc_snapshot_ready = false;
    communication_rc_snapshot_ms = 0U;
    communication_rc_valid_count = 0U;
    communication_rc_last_valid_ms = 0U;
    communication_rc_assembly_error_count = 0U;
    Communication_RC_SetSafe();

    /* Yaw 电机与板间通信共用 CAN2，允许 Motor4310_Init 已启动总线。 */
    if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Start(&hcan2);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    else if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }

    status = HAL_CAN_ActivateNotification(&hcan2,
                                          CAN_IT_RX_FIFO0_MSG_PENDING);
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

bool Communication_RC_Parse(
    const uint8_t frame[COMM_RC_FRAME_SIZE],
    Communication_RcControl_t *control)
{
    Communication_RcControl_t decoded = {0};
    uint32_t index;

    if ((frame == NULL) || (control == NULL))
    {
        return false;
    }

    decoded.rc.ch[0] = (int16_t)(((uint16_t)frame[0] |
                                  ((uint16_t)frame[1] << 8)) & 0x07FFU) - 1024;
    decoded.rc.ch[1] = (int16_t)(((uint16_t)frame[1] >> 3 |
                                  ((uint16_t)frame[2] << 5)) & 0x07FFU) - 1024;
    decoded.rc.ch[2] = (int16_t)(((uint16_t)frame[2] >> 6 |
                                  ((uint16_t)frame[3] << 2) |
                                  ((uint16_t)frame[4] << 10)) & 0x07FFU) - 1024;
    decoded.rc.ch[3] = (int16_t)(((uint16_t)frame[4] >> 1 |
                                  ((uint16_t)frame[5] << 7)) & 0x07FFU) - 1024;
    decoded.rc.ch[4] = (int16_t)(((uint16_t)frame[16] |
                                  ((uint16_t)frame[17] << 8)) & 0x07FFU) - 1024;
    decoded.rc.s[0] = (frame[5] >> 6) & 0x03U;
    decoded.rc.s[1] = (frame[5] >> 4) & 0x03U;

    for (index = 0U; index < 4U; index++)
    {
        if ((decoded.rc.ch[index] < -660) ||
            (decoded.rc.ch[index] > 660))
        {
            memset(control, 0, sizeof(*control));
            control->rc.s[0] = COMM_RC_SW_MID;
            control->rc.s[1] = COMM_RC_SW_MID;
            return false;
        }
    }

    if ((decoded.rc.s[0] == 0U) || (decoded.rc.s[0] > COMM_RC_SW_MID) ||
        (decoded.rc.s[1] == 0U) || (decoded.rc.s[1] > COMM_RC_SW_MID))
    {
        memset(control, 0, sizeof(*control));
        control->rc.s[0] = COMM_RC_SW_MID;
        control->rc.s[1] = COMM_RC_SW_MID;
        return false;
    }

    if ((decoded.rc.ch[4] < -660) || (decoded.rc.ch[4] > 660))
    {
        decoded.rc.ch[4] = 0;
    }

    *control = decoded;
    return true;
}

void Communication_Process(void)
{
    uint8_t frame[COMM_RC_FRAME_SIZE];
    Communication_RcControl_t decoded;
    uint32_t received_ms = 0U;
    uint32_t now_ms;
    uint32_t saved_primask;
    bool frame_available;

    saved_primask = __get_PRIMASK();
    __disable_irq();
    frame_available = communication_rc_snapshot_ready;
    if (frame_available)
    {
        memcpy(frame, communication_rc_snapshot, COMM_RC_FRAME_SIZE);
        received_ms = communication_rc_snapshot_ms;
        communication_rc_snapshot_ready = false;
    }
    __set_PRIMASK(saved_primask);

    if (frame_available)
    {
        if (Communication_RC_Parse(frame, &decoded))
        {
            saved_primask = __get_PRIMASK();
            __disable_irq();
            communication_rc = decoded;
            communication_rc_last_valid_ms = received_ms;
            communication_rc_valid_count++;
            communication_rc_online = true;
            __set_PRIMASK(saved_primask);
        }
        else
        {
            saved_primask = __get_PRIMASK();
            __disable_irq();
            communication_rc = decoded;
            __set_PRIMASK(saved_primask);
        }
    }

    now_ms = HAL_GetTick();
    if ((!communication_rc_online) ||
        ((uint32_t)(now_ms - communication_rc_last_valid_ms) >=
         COMM_RC_TIMEOUT_MS))
    {
        Communication_RC_SetSafe();
    }
}

bool Communication_RC_Get(Communication_RcControl_t *control)
{
    uint32_t saved_primask;
    bool online;

    if (control == NULL)
    {
        return false;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();
    *control = communication_rc;
    online = communication_rc_online;
    __set_PRIMASK(saved_primask);

    return online;
}

bool Communication_RC_IsOnline(void)
{
    return communication_rc_online;
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

    if (hcan == NULL)
    {
        return;
    }

    if (hcan->Instance != CAN2)
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

        if ((header.StdId == MOTOR4310_FEEDBACK_CAN_ID) &&
            ((data[0] & 0x0FU) ==
             (MOTOR4310_CONTROL_CAN_ID & 0x0FU)))
        {
            Motor4310_ParseFeedback(data);
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

        Communication_RC_AcceptFragment((uint16_t)header.StdId, data);
        Communication_CAN_OnReceive((uint16_t)header.StdId, data);
    }
}
