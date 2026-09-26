#include "communication.h"
#include "can.h"
#include "motor4310.h"
#include "motor3508.h"
#include "motor2006.h"
#include "dial_motor.h"
#include <float.h>
#include <string.h>

#define COMM_CAN_FILTER_BANK        14U
#define COMM_CAN_WHEEL_FILTER_BANK  16U
#define COMM_CAN_SLAVE_START_BANK   14U
#define COMM_CAN_STD_ID_TO_FILTER(id) ((uint32_t)(id) << 5U)
#define COMM_CAN_RX_FRAME_COUNT     5U
#define COMM_RC_PART_D1             0x01U
#define COMM_RC_PART_D2             0x02U

static volatile Communication_CanRxFrame_t
    communication_rx_frames[COMM_CAN_RX_FRAME_COUNT]; /* D1~D5 各自的最新接收快照。 */
static uint8_t communication_rc_assembly[COMM_RC_FRAME_SIZE]; /* D1~D3 拼接中的遥控原始帧。 */
static volatile uint8_t communication_rc_assembly_mask; /* 已收到 D1/D2 分片的位掩码。 */
static uint8_t communication_rc_snapshot[COMM_RC_FRAME_SIZE]; /* 提交给任务解析的完整遥控快照。 */
static volatile bool communication_rc_snapshot_ready; /* 是否有完整遥控帧等待任务解析。 */
static volatile uint32_t communication_rc_snapshot_ms; /* 完整遥控帧拼接完成的时间。 */

Communication_RcControl_t communication_rc; /* 当前已解析的遥控器数据。 */
volatile bool communication_rc_online = false; /* 遥控链路当前是否在线。 */
volatile uint32_t communication_rc_valid_count = 0U; /* 累计有效遥控帧数。 */
volatile uint32_t communication_rc_last_valid_ms = 0U; /* 最近有效遥控帧时间。 */
volatile uint32_t communication_rc_assembly_error_count = 0U; /* D1~D3 拼帧错误数。 */

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

    /* D4 底盘角速度、D5 四轮转速由通用 CAN 快照保存。 */
}

static int32_t Communication_CAN_RxIndex(uint16_t std_id)
{
    if ((std_id >= COMM_CAN_RX_ID_D1) && (std_id <= COMM_CAN_RX_ID_D5))
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
     * 0xD1、0xD2、0xD3、0xD4；D5 由 bank 16 单独接收。
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

    /* bank 15 留给 Yaw 电机；bank 16 单独精确接收 D5 四轮转速。 */
    filter.FilterBank = COMM_CAN_WHEEL_FILTER_BANK;
    filter.FilterIdHigh = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D5);
    filter.FilterIdLow = filter.FilterIdHigh;
    filter.FilterMaskIdHigh = filter.FilterIdHigh;
    filter.FilterMaskIdLow = filter.FilterIdHigh;
    status = HAL_CAN_ConfigFilter(&hcan2, &filter);
    if (status != HAL_OK) { return status; }

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
    uint32_t primask;
    HAL_StatusTypeDef status;

    if ((data == NULL) ||
        ((std_id != COMM_CAN_TX_ID_C1) &&
         (std_id != COMM_CAN_TX_ID_C2)))
    {
        return HAL_ERROR;
    }

    header.StdId = std_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = COMM_CAN_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;

    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U ?
        HAL_BUSY : HAL_CAN_AddTxMessage(&hcan2, &header,
                                       (uint8_t *)data, &mailbox);
    __set_PRIMASK(primask);
    return status;
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

HAL_StatusTypeDef Communication_CAN_SendLiftLock(bool hold, uint8_t sequence)
{
    uint8_t data[COMM_CAN_FRAME_SIZE] = {0};
    data[0] = COMM_LIFT_LOCK_MAGIC;
    data[1] = hold ? 1U : 0U;
    data[2] = sequence;
    return Communication_CAN_SendC2(data);
}

bool Communication_CAN_ChassisWheelsStopped(uint32_t request_start_ms)
{
    Communication_CanRxFrame_t frame;
    uint32_t index;
    int16_t speed;

    if (!Communication_CAN_GetLatest(COMM_CAN_RX_ID_D5, &frame) ||
        (uint32_t)(HAL_GetTick() - frame.last_rx_ms) >=
            LIFT_CHASSIS_SPEED_TIMEOUT_MS ||
        (int32_t)(frame.last_rx_ms - request_start_ms) < 0)
    { return false; }
    for (index = 0U; index < 4U; index++)
    {
        speed = (int16_t)((uint16_t)frame.data[index * 2U] |
                          ((uint16_t)frame.data[index * 2U + 1U] << 8U));
        if (speed > LIFT_CHASSIS_STOP_SPEED_RPM ||
            speed < -LIFT_CHASSIS_STOP_SPEED_RPM)
        { return false; }
    }
    return true;
}

HAL_StatusTypeDef Communication_CAN_SendYawAngle(float angle_deg)
{
    return Communication_CAN_SendYawState(angle_deg, false);
}

HAL_StatusTypeDef Communication_CAN_SendYawState(float angle_deg, bool turning)
{
    uint8_t data[COMM_CAN_FRAME_SIZE] = {0};
    int16_t encoded;

    if (!(angle_deg >= -FLT_MAX && angle_deg <= FLT_MAX))
    { return HAL_ERROR; }
    if (angle_deg > 327.67f) { angle_deg = 327.67f; }
    else if (angle_deg < -327.68f) { angle_deg = -327.68f; }
    encoded = (int16_t)(angle_deg * 100.0f);
    data[0] = (uint8_t)(uint16_t)encoded;
    data[1] = (uint8_t)((uint16_t)encoded >> 8);
    data[2] = turning ? 0x03U : 0x01U;
    return Communication_CAN_SendC1(data);
}

bool Communication_CAN_GetChassisYawRate(float *rate_deg_s)
{
    Communication_CanRxFrame_t frame;
    int16_t encoded;

    if (rate_deg_s == NULL ||
        !Communication_CAN_GetLatest(COMM_CAN_RX_ID_D4, &frame) ||
        (uint32_t)(HAL_GetTick() - frame.last_rx_ms) >=
            CLOUD_FOLLOW_RATE_TIMEOUT_MS ||
        (frame.data[2] & 0x01U) == 0U)
    {
        return false;
    }
    encoded = (int16_t)((uint16_t)frame.data[0] |
                        ((uint16_t)frame.data[1] << 8));
    *rate_deg_s = (float)encoded * 0.01f;
    return true;
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
    decoded.mouse.x = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    decoded.mouse.y = (int16_t)((uint16_t)frame[8] | ((uint16_t)frame[9] << 8));
    decoded.mouse.z = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    decoded.mouse.left = (frame[12] & 0x01U) != 0U;
    decoded.mouse.right = (frame[13] & 0x01U) != 0U;
    decoded.key = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8);

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
    online = communication_rc_online &&
             (uint32_t)(HAL_GetTick() - communication_rc_last_valid_ms) <
             COMM_RC_TIMEOUT_MS;
    __set_PRIMASK(saved_primask);

    return online;
}

bool Communication_RC_IsOnline(void)
{
    return communication_rc_online &&
           (uint32_t)(HAL_GetTick() - communication_rc_last_valid_ms) <
           COMM_RC_TIMEOUT_MS;
}

__weak void Communication_CAN_OnReceive(
    uint16_t std_id,
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    (void)std_id;
    (void)data;
}

/* CAN1/2 RX0 IRQ 均由 CubeMX 生成，在此统一分发。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[COMM_CAN_FRAME_SIZE];
    int32_t index;

    if (hcan == NULL)
    {
        return;
    }

    if (hcan->Instance != CAN1 && hcan->Instance != CAN2)
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

        if (hcan->Instance == CAN1)
        {
            /* CAN1 共用：Pitch 4310、两路 3508、M2006 和 LK4005。 */
            Motor4310_ProcessCanFrame(hcan, header.StdId, data);
            Motor3508_ProcessCanFrame(hcan, header.StdId, data);
            Motor2006_ProcessCanFrame(hcan, header.StdId, data);
            DialMotor_ProcessCanFrame(hcan, header.StdId, data);
            continue;
        }

        if (header.StdId == MOTOR4310_FEEDBACK_CAN_ID)
        {
            Motor4310_ProcessCanFrame(hcan, header.StdId, data);
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
