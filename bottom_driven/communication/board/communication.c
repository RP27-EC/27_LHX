#include "communication.h"
#include "can.h"
#include "motor4310.h"
#include "motor3508.h"
#include "motor2006.h"
#include "dial_motor.h"
#include "peripheral_config.h"
#include <float.h>
#include <string.h>

#define COMM_CAN_FILTER_BANK        14U
#define COMM_CAN_WHEEL_FILTER_BANK  16U
#define COMM_CAN_SLAVE_START_BANK   14U
#define COMM_CAN_STD_ID_TO_FILTER(id) ((uint32_t)(id) << 5U)
#define COMM_CAN_RX_FRAME_COUNT     7U
#define COMM_RC_PART_D1             0x01U
#define COMM_RC_PART_D2             0x02U

static volatile Communication_CanRxFrame_t
    communication_rx_frames[COMM_CAN_RX_FRAME_COUNT]; // D1~D7 各自的最新接收快照。
static uint8_t communication_rc_assembly[COMM_RC_FRAME_SIZE]; // D1~D3 拼接中的遥控原始帧。
static volatile uint8_t communication_rc_assembly_mask; // 已收到 D1/D2 分片的位掩码。
static volatile uint32_t communication_rc_assembly_start_ms; // 本轮 D1 接收时间，后续分片共用此有效期。
static uint8_t communication_rc_snapshot[COMM_RC_FRAME_SIZE]; // 提交给任务解析的完整遥控快照。
static volatile bool communication_rc_snapshot_ready; // 是否有完整遥控帧等待任务解析。
static volatile uint32_t communication_rc_snapshot_ms; // 完整遥控帧对应的 D1 接收时间。

Communication_RcControl_t communication_rc; // 当前已解析的遥控器数据。
volatile bool communication_rc_online = false; // 遥控链路当前是否在线。
volatile uint32_t communication_rc_valid_count = 0U; // 累计有效遥控帧数。
volatile uint32_t communication_rc_last_valid_ms = 0U; // 最近有效遥控帧时间。
volatile uint32_t communication_rc_assembly_error_count = 0U; // D1~D3 拼帧错误数。

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

// 中断中只做分包拼接和快照提交，不在这里进行业务解析。
static void Communication_RC_AcceptFragment(
    uint16_t std_id,
    const uint8_t data[COMM_CAN_FRAME_SIZE])
{
    if (std_id == COMM_CAN_RX_ID_D1)
    {
        memcpy(&communication_rc_assembly[0], data, COMM_CAN_FRAME_SIZE);
        communication_rc_assembly_start_ms = HAL_GetTick();
        communication_rc_assembly_mask = COMM_RC_PART_D1;
        return;
    }

    if (std_id == COMM_CAN_RX_ID_D2)
    {
        if ((communication_rc_assembly_mask != COMM_RC_PART_D1) ||
            ((uint32_t)(HAL_GetTick() - communication_rc_assembly_start_ms) >=
             communication_config.timeout_ms))
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
            ((uint32_t)(HAL_GetTick() - communication_rc_assembly_start_ms) >=
             communication_config.timeout_ms) ||
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
        // 后续分片不延长 D1 的有效期。
        communication_rc_snapshot_ms = communication_rc_assembly_start_ms;
        communication_rc_snapshot_ready = true;
        communication_rc_assembly_mask = 0U;
    }

    // D4 底盘角速度、D5 四轮转速由通用 CAN 快照保存。
}

static int32_t Communication_CAN_RxIndex(uint16_t std_id)
{
    if ((std_id >= COMM_CAN_RX_ID_D1) && (std_id <= COMM_CAN_RX_ID_D7))
    {
        return (int32_t)(std_id - COMM_CAN_RX_ID_D1);
    }

    return -1;
}

// 配置下板反馈 ID 过滤器，启动 CAN2 接收并开启接收中断。
HAL_StatusTypeDef Communication_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    // CAN2 bank 14 接收 D1~D4，bank 16 接收 D5~D7。
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

    // bank 15 留给 Yaw 电机；bank 16 接收轮速、热量和弹速。
    filter.FilterBank = COMM_CAN_WHEEL_FILTER_BANK;
    filter.FilterIdHigh = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D5);
    filter.FilterIdLow = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D6);
    filter.FilterMaskIdHigh = COMM_CAN_STD_ID_TO_FILTER(COMM_CAN_RX_ID_D7);
    filter.FilterMaskIdLow = filter.FilterMaskIdHigh;
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

    // Yaw 电机与板间通信共用 CAN2，允许 Motor4310_Init 已启动总线。
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

HAL_StatusTypeDef Communication_CAN_SendYawAngle(float angle_deg)
{
    return Communication_CAN_SendYawState(angle_deg, false, false, false,
                                          false, true, false);
}

HAL_StatusTypeDef Communication_CAN_SendYawState(float angle_deg, bool turning,
                                                  bool allow_turn,
                                                  bool allow_spin,
                                                  bool spin_selected,
                                                  bool angle_valid,
                                                  bool bottom_mode_blocked)
{
    uint8_t data[COMM_CAN_FRAME_SIZE] = {0};
    int16_t encoded;

    if (angle_valid && !(angle_deg >= -FLT_MAX && angle_deg <= FLT_MAX))
    { return HAL_ERROR; }
    if (!angle_valid) { angle_deg = 0.0f; }
    if (angle_deg > 327.67f) { angle_deg = 327.67f; }
    else if (angle_deg < -327.68f) { angle_deg = -327.68f; }
    encoded = (int16_t)(angle_deg * 100.0f);
    data[0] = (uint8_t)(uint16_t)encoded;
    data[1] = (uint8_t)((uint16_t)encoded >> 8);
    data[2] = (angle_valid ? 0x01U : 0U) |
              (turning ? 0x02U : 0U) |
              (allow_turn ? 0x04U : 0U) |
              (allow_spin ? 0x08U : 0U) |
              (spin_selected ? 0x10U : 0U) |
              (bottom_mode_blocked ? 0x20U : 0U);
    return Communication_CAN_SendC1(data);
}

bool Communication_CAN_GetChassisYawRate(float *rate_deg_s)
{
    uint32_t sample_ms;
    return Communication_CAN_GetChassisYawRateState(rate_deg_s, &sample_ms);
}

bool Communication_CAN_GetChassisYawRateState(float *rate_deg_s, uint32_t *sample_ms)
{
    Communication_CanRxFrame_t frame;
    int16_t encoded;

    if (rate_deg_s == NULL || sample_ms == NULL ||
        !Communication_CAN_GetLatest(COMM_CAN_RX_ID_D4, &frame) ||
        (uint32_t)(HAL_GetTick() - frame.last_rx_ms) >=
            communication_config.yaw_rate_timeout_ms ||
        (frame.data[2] & 0x01U) == 0U)
    {
        return false;
    }
    encoded = (int16_t)((uint16_t)frame.data[0] |
                        ((uint16_t)frame.data[1] << 8));
    *rate_deg_s = (float)encoded * 0.01f;
    *sample_ms = frame.last_rx_ms;
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

static bool Communication_RC_Parse(
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
    bool frame_valid;

    saved_primask = __get_PRIMASK();
    __disable_irq();
    now_ms = HAL_GetTick();
    // 缺片期间也清理过期拼帧，保留仍在有效期内的新 D1。
    if ((communication_rc_assembly_mask != 0U) &&
        ((uint32_t)(now_ms - communication_rc_assembly_start_ms) >=
         communication_config.timeout_ms))
    {
        communication_rc_assembly_mask = 0U;
        communication_rc_assembly_error_count++;
    }
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
        frame_valid = Communication_RC_Parse(frame, &decoded);
        saved_primask = __get_PRIMASK();
        __disable_irq();
        // 任务等待或解析期间过期的完整帧也不发布。
        if ((uint32_t)(HAL_GetTick() - received_ms) <
            communication_config.timeout_ms)
        {
            communication_rc = decoded;
            if (frame_valid)
            {
                communication_rc_last_valid_ms = received_ms;
                communication_rc_valid_count++;
                communication_rc_online = true;
            }
        }
        __set_PRIMASK(saved_primask);
    }

    now_ms = HAL_GetTick();
    if ((!communication_rc_online) ||
        ((uint32_t)(now_ms - communication_rc_last_valid_ms) >=
         communication_config.timeout_ms))
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
             communication_config.timeout_ms;
    __set_PRIMASK(saved_primask);

    return online;
}

bool Communication_RC_IsOnline(void)
{
    return communication_rc_online &&
           (uint32_t)(HAL_GetTick() - communication_rc_last_valid_ms) <
           communication_config.timeout_ms;
}

// CAN1/2 RX0 IRQ 均由 CubeMX 生成，在此统一分发。
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

    // 一次中断排空 FIFO0，避免高频报文在队列中积压。
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
            // CAN1 共用：Pitch 4310、两路 3508、M2006 和 LK4005。
            motor4310.process_can_frame(hcan, header.StdId, data);
            motor3508.process_can_frame(hcan, header.StdId, data);
            motor2006.process_can_frame(hcan, header.StdId, data);
            dial_motor.process_can_frame(hcan, header.StdId, data);
            continue;
        }

        if (header.StdId == MOTOR4310_FEEDBACK_CAN_ID)
        {
            motor4310.process_can_frame(hcan, header.StdId, data);
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
    }
}

bool Communication_GetHeatSnapshot(Communication_HeatSnapshot_t *heat)
{
    Communication_CanRxFrame_t frame;
    if (heat == NULL) { return false; }
    memset(heat, 0, sizeof(*heat));
    if (!Communication_CAN_GetLatest(COMM_CAN_RX_ID_D6, &frame)) { return false; }
    heat->heat = (uint16_t)(frame.data[0] | ((uint16_t)frame.data[1] << 8));
    heat->limit = (uint16_t)(frame.data[2] | ((uint16_t)frame.data[3] << 8));
    heat->cooling = (uint16_t)(frame.data[4] | ((uint16_t)frame.data[5] << 8));
    heat->valid = (frame.data[6] & 1U) != 0U;
    heat->output_allowed = (frame.data[6] & 2U) != 0U;
    heat->sequence = frame.data[7];
    heat->last_rx_ms = frame.last_rx_ms;
    return true;
}

// 按原样本年龄恢复测速时刻，供发射任务排除历史测速。
bool Communication_GetShotSnapshot(Communication_ShotSnapshot_t *shot)
{
    Communication_CanRxFrame_t frame;
    if (shot == NULL) { return false; }
    memset(shot, 0, sizeof(*shot));
    if (!Communication_CAN_GetLatest(COMM_CAN_RX_ID_D7, &frame)) { return false; }
    shot->speed_m_s = (uint16_t)(frame.data[0] | ((uint16_t)frame.data[1] << 8)) * 0.01f;
    shot->speed_limit_m_s = (uint16_t)(frame.data[2] | ((uint16_t)frame.data[3] << 8)) * 0.01f;
    shot->sequence = (uint16_t)(frame.data[4] | ((uint16_t)frame.data[5] << 8));
    shot->valid = (frame.data[6] & 1U) != 0U;
    shot->speed_limit_valid = (frame.data[6] & 2U) != 0U;
    shot->last_rx_ms = frame.last_rx_ms;
    shot->sample_ms = frame.last_rx_ms - (uint32_t)frame.data[7] * 10U;
    return true;
}

// 绑定当前状态和模块接口。
const BoardLinkModule board_link =
{
    .config = &communication_config,
    .data = {
        .frames = communication_rx_frames,
        .remote = &communication_rc,
    },
    .diagnostics = {
        .remote_online = &communication_rc_online,
        .valid_count = &communication_rc_valid_count,
        .last_valid_ms = &communication_rc_last_valid_ms,
        .assembly_errors = &communication_rc_assembly_error_count,
    },
    .can_init = Communication_CAN_Init,
    .can_send = Communication_CAN_Send,
    .can_send_c1 = Communication_CAN_SendC1,
    .can_send_c2 = Communication_CAN_SendC2,
    .can_send_lift_lock = Communication_CAN_SendLiftLock,
    .can_send_yaw_angle = Communication_CAN_SendYawAngle,
    .can_send_yaw_state = Communication_CAN_SendYawState,
    .can_get_chassis_yaw_rate = Communication_CAN_GetChassisYawRate,
    .can_get_chassis_yaw_rate_state = Communication_CAN_GetChassisYawRateState,
    .can_get_latest = Communication_CAN_GetLatest,
    .get_heat_snapshot = Communication_GetHeatSnapshot,
    .get_shot_snapshot = Communication_GetShotSnapshot,
    .process = Communication_Process,
    .rc_get = Communication_RC_Get,
    .rc_is_online = Communication_RC_IsOnline,
};
