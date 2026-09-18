#include "telecontrol.h"
#include <string.h>

/* 中断写入，task 读取；不让 task 直接读取正在接收的 DMA 缓冲区。 */
static uint8_t rc_frame_snapshot[RC_FRAME_LEN];
static volatile bool rc_frame_ready = false;

/* 用于调试器观察。 */
volatile uint32_t rc_rx_frame_count = 0;
volatile uint16_t rc_rx_last_size = 0;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];//接收数据储存数值

RC_ctrl_t rc_ctrl = { .rc = { .ch = {0}, .s = {RC_SW_MID, RC_SW_MID} } };

//串口DMA接收初始化
void control_usart_init(uint8_t *rx_1buff,uint8_t *rx_2buff,uint16_t dma_buf_num)
{
    DMA_HandleTypeDef *dma = huart5.hdmarx;
    DMA_Stream_TypeDef *stream;

    if ((dma == NULL) ||
        (rx_1buff == NULL) ||
        (rx_2buff == NULL) ||
        (dma_buf_num != SBUS_RX_BUF_NUM))
    {
        return;
    }

    /* H7 的 Instance 是 void *；UART5 使用 DMA1 Stream。 */
    stream = (DMA_Stream_TypeDef *)dma->Instance;

    /* 配置期间关闭接收 DMA 请求和 IDLE 中断。 */
    CLEAR_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);

    __HAL_DMA_DISABLE(dma);
    while ((stream->CR & DMA_SxCR_EN) != 0U)
    {
        /* 等待 DMA 真正停止，才能修改地址和计数。 */
    }

    /* 当前方案由 UART IDLE 中断处理帧，不使用 DMA 完成回调。 */
    __HAL_DMA_DISABLE_IT(dma,
                        DMA_IT_TC  |
                        DMA_IT_HT  |
                        DMA_IT_TE  |
                        DMA_IT_DME |
                        DMA_IT_FE);

    __HAL_DMA_CLEAR_FLAG(
        dma,
        __HAL_DMA_GET_TC_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_HT_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_TE_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
        __HAL_DMA_GET_FE_FLAG_INDEX(dma));

    /* H723 UART5 的接收数据寄存器是 RDR。 */
    stream->PAR = (uint32_t)&huart5.Instance->RDR;
    stream->M0AR = (uint32_t)rx_1buff;
    stream->M1AR = (uint32_t)rx_2buff;
    stream->NDTR = dma_buf_num;

    /* 从 M0 开始，开启双缓冲。 */
    CLEAR_BIT(stream->CR, DMA_SxCR_CT);
    SET_BIT(stream->CR, DMA_SxCR_DBM);

    rc_frame_ready = false;
    rc_rx_frame_count = 0;
    rc_rx_last_size = 0;

    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    __HAL_UART_CLEAR_FLAG(&huart5,
                         UART_CLEAR_OREF |
                         UART_CLEAR_NEF  |
                         UART_CLEAR_PEF  |
                         UART_CLEAR_FEF);

    __HAL_DMA_ENABLE(dma);
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}


//IDLE 处理函数函数
void RC_UART5_IdleHandler(void)
{
    DMA_HandleTypeDef *dma = huart5.hdmarx;
    DMA_Stream_TypeDef *stream;
    uint32_t uart_errors;
    uint32_t target_before;
    uint32_t current_target;
    uint32_t full_before;
    uint32_t remaining;
    uint16_t received = 0U;
    uint8_t *completed_buffer;

    if ((__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_IDLE) == RESET))
    {
        return;
    }

    uart_errors = huart5.Instance->ISR &
                  (USART_ISR_PE | USART_ISR_FE |
                   USART_ISR_NE | USART_ISR_ORE);

    __HAL_UART_CLEAR_IDLEFLAG(&huart5);

    if (dma == NULL)
    {
        return;
    }

    /* 转为 DMA Stream 寄存器类型后访问 CR 等寄存器。 */
    stream = (DMA_Stream_TypeDef *)dma->Instance;

    /* 必须在软件停止 DMA 之前记录状态。 */
    target_before = stream->CR & DMA_SxCR_CT;
    full_before = __HAL_DMA_GET_FLAG(
        dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma));

    __HAL_DMA_DISABLE(dma);
    while ((stream->CR & DMA_SxCR_EN) != 0U)
    {
        /* 等待 DMA 停止，再读取计数和操作缓冲区。 */
    }

    current_target = stream->CR & DMA_SxCR_CT;
    remaining = __HAL_DMA_GET_COUNTER(dma);

    if (remaining <= SBUS_RX_BUF_NUM)
    {
        received = (uint16_t)(SBUS_RX_BUF_NUM - remaining);
    }

    /* 把停止过程中出现的串口错误也计入。 */
    uart_errors |= huart5.Instance->ISR &
                   (USART_ISR_PE | USART_ISR_FE |
                    USART_ISR_NE | USART_ISR_ORE);

    rc_rx_last_size = received;

    completed_buffer = (current_target == 0U)
                       ? sbus_rx_buf[0]
                       : sbus_rx_buf[1];

    /*
     * 只接受完整的 18 字节单帧：
     * 停止前没有收满缓冲区，停止期间没有切换缓冲区，
     * 且没有串口错误。
     */
    if ((received == RC_FRAME_LEN) &&
        (full_before == RESET) &&
        (current_target == target_before) &&
        (uart_errors == 0U))
    {
        memcpy(rc_frame_snapshot, completed_buffer, RC_FRAME_LEN);
        rc_rx_frame_count++;
        rc_frame_ready = true;
    }

    /* 下次从另一个缓冲区重新接收。 */
    stream->CR ^= DMA_SxCR_CT;
    __HAL_DMA_SET_COUNTER(dma, SBUS_RX_BUF_NUM);

    __HAL_DMA_CLEAR_FLAG(
        dma,
        __HAL_DMA_GET_TC_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_HT_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_TE_FLAG_INDEX(dma)  |
        __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
        __HAL_DMA_GET_FE_FLAG_INDEX(dma));

    __HAL_UART_CLEAR_FLAG(&huart5,
                         UART_CLEAR_OREF |
                         UART_CLEAR_NEF  |
                         UART_CLEAR_PEF  |
                         UART_CLEAR_FEF);

    __HAL_DMA_ENABLE(dma);
}

bool RC_TakeFrame(uint8_t frame[RC_FRAME_LEN])
{
    uint32_t saved_primask;
    bool available;

    if (frame == NULL)
    {
        return false;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();

    available = rc_frame_ready;

    if (available)
    {
        memcpy(frame, rc_frame_snapshot, RC_FRAME_LEN);
        rc_frame_ready = false;
    }

    __set_PRIMASK(saved_primask);

    return available;
}

/*
 * 从模板 rc_sensor_update 移植 DBUS 位解包逻辑。
 * 仅解析摇杆、拨轮和拨杆；字节 6~15 的键鼠数据暂不处理。
 * 输出通道以 1024 为中心归零，正常范围 -660~660。
 * 不保留模板针对 ch3 == -660 的特例，以免屏蔽合法满量程输入。
 */
bool RC_ParseFrame(const uint8_t frame[RC_FRAME_LEN], RC_ctrl_t *control)
{
    RC_ctrl_t decoded = {0};
    uint32_t i;

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

    /* 与模板 s1/s2 位序一致：s[0] 取 bit 7~6，s[1] 取 bit 5~4。 */
    decoded.rc.s[0] = (frame[5] >> 6) & 0x03U;
    decoded.rc.s[1] = (frame[5] >> 4) & 0x03U;

    /* 摇杆异常时清零输出并拒绝该帧；两个拨杆均须为 1、2、3。 */
    for (i = 0U; i < 4U; i++)
    {
        if ((decoded.rc.ch[i] < -660) || (decoded.rc.ch[i] > 660))
        {
            memset(control, 0, sizeof(*control));
            control->rc.s[0] = RC_SW_MID;
            control->rc.s[1] = RC_SW_MID;
            return false;
        }
    }
    if ((decoded.rc.s[0] == 0U) || (decoded.rc.s[1] == 0U))
    {
        memset(control, 0, sizeof(*control));
        control->rc.s[0] = RC_SW_MID;
        control->rc.s[1] = RC_SW_MID;
        return false;
    }

    /* 保留模板拨轮异常归零处理。 */
    if ((decoded.rc.ch[4] < -660) || (decoded.rc.ch[4] > 660))
    {
        decoded.rc.ch[4] = 0;
    }

    *control = decoded;
    return true;
}
