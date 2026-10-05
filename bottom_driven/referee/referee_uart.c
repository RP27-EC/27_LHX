#include "referee_uart.h"
#include "referee.h"
#include <string.h>

// 数组起点及长度均为Cache行的整数倍；当前链接配置已把.bss放在AXI SRAM。
uint8_t referee_uart_dma_buffer[REFEREE_DMA_BUFFER_SIZE] __attribute__((aligned(32)));

volatile RefereeUartDiagnostics_t referee_uart_diagnostics; // 接收事件、丢包与重启状态。
static UART_HandleTypeDef *referee_uart; // 已绑定的裁判串口。
static uint8_t byte_queue[REFEREE_RX_QUEUE_SIZE]; // 中断搬运与任务解析之间的字节队列。
static volatile uint32_t queue_head, queue_tail; // 单生产者/单消费者累计位置。
static volatile bool queue_gap, restart_pending; // 数据断点与待重启标记。
static uint16_t dma_previous_position; // 上次已经搬运到的位置。

// DMA 启动前清理缓存，避免旧缓存回写覆盖接收数据。
static void dma_cache_prepare(void) {
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)referee_uart_dma_buffer, REFEREE_DMA_BUFFER_SIZE);
    }
}
// CPU 读 DMA 数据前使缓存失效。
static void dma_cache_invalidate(void) {
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)referee_uart_dma_buffer, REFEREE_DMA_BUFFER_SIZE);
    }
}
// 绑定循环 DMA，重复启动和运行中换绑均不允许。
HAL_StatusTypeDef RefereeUart_Start(UART_HandleTypeDef *uart) {
    HAL_StatusTypeDef result;
    uint32_t primask;
    if (uart == NULL || uart->hdmarx == NULL || uart->hdmarx->Init.Mode != DMA_CIRCULAR) { return HAL_ERROR; }
    // 运行中禁止换绑串口；错误重启保留原绑定。
    if (referee_uart != NULL && referee_uart != uart) { return HAL_BUSY; }
    if (referee_uart_diagnostics.receiving) { return HAL_BUSY; }
    primask=__get_PRIMASK(); __disable_irq();
    queue_head=0U; queue_tail=0U; queue_gap=false;
    dma_previous_position=0U; referee_uart=uart;
    __set_PRIMASK(primask);
    dma_cache_prepare();
    result=HAL_UARTEx_ReceiveToIdle_DMA(uart,referee_uart_dma_buffer,REFEREE_DMA_BUFFER_SIZE);
    referee_uart_diagnostics.receiving=result==HAL_OK;
    if (result!=HAL_OK) { referee_uart_diagnostics.restart_error_count++; }
    // 循环 DMA 保留 HT、TC 中断，通过半满和全满事件持续搬运数据。
    return result;
}
// 中断只搬运字节，队列满时记录数据断点。
static void enqueue(uint16_t begin, uint16_t length) {
    uint32_t head=queue_head; uint16_t i;
    if (length>REFEREE_RX_QUEUE_SIZE-(uint32_t)(head-queue_tail)) {
        referee_uart_diagnostics.queue_overflow_count++; queue_gap=true; return;
    }
    for(i=0;i<length;i++) { byte_queue[(head+i)%REFEREE_RX_QUEUE_SIZE]=referee_uart_dma_buffer[begin+i]; }
    __DMB(); queue_head=head+length;
    referee_uart_diagnostics.received_byte_count+=length;
}
// 按 DMA 位置差搬运新字节，处理缓冲回绕。
bool RefereeUart_OnRxEvent(UART_HandleTypeDef *uart, uint16_t position) {
    if (uart==NULL || uart!=referee_uart) { return false; }
    if (position>REFEREE_DMA_BUFFER_SIZE) { queue_gap=true; return true; }
    referee_uart_diagnostics.rx_event_count++;
    referee_uart_diagnostics.last_dma_position=position;
    referee_uart_diagnostics.last_rx_ms=HAL_GetTick();
    dma_cache_invalidate();
    if (position>dma_previous_position) { enqueue(dma_previous_position,position-dma_previous_position); }
    else if(position<dma_previous_position) {
        enqueue(dma_previous_position,REFEREE_DMA_BUFFER_SIZE-dma_previous_position);
        enqueue(0U,position);
    }
    dma_previous_position=position; return true;
}
// 记录错误，接收重启留给任务处理。
bool RefereeUart_OnError(UART_HandleTypeDef *uart) {
    if(uart==NULL || uart!=referee_uart) { return false; }
    referee_uart_diagnostics.last_uart_error=uart->ErrorCode;
    referee_uart_diagnostics.uart_error_count++;
    referee_uart_diagnostics.receiving=false;
    restart_pending=true; queue_gap=true; return true;
}
// 任务中处理断点、重启和裁判协议解析。
void RefereeUart_Process(uint32_t now_ms) {
    uint8_t block[256]; uint16_t count=0U; bool gap;
    uint32_t primask=__get_PRIMASK();
    __disable_irq(); gap=queue_gap; queue_gap=false;
    if(gap) { queue_tail=queue_head; }
    __set_PRIMASK(primask);
    if(gap) { referee.reset_stream(); }
    if(restart_pending && referee_uart!=NULL) {
        restart_pending=false;
        (void)HAL_UART_AbortReceive(referee_uart);
        if(RefereeUart_Start(referee_uart)!=HAL_OK) { restart_pending=true; }
    }
    while(count<sizeof(block) && queue_tail!=queue_head) {
        block[count++]=byte_queue[queue_tail%REFEREE_RX_QUEUE_SIZE]; __DMB(); queue_tail++;
    }
    if(count) { referee.feed(block,count,now_ms); }
    else { referee.update(now_ms); }
}

// HAL回调由Core/Src/usart.c统一拥有，调用本模块的OnRxEvent/OnError。

// 绑定现有状态与函数，供外部通过模块结构体访问。
const RefereeUartModule referee_uart_driver =
{
    .data = {
        .dma_bytes = referee_uart_dma_buffer,
    },
    .diagnostics = {
        .state = &referee_uart_diagnostics,
    },
    .start = RefereeUart_Start,
    .process = RefereeUart_Process,
    .on_rx_event = RefereeUart_OnRxEvent,
    .on_error = RefereeUart_OnError,
};
