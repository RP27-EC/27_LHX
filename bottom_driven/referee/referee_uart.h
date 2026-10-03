#ifndef REFEREE_UART_H
#define REFEREE_UART_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define REFEREE_DMA_BUFFER_SIZE 512U // 裁判循环 DMA 缓冲长度，字节。
#define REFEREE_RX_QUEUE_SIZE 2048U // 中断到任务的接收队列容量，字节。
// 当前Keil的RW_IRAM1已在AXI SRAM(0x24000000)，DMA1可访问。
// 若改链接内存区域，必须保证下方DMA数组不落入0x20000000的DTCM。
typedef struct {
    bool receiving; // DMA接收已经启动。
    uint32_t rx_event_count; // IDLE/半满/全满回调计数。
    uint32_t received_byte_count; // 已搬入队列的字节数。
    uint32_t queue_overflow_count; // 字节队列溢出，解析器会重新同步。
    uint32_t uart_error_count; // 串口接收错误次数。
    uint32_t restart_error_count; // DMA 启动或重启失败次数。
    uint32_t last_uart_error; // 最近一次 HAL 错误码。
    uint32_t last_rx_ms; // 最近一次接收回调时间。
    uint16_t last_dma_position; // 最近回调的 DMA 写入位置。
} RefereeUartDiagnostics_t;
extern volatile RefereeUartDiagnostics_t referee_uart_diagnostics;
extern uint8_t referee_uart_dma_buffer[REFEREE_DMA_BUFFER_SIZE];
HAL_StatusTypeDef RefereeUart_Start(UART_HandleTypeDef *uart); // 外设配置完成后绑定串口并启动循环DMA。
void RefereeUart_Process(uint32_t now_ms); // 只由一个任务周期调用；CRC和解析在这里执行。
bool RefereeUart_OnRxEvent(UART_HandleTypeDef *uart, uint16_t position);
bool RefereeUart_OnError(UART_HandleTypeDef *uart);
#endif
