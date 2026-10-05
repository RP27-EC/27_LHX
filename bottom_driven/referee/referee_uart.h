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

// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const uint8_t *dma_bytes; // 循环 DMA 原始字节。
} RefereeUartModuleDataRefs;

typedef struct
{
    const volatile RefereeUartDiagnostics_t *state; // 串口 DMA、队列和错误统计。
} RefereeUartModuleDiagnosticsRefs;

typedef struct
{
    RefereeUartModuleDataRefs data; // 反馈与解析数据引用。
    RefereeUartModuleDiagnosticsRefs diagnostics; // 通信诊断引用。

    // 初始化。
    HAL_StatusTypeDef (*start)(UART_HandleTypeDef *uart); // 绑定裁判串口并启动循环 DMA。

    // 状态维护。
    void (*process)(uint32_t now_ms); // 周期处理接收数据和在线状态。

    // 接收与解析。
    bool (*on_rx_event)(UART_HandleTypeDef *uart, uint16_t position); // 处理裁判串口接收事件。
    bool (*on_error)(UART_HandleTypeDef *uart); // 处理裁判串口错误。
} RefereeUartModule;

extern const RefereeUartModule referee_uart_driver; // 模块统一访问入口。

#endif
