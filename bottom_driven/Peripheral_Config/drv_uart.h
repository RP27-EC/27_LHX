/**
 ******************************************************************************
 * @file        drv_uart.h
 * @author      RobotPilots@2020
 * @brief       UART Driver Package(Based on HAL).
 ******************************************************************************
 * @attention
 * 
 * Copyright 2020 RobotPilots
 * 
 * @Version     V1.0
 * @date        15-August-2020
 ******************************************************************************
 */
#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void USART1_Init(void);
void USART2_Init(void);
void USART3_Init(void);
void USART5_Init(void);
void USART6_Init(void);
void USART8_Init(void);
void USART9_Init(void);
void USART10_Init(void);
void WL_UART_printf(char *format, ...);

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart1;

#define USART1_RX_BUF_LEN     400
#define USART7_RX_BUF_LEN	  400
#define USART8_RX_BUF_LEN     400
#define USART9_RX_BUF_LEN     200
#define USART10_RX_BUF_LEN    200

/* 1: use HAL_UARTEx_ReceiveToIdle_DMA for RC UART5, 0: use legacy double-buffer DMA */
#ifndef UART5_RC_USE_HAL_TOIDLE_DMA
#define UART5_RC_USE_HAL_TOIDLE_DMA  1
#endif

#endif
