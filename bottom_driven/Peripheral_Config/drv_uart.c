/**
 ******************************************************************************
 * @file        drv_uart.c
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
 
/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"
#include "judge_protocol.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;

/* Private macro -------------------------------------------------------------*/
#define USART5_RX_DATA_FRAME_LEN	(18u)	// 串口2数据帧长度
#define USART5_RX_BUF_LEN			(USART5_RX_DATA_FRAME_LEN)	// 串口2接收缓冲区长度


//#define USART5_RX_BUF_LEN	  600	//200
#define BUFF_SIZE 512
uint8_t rx_buff[BUFF_SIZE];

/* Private function prototypes -----------------------------------------------*/
__WEAK void USART10_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART9_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART8_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART7_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength);
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static void uart5_rc_start_receive(void);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__attribute__((section (".AXI_SRAM"))) uint8_t usart1_dma_rxbuf[USART1_RX_BUF_LEN];
__attribute__((section (".AXI_SRAM"))) uint8_t usart10_dma_rxbuf[USART10_RX_BUF_LEN];
__attribute__((section (".AXI_SRAM"))) uint8_t usart7_dma_rxbuf[USART7_RX_BUF_LEN];
__attribute__((section (".AXI_SRAM"))) uint8_t usart8_dma_rxbuf[USART8_RX_BUF_LEN];
__attribute__((section (".AXI_SRAM"))) uint8_t usart9_dma_rxbuf[USART9_RX_BUF_LEN];
#if UART5_RC_USE_HAL_TOIDLE_DMA
__attribute__((section (".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[USART5_RX_BUF_LEN];
#else
__attribute__((section (".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[2][USART5_RX_BUF_LEN];
#endif

/* Exported variables --------------------------------------------------------*/


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
#if UART5_RC_USE_HAL_TOIDLE_DMA
		if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
		{
			if(Size == USART5_RX_DATA_FRAME_LEN)
			{
				USART5_rxDataHandler(usart5_dma_rxbuf);
			}
			uart5_rc_start_receive();
		}
#else
		if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) 
		{ 
			__HAL_DMA_DISABLE(huart->hdmarx); 

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT; 

			__HAL_DMA_SET_COUNTER(huart->hdmarx,USART5_RX_DATA_FRAME_LEN*2); 

			if(Size == USART5_RX_DATA_FRAME_LEN) 
			{ 
			USART5_rxDataHandler(usart5_dma_rxbuf[0]); 
			} 
		}
		else
		{
			__HAL_DMA_DISABLE(huart->hdmarx); 

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT); 
	 
			__HAL_DMA_SET_COUNTER(huart->hdmarx,USART5_RX_DATA_FRAME_LEN*2); 

			if(Size == USART5_RX_DATA_FRAME_LEN) 
			{ 
			USART5_rxDataHandler(usart5_dma_rxbuf[1]); 
			} 		 
		}							   
#endif
	}

#if !UART5_RC_USE_HAL_TOIDLE_DMA
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
	/* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_ENABLE(huart->hdmarx);
#endif
}

#if UART5_RC_USE_HAL_TOIDLE_DMA
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART5)
	{
		uart5_rc_start_receive();
	}
}
#endif

static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

// huart->RxEventType = HAL_UART_RXEVENT_IDLE;

 huart->RxXferSize    = DataLength*2;
 
	/*使能串口DMA模式*/
 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

	/*使能uart中断*/
 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
 
	/*关闭DMA传输，确保可以设置DMA起点和终点*/
do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

	/*配置DMA起点地址和终点地址*/
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

	/*配置DMA数据传输长度*/
((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	/*使能DMA*/
__HAL_DMA_ENABLE(huart->hdmarx);	
}

void USART5_Init(void)
{
#if UART5_RC_USE_HAL_TOIDLE_DMA
	uart5_rc_start_receive();
#else
	USART_DMAEx_MultiBuffer_Init(&huart5, (uint32_t*)&usart5_dma_rxbuf[0], \
							    (uint32_t*)&usart5_dma_rxbuf[1], USART5_RX_BUF_LEN); // 接收完毕后重启
#endif
}

/**
 *	@brief	USART1 Initialization
 */
void USART1_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart1, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
	
}

/**
 *	@brief	USART10 Initialization
 */
void USART10_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART10_RX_BUF_LEN);
	
}

/**
 *	@brief	USART10 Initialization
 */
void USART7_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart7, usart7_dma_rxbuf, USART7_RX_BUF_LEN);
	
}

/**
 *	@brief	USART8 Initialization
 */
void USART8_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart8, usart8_dma_rxbuf, USART8_RX_BUF_LEN);
	
}

/**
 *	@brief	USART9 Initialization
 */
void USART9_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart9, UART_IT_IDLE);
		
	
	HAL_UART_Receive_DMA(&huart9, usart9_dma_rxbuf, USART9_RX_BUF_LEN);	
	
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in DRV_UART_IRQHandler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	
	if (huart == &huart10)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART10_RX_BUF_LEN);
		/* handle dbus data dbus_buf from DMA */
		USART10_rxDataHandler(usart10_dma_rxbuf);
		//memset(usart10_dma_rxbuf, 0, USART10_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	
	else if (huart == &huart1)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_Receive_DMA(&huart1, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
		/* handle dbus data dbus_buf from DMA */
		USART1_rxDataHandler(usart1_dma_rxbuf);
		memset(usart1_dma_rxbuf, 0, USART1_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	
	else if (huart == &huart7)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_Receive_DMA(&huart7, usart7_dma_rxbuf, USART7_RX_BUF_LEN);
		/* handle dbus data dbus_buf from DMA */
		USART7_rxDataHandler(usart7_dma_rxbuf);
		//memset(usart7_dma_rxbuf, 0, USART7_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	
	else if (huart == &huart8)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_Receive_DMA(&huart8, usart8_dma_rxbuf, USART8_RX_BUF_LEN);
		/* handle dbus data dbus_buf from DMA */
		USART8_rxDataHandler(usart8_dma_rxbuf);
		//memset(usart8_dma_rxbuf, 0, USART8_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	
	else if (huart == &huart9)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_Receive_DMA(&huart9, usart9_dma_rxbuf, USART9_RX_BUF_LEN);
		/* handle dbus data dbus_buf from DMA */
		USART9_rxDataHandler(usart9_dma_rxbuf);
		//memset(usart9_dma_rxbuf, 0, USART9_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
}

static void uart5_rc_start_receive(void)
{
#if UART5_RC_USE_HAL_TOIDLE_DMA
	HAL_UART_DMAStop(&huart5);
	__HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_rxbuf, USART5_RX_BUF_LEN);
	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
#endif
}


static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		((DMA_Stream_TypeDef  *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		((DMA_Stream_TypeDef  *)hdma->Instance)->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = DstAddress;

			/* Configure DMA Stream source address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = SrcAddress;
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}
/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

void WL_UART_printf(char *format, ...)
{
		char String[100];
		va_list arg;
		va_start(arg, format);
		vsprintf(String, format, arg);
		va_end(arg);
		
		HAL_UART_Transmit(&huart7, (uint8_t *)String, strlen(String), 1000);
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART1 处理协议
 */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART7 处理协议
 */
__WEAK void USART7_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART5 处理协议
 */
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART10 处理协议
 */
__WEAK void USART10_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART8 处理协议
 */
__WEAK void USART8_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART9 处理协议
 */
__WEAK void USART9_rxDataHandler(uint8_t *rxBuf)
{	
}
