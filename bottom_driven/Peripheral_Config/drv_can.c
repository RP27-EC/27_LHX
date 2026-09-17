/**
  ******************************************************************************
  * @file    drv_can.c
  * @brief   CAN底层驱动
  ******************************************************************************
  * @attention
  * 
  * Copyright 2024 RobotPilots
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"
#include "fdcan.h"
/* Exported variables --------------------------------------------------------*/
/* CAN 200/1FF发送数组 */

/* Private function prototypes -----------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN3_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
/* Private variables ---------------------------------------------------------*/
/**
  * @brief CAN1\CAN2实例
  */

extern FDCAN_HandleTypeDef hfdcan3;

CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_RxFrameTypeDef hcan3RxFrame;
/* Exported functions --------------------------------------------------------*/
void FDCAN1_Restart(void) {
    // 禁用 FDCAN 模块
    HAL_FDCAN_DeInit(&hfdcan1);

		MX_FDCAN1_Init();

   CAN1_Filter_Init();

    // 启动 FDCAN 模块
    HAL_FDCAN_Start(&hfdcan1);

    // 启用中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING, 0);
}

void FDCAN2_Restart(void) {
    // 禁用 FDCAN 模块
    HAL_FDCAN_DeInit(&hfdcan2);

		MX_FDCAN2_Init();

   CAN2_Filter_Init();

    // 启动 FDCAN 模块
    HAL_FDCAN_Start(&hfdcan2);

    // 启用中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING, 0);
}
/**
  * @brief  can接受中断，在stm32f4xx_hal_can.c内弱定义
  * @param  
  * @retval 
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
  
  if(hcan == &hfdcan1)
  {
		HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		
		CAN1_rxDataHandler(hcan1RxFrame.header.Identifier, hcan1RxFrame.data);
  }
  else if(hcan == &hfdcan2)
  {
		HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		
		CAN2_rxDataHandler(hcan2RxFrame.header.Identifier, hcan2RxFrame.data);
  }
  else 
  {
    return;
  }
}
FDCAN_RxHeaderTypeDef RxHeader2;
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
		if(hfdcan->Instance == FDCAN3)
    {
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &hcan3RxFrame.header, hcan3RxFrame.data);
			CAN3_rxDataHandler(hcan3RxFrame.header.Identifier, hcan3RxFrame.data);
    }
  }
}


/**
  * @brief  CAN发送数据
  * @param  hcan: CAN_HandleTypeDef
  * @param  stdId: 标准ID
  * @param  dat: 数组指针
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
	FDCAN_TxHeaderTypeDef tx_message;
	
	tx_message.IdType = FDCAN_STANDARD_ID;
	tx_message.Identifier = stdId;
	tx_message.DataLength = FDCAN_DLC_BYTES_8;
	tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_message.BitRateSwitch = FDCAN_BRS_OFF;
	tx_message.FDFormat = FDCAN_CLASSIC_CAN;
	tx_message.TxFrameType = FDCAN_DATA_FRAME;
	tx_message.MessageMarker = 0;
	tx_message.TxEventFifoControl =FDCAN_NO_TX_EVENTS;
	HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_message, dat);

	if (hcan->Instance->ECR != 0U)
    {
        CLEAR_BIT(hcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
	
	return HAL_OK;
}
/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *  @brief  [__WEAK] 需要在Protocol Layer中实现具体的 CAN1 处理协议
 */
__WEAK void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 *  @brief  [__WEAK] 需要在Protocol Layer中实现具体的 CAN2 处理协议
 */
__WEAK void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 *  @brief  [__WEAK] 需要在Protocol Layer中实现具体的 CAN2 处理协议
 */
__WEAK void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
  * @brief  CAN1滤波器初始化
  * @param  
  * @retval 
  */
void CAN1_Filter_Init(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	/* 开启RX FIFO0的新数据中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);
}

/**
  * @brief  CAN2滤波器初始化
  * @param  
  * @retval 
  */
void CAN2_Filter_Init(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	/* 开启RX FIFO1的新数据中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan2);
}

/**
  * @brief  CAN2滤波器初始化
  * @param  
  * @retval 
  */
void CAN3_Filter_Init(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
	HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	/* 开启RX FIFO1的新数据中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan3);
}

