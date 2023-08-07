/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdbool.h>

extern uint32_t counter[3];
extern int pos[3];
extern bool receiveNsendFlag;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_HandleTypeDef fdcan;
uint8_t TxData[8];
uint8_t RxData[26];
int fingerAddress = 0xDB;  //TODO need to check the finger address shit


typedef enum {
    Thumb = 0XD2,
    Index = 0XD3,
    Middle = 0XD4,
    Ring = 0XD5,
    Pinky = 0XD6,
//    left_thumb = 0XD7,  //TODO implement right and left hands
//    left_index  = 0XD8
//    left_middle = 0XD9,
//    left_ring = 0XDA,
//    left_pinky = 0XDB
} FingerAddress;

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan2;

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 2;
  hfdcan2.Init.NominalTimeSeg1 = 17;
  hfdcan2.Init.NominalTimeSeg2 = 6;
  hfdcan2.Init.DataPrescaler = 6;
  hfdcan2.Init.DataSyncJumpWidth = 9;
  hfdcan2.Init.DataTimeSeg1 = 15;
  hfdcan2.Init.DataTimeSeg2 = 9;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void fdcan_init(FDCAN_HandleTypeDef *hfdcan){
	fdcan = *hfdcan;
	TxHeader.Identifier= fingerAddress;
	TxHeader.IdType=FDCAN_STANDARD_ID;
	TxHeader.TxFrameType=FDCAN_DATA_FRAME;
	TxHeader.DataLength=FDCAN_DLC_BYTES_12;
	TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch=FDCAN_BRS_OFF;
	TxHeader.FDFormat=FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker=0;
}

void set_fingerAddress(void)
{
	 uint8_t bit0 =(HAL_GPIO_ReadPin(ID_Bit0_GPIO_Port, ID_Bit0_Pin)==GPIO_PIN_SET);//1 else 0
	 uint8_t bit1 =(HAL_GPIO_ReadPin(ID_Bit0_GPIO_Port, ID_Bit0_Pin)==GPIO_PIN_SET);//1 else 0
	 uint8_t bit2 =(HAL_GPIO_ReadPin(ID_Bit0_GPIO_Port, ID_Bit0_Pin)==GPIO_PIN_SET);//1 else 0
	 uint8_t ID =  (bit2 << 2) | (bit1 << 1) | bit0;
	 switch(ID){
	 case 1:
		 fingerAddress=Pinky;
		 break;
	 case 2:
		 fingerAddress=Ring;
		 break;
	 case 3:
		 fingerAddress=Middle;
		 break;
	 case 4:
		 fingerAddress=Index;
		 break;
	 case 5:
		 fingerAddress=Thumb;
		 break;
	 }
}

void fdcan_process_rx_message(void) {
	uint16_t motor_id = RxHeader.Identifier;
	if (motor_id == fingerAddress) {
		pos[0] = RxData[5];
		pos[1] = RxData[6];
		pos[2] = RxData[7];
		fdcan_transmit_tx_message();
	}
}

void fdcan_transmit_tx_message(void){
	TxData[0] = counter[0]/2048;
	TxData[1] = counter[1]/2048;
	TxData[2] = counter[2]/2048;
	TxData[3] = pos[0];
	TxData[4] = pos[1];
	TxData[5] = pos[2];
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
	receiveNsendFlag = false;
}

/* USER CODE END 1 */
