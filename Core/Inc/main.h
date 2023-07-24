/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOC
#define uLED_G_Pin GPIO_PIN_2
#define uLED_G_GPIO_Port GPIOC
#define uLED_R_Pin GPIO_PIN_3
#define uLED_R_GPIO_Port GPIOC
#define ENC2_B_Pin GPIO_PIN_1
#define ENC2_B_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_5
#define ENC2_A_GPIO_Port GPIOA
#define MAX_FAULT_Pin GPIO_PIN_5
#define MAX_FAULT_GPIO_Port GPIOC
#define MAX_TRIGA_Pin GPIO_PIN_0
#define MAX_TRIGA_GPIO_Port GPIOB
#define MAX_TRIGB_Pin GPIO_PIN_1
#define MAX_TRIGB_GPIO_Port GPIOB
#define MAX_EN_Pin GPIO_PIN_2
#define MAX_EN_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOC
#define LR_Bit_Pin GPIO_PIN_9
#define LR_Bit_GPIO_Port GPIOA
#define ID_Bit0_Pin GPIO_PIN_10
#define ID_Bit0_GPIO_Port GPIOA
#define ID_Bit1_Pin GPIO_PIN_11
#define ID_Bit1_GPIO_Port GPIOA
#define ID_Bit2_Pin GPIO_PIN_12
#define ID_Bit2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA
#define SPI_CLK_Pin GPIO_PIN_10
#define SPI_CLK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_12
#define SPI_MOSI_GPIO_Port GPIOC
#define SPI_MAX_CMD_Pin GPIO_PIN_4
#define SPI_MAX_CMD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
