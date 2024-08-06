/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define CUR3_Pin GPIO_PIN_0
#define CUR3_GPIO_Port GPIOC
#define SPAN3_Pin GPIO_PIN_1
#define SPAN3_GPIO_Port GPIOC
#define DAT4_OUT_Pin GPIO_PIN_2
#define DAT4_OUT_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_3
#define DIR3_GPIO_Port GPIOC
#define DAT4_Pin GPIO_PIN_0
#define DAT4_GPIO_Port GPIOA
#define DAT5_Pin GPIO_PIN_1
#define DAT5_GPIO_Port GPIOA
#define DAT6_Pin GPIO_PIN_2
#define DAT6_GPIO_Port GPIOA
#define DAT7_Pin GPIO_PIN_3
#define DAT7_GPIO_Port GPIOA
#define A_SPAN_Pin GPIO_PIN_6
#define A_SPAN_GPIO_Port GPIOA
#define SPAN2_Pin GPIO_PIN_7
#define SPAN2_GPIO_Port GPIOA
#define CUR2_Pin GPIO_PIN_4
#define CUR2_GPIO_Port GPIOC
#define PP4_Pin GPIO_PIN_5
#define PP4_GPIO_Port GPIOC
#define SPAN1_Pin GPIO_PIN_0
#define SPAN1_GPIO_Port GPIOB
#define CUR1_Pin GPIO_PIN_1
#define CUR1_GPIO_Port GPIOB
#define ES3_Pin GPIO_PIN_7
#define ES3_GPIO_Port GPIOE
#define ES2_Pin GPIO_PIN_9
#define ES2_GPIO_Port GPIOE
#define DIR1_Pin GPIO_PIN_13
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_14
#define DIR2_GPIO_Port GPIOE
#define RSTSEL_Pin GPIO_PIN_15
#define RSTSEL_GPIO_Port GPIOE
#define DACRST_Pin GPIO_PIN_10
#define DACRST_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_11
#define ENABLE_GPIO_Port GPIOB
#define DAC_SYNC_Pin GPIO_PIN_12
#define DAC_SYNC_GPIO_Port GPIOB
#define DAC_SCK_Pin GPIO_PIN_13
#define DAC_SCK_GPIO_Port GPIOB
#define LDAC_Pin GPIO_PIN_14
#define LDAC_GPIO_Port GPIOB
#define DAC_MOSI_Pin GPIO_PIN_15
#define DAC_MOSI_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define TTL0_Pin GPIO_PIN_10
#define TTL0_GPIO_Port GPIOD
#define TTL1_Pin GPIO_PIN_11
#define TTL1_GPIO_Port GPIOD
#define TTL2_Pin GPIO_PIN_12
#define TTL2_GPIO_Port GPIOD
#define TTL3_Pin GPIO_PIN_13
#define TTL3_GPIO_Port GPIOD
#define TTL4_Pin GPIO_PIN_14
#define TTL4_GPIO_Port GPIOD
#define TTL5_Pin GPIO_PIN_15
#define TTL5_GPIO_Port GPIOD
#define TXD2_Pin GPIO_PIN_6
#define TXD2_GPIO_Port GPIOC
#define RXD2_Pin GPIO_PIN_7
#define RXD2_GPIO_Port GPIOC
#define ES1_Pin GPIO_PIN_8
#define ES1_GPIO_Port GPIOA
#define TXD1_Pin GPIO_PIN_9
#define TXD1_GPIO_Port GPIOA
#define RXD1_Pin GPIO_PIN_10
#define RXD1_GPIO_Port GPIOA
#define SD_DET1_Pin GPIO_PIN_4
#define SD_DET1_GPIO_Port GPIOD
#define TTL6_Pin GPIO_PIN_6
#define TTL6_GPIO_Port GPIOD
#define DAT7_OUT_Pin GPIO_PIN_4
#define DAT7_OUT_GPIO_Port GPIOB
#define DAT6_OUT_Pin GPIO_PIN_5
#define DAT6_OUT_GPIO_Port GPIOB
#define DAT5_OUT_Pin GPIO_PIN_6
#define DAT5_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
