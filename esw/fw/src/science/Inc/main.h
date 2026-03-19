/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
void PostInit(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UV_Sensor_Pin GPIO_PIN_0
#define UV_Sensor_GPIO_Port GPIOA
#define Debug_LED1_Pin GPIO_PIN_5
#define Debug_LED1_GPIO_Port GPIOA
#define Debug_LED2_Pin GPIO_PIN_6
#define Debug_LED2_GPIO_Port GPIOA
#define Debug_LED3_Pin GPIO_PIN_7
#define Debug_LED3_GPIO_Port GPIOA
#define CAN_RX_LED_Pin GPIO_PIN_11
#define CAN_RX_LED_GPIO_Port GPIOB
#define CAN_TX_LED_Pin GPIO_PIN_12
#define CAN_TX_LED_GPIO_Port GPIOB
#define CAN_Standby_Pin GPIO_PIN_10
#define CAN_Standby_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
