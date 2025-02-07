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
#define ARM_LASER_Pin GPIO_PIN_14
#define ARM_LASER_GPIO_Port GPIOC
#define CURR_24V_Pin GPIO_PIN_0
#define CURR_24V_GPIO_Port GPIOF
#define TEMP_24V_Pin GPIO_PIN_1
#define TEMP_24V_GPIO_Port GPIOF
#define CURR_18V_Pin GPIO_PIN_0
#define CURR_18V_GPIO_Port GPIOA
#define TEMP_18V_Pin GPIO_PIN_1
#define TEMP_18V_GPIO_Port GPIOA
#define CURR_12V_Pin GPIO_PIN_3
#define CURR_12V_GPIO_Port GPIOA
#define TEMP_12V_Pin GPIO_PIN_4
#define TEMP_12V_GPIO_Port GPIOA
#define TEMP_12V_SPLIT_Pin GPIO_PIN_7
#define TEMP_12V_SPLIT_GPIO_Port GPIOA
#define CURR_12V_SPLIT_Pin GPIO_PIN_0
#define CURR_12V_SPLIT_GPIO_Port GPIOB
#define CURR_7V_Pin GPIO_PIN_1
#define CURR_7V_GPIO_Port GPIOB
#define TEMP_7V_Pin GPIO_PIN_2
#define TEMP_7V_GPIO_Port GPIOB
#define TEMP_5V_Pin GPIO_PIN_11
#define TEMP_5V_GPIO_Port GPIOB
#define CURR_5V_Pin GPIO_PIN_12
#define CURR_5V_GPIO_Port GPIOB
#define CURR_3V3_Pin GPIO_PIN_14
#define CURR_3V3_GPIO_Port GPIOB
#define TEMP_3V3_Pin GPIO_PIN_15
#define TEMP_3V3_GPIO_Port GPIOB
#define CAN_RX_LED_Pin GPIO_PIN_8
#define CAN_RX_LED_GPIO_Port GPIOA
#define CAN_TX_LED_Pin GPIO_PIN_9
#define CAN_TX_LED_GPIO_Port GPIOA
#define CAN_STANDBY_Pin GPIO_PIN_10
#define CAN_STANDBY_GPIO_Port GPIOA
#define AUTON_LED_B_Pin GPIO_PIN_5
#define AUTON_LED_B_GPIO_Port GPIOB
#define AUTON_LED_G_Pin GPIO_PIN_6
#define AUTON_LED_G_GPIO_Port GPIOB
#define AUTON_LED_R_Pin GPIO_PIN_7
#define AUTON_LED_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
