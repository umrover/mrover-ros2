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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void HAL_PostInit();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_14
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC32_OUTC15_Pin GPIO_PIN_15
#define RCC_OSC32_OUTC15_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define QUAD_2_A_Pin GPIO_PIN_0
#define QUAD_2_A_GPIO_Port GPIOA
#define QUAD_2_B_Pin GPIO_PIN_1
#define QUAD_2_B_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define QUAD_1_B_Pin GPIO_PIN_4
#define QUAD_1_B_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define QUAD_1_A_Pin GPIO_PIN_6
#define QUAD_1_A_GPIO_Port GPIOA
#define LIMIT_2_A_Pin GPIO_PIN_1
#define LIMIT_2_A_GPIO_Port GPIOB
#define LIMIT_2_B_Pin GPIO_PIN_2
#define LIMIT_2_B_GPIO_Port GPIOB
#define CAN_RX_LED_Pin GPIO_PIN_11
#define CAN_RX_LED_GPIO_Port GPIOB
#define CAN_TX_LED_Pin GPIO_PIN_12
#define CAN_TX_LED_GPIO_Port GPIOB
#define LIMIT_0_B_Pin GPIO_PIN_13
#define LIMIT_0_B_GPIO_Port GPIOB
#define LIMIT_0_A_Pin GPIO_PIN_14
#define LIMIT_0_A_GPIO_Port GPIOB
#define MOTOR_DIR_0_Pin GPIO_PIN_6
#define MOTOR_DIR_0_GPIO_Port GPIOC
#define MOTOR_DIR_1_Pin GPIO_PIN_8
#define MOTOR_DIR_1_GPIO_Port GPIOC
#define MOTOR_DIR_2_Pin GPIO_PIN_9
#define MOTOR_DIR_2_GPIO_Port GPIOC
#define MOTOR_PWM_0_Pin GPIO_PIN_8
#define MOTOR_PWM_0_GPIO_Port GPIOA
#define MOTOR_PWM_1_Pin GPIO_PIN_9
#define MOTOR_PWM_1_GPIO_Port GPIOA
#define MOTOR_PWM_2_Pin GPIO_PIN_10
#define MOTOR_PWM_2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define LIMIT_1_B_Pin GPIO_PIN_10
#define LIMIT_1_B_GPIO_Port GPIOC
#define LIMIT_1_A_Pin GPIO_PIN_12
#define LIMIT_1_A_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define QUAD_0_A_Pin GPIO_PIN_6
#define QUAD_0_A_GPIO_Port GPIOB
#define QUAD_0_B_Pin GPIO_PIN_7
#define QUAD_0_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
