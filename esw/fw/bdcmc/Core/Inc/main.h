/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LIMIT_0_Pin GPIO_PIN_13
#define LIMIT_0_GPIO_Port GPIOC
#define LIMIT_1_Pin GPIO_PIN_14
#define LIMIT_1_GPIO_Port GPIOC
#define LIMIT_2_Pin GPIO_PIN_15
#define LIMIT_2_GPIO_Port GPIOC
#define LIMIT_3_Pin GPIO_PIN_0
#define LIMIT_3_GPIO_Port GPIOF
#define MOTOR_PWM_Pin GPIO_PIN_14
#define MOTOR_PWM_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_15
#define MOTOR_DIR_GPIO_Port GPIOB
#define QUAD_B_Pin GPIO_PIN_6
#define QUAD_B_GPIO_Port GPIOB
#define QUAD_A_Pin GPIO_PIN_7
#define QUAD_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
