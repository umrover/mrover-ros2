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
void PostInit();
void Loop();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIMIT_A_Pin GPIO_PIN_13
#define LIMIT_A_GPIO_Port GPIOC
#define LIMIT_B_Pin GPIO_PIN_14
#define LIMIT_B_GPIO_Port GPIOC
#define MOTOR_CURRENT_Pin GPIO_PIN_0
#define MOTOR_CURRENT_GPIO_Port GPIOA
#define LPUART_TX_Pin GPIO_PIN_2
#define LPUART_TX_GPIO_Port GPIOA
#define LPUART_RX_Pin GPIO_PIN_3
#define LPUART_RX_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOB
#define CAN_RX_LED_Pin GPIO_PIN_11
#define CAN_RX_LED_GPIO_Port GPIOB
#define CAN_TX_LED_Pin GPIO_PIN_12
#define CAN_TX_LED_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_13
#define MOTOR_DIR_GPIO_Port GPIOB
#define MOTOR_PWM_Pin GPIO_PIN_8
#define MOTOR_PWM_GPIO_Port GPIOA
#define FDCAN_RX_Pin GPIO_PIN_11
#define FDCAN_RX_GPIO_Port GPIOA
#define FDCAN_TX_Pin GPIO_PIN_12
#define FDCAN_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CAN_STB_Pin GPIO_PIN_5
#define CAN_STB_GPIO_Port GPIOB
#define QUAD_A_Pin GPIO_PIN_6
#define QUAD_A_GPIO_Port GPIOB
#define QUAD_B_Pin GPIO_PIN_7
#define QUAD_B_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
