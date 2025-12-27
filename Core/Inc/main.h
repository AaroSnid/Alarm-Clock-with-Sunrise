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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_1
#define D2_GPIO_Port GPIOC
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOC
#define D4_Pin GPIO_PIN_3
#define D4_GPIO_Port GPIOC
#define RCLK_Pin GPIO_PIN_1
#define RCLK_GPIO_Port GPIOA
#define SRCLK_Pin GPIO_PIN_4
#define SRCLK_GPIO_Port GPIOA
#define SN74HC959_SCK_Pin GPIO_PIN_5
#define SN74HC959_SCK_GPIO_Port GPIOA
#define SN74HC595_MOSI_Pin GPIO_PIN_7
#define SN74HC595_MOSI_GPIO_Port GPIOA
#define SER_Pin GPIO_PIN_0
#define SER_GPIO_Port GPIOB
#define Selector_Pin GPIO_PIN_13
#define Selector_GPIO_Port GPIOB
#define Selector_EXTI_IRQn EXTI15_10_IRQn
#define Decrease_Pin GPIO_PIN_14
#define Decrease_GPIO_Port GPIOB
#define Decrease_EXTI_IRQn EXTI15_10_IRQn
#define Increase_Pin GPIO_PIN_15
#define Increase_GPIO_Port GPIOB
#define Increase_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Alarm_Pin GPIO_PIN_7
#define Alarm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
