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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MAX7219.h"
#include "DAC_soft.h"
#include "PWM.h"
#include "FreeRTOS.h"
#include "math.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;

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

typedef enum {					// тайпдеф режимов
	PWM_MODE = 0,
	DAC_MODE
} generation_mode_t;

typedef enum {					// Тайпдеф для удобной работы с разрядами
	POINT_TENS = 1,
	UNITS,
	TENS,
	HUNDREDS,
	THOUSANDS
} digit_position_t;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define DAC_D0_Pin GPIO_PIN_1
#define DAC_D0_GPIO_Port GPIOA
#define DAC_D1_Pin GPIO_PIN_2
#define DAC_D1_GPIO_Port GPIOA
#define DAC_D2_Pin GPIO_PIN_3
#define DAC_D2_GPIO_Port GPIOA
#define DAC_D3_Pin GPIO_PIN_4
#define DAC_D3_GPIO_Port GPIOA
#define DAC_D4_Pin GPIO_PIN_5
#define DAC_D4_GPIO_Port GPIOA
#define DAC_D5_Pin GPIO_PIN_6
#define DAC_D5_GPIO_Port GPIOA
#define DAC_D6_Pin GPIO_PIN_7
#define DAC_D6_GPIO_Port GPIOA
#define BUTTON_LEFT_Pin GPIO_PIN_1
#define BUTTON_LEFT_GPIO_Port GPIOB
#define BUTTON_LEFT_EXTI_IRQn EXTI1_IRQn
#define BUTTON_UP_Pin GPIO_PIN_10
#define BUTTON_UP_GPIO_Port GPIOB
#define BUTTON_UP_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_DOWN_Pin GPIO_PIN_12
#define BUTTON_DOWN_GPIO_Port GPIOB
#define BUTTON_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_OK_Pin GPIO_PIN_13
#define BUTTON_OK_GPIO_Port GPIOB
#define BUTTON_OK_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_RETURN_Pin GPIO_PIN_14
#define BUTTON_RETURN_GPIO_Port GPIOB
#define BUTTON_RETURN_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_RIGHT_Pin GPIO_PIN_15
#define BUTTON_RIGHT_GPIO_Port GPIOB
#define BUTTON_RIGHT_EXTI_IRQn EXTI15_10_IRQn
#define DAC_D7_Pin GPIO_PIN_8
#define DAC_D7_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_6
#define SPI_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
