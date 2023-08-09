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
extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim5;

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

/*	Режимы работы генератора	*/
typedef enum {
	PWM_MODE = 0,
	DAC_MODE
} generation_mode_t;

/*	Разрядности в числах	*/
typedef enum {
	POINT_TENS = 1,
	UNITS,
	TENS,
	HUNDREDS,
} digit_position_t;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BUTTON_LEFT_Pin GPIO_PIN_1
#define BUTTON_LEFT_GPIO_Port GPIOA
#define BUTTON_LEFT_EXTI_IRQn EXTI1_IRQn
#define BUTTON_RIGHT_Pin GPIO_PIN_2
#define BUTTON_RIGHT_GPIO_Port GPIOA
#define BUTTON_RIGHT_EXTI_IRQn EXTI2_IRQn
#define BUTTON_OK_Pin GPIO_PIN_3
#define BUTTON_OK_GPIO_Port GPIOA
#define BUTTON_OK_EXTI_IRQn EXTI3_IRQn
#define BUTTON_RETURN_Pin GPIO_PIN_4
#define BUTTON_RETURN_GPIO_Port GPIOA
#define BUTTON_RETURN_EXTI_IRQn EXTI4_IRQn
#define DAC_D0_Pin GPIO_PIN_0
#define DAC_D0_GPIO_Port GPIOB
#define DAC_D1_Pin GPIO_PIN_1
#define DAC_D1_GPIO_Port GPIOB
#define DAC_D2_Pin GPIO_PIN_2
#define DAC_D2_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_8
#define SPI_CS_GPIO_Port GPIOA
#define BUTTON_UP_Pin GPIO_PIN_9
#define BUTTON_UP_GPIO_Port GPIOA
#define BUTTON_UP_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_DOWN_Pin GPIO_PIN_10
#define BUTTON_DOWN_GPIO_Port GPIOA
#define BUTTON_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define DAC_D3_Pin GPIO_PIN_3
#define DAC_D3_GPIO_Port GPIOB
#define DAC_D4_Pin GPIO_PIN_4
#define DAC_D4_GPIO_Port GPIOB
#define DAC_D5_Pin GPIO_PIN_5
#define DAC_D5_GPIO_Port GPIOB
#define DAC_D6_Pin GPIO_PIN_6
#define DAC_D6_GPIO_Port GPIOB
#define DAC_D7_Pin GPIO_PIN_7
#define DAC_D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
