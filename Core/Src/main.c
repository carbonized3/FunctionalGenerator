/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osMessageQId myQueue01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void buttonTask(void const * argument);
void displayTask(void const * argument);
void dacTask(void const * argument);
void pwmTask(void const * argument);

/* USER CODE BEGIN PFP */

/*	имеем всевозможных 5 стадий настроек, для режима ЦАПа и ШиМ некоторые из неих будут отличаться.
 * 	По умолчанию все частоты 1 кГц, значит количество цифр на дисплее - 2. ЦАП и ШиМ изначально каждые 100 мс
 * 	проверяют состояние переменной dac_is_running и pwm_is_running.	*/
bool start_stage = true;
bool set_mode_stage = false;
bool set_signal_stage = false;
bool set_freq_stage = false;
bool set_duty_stage = false;
uint8_t digit_position = POINT_TENS;
uint8_t digits_amount = 2;
float freq_dac = 1.0;
float freq_pwm = 1.0;							// Также нужно сохранять digits_amount по уму
signal_t sign = TRIANGLE;
volatile bool dac_is_running = 0;
volatile bool pwm_is_running = 0;
volatile uint16_t button_exti;

/*	Массив строк для удобного их вывода на дислпей на стадии установки сигнала	*/
static char signal_msg[][9] = {
		"TRIANGLE", "SINUS", "SAW", "REV SAW"
};

xQueueHandle xButtonQueue = NULL;
xQueueHandle xDisplayStringQueue = NULL;
xQueueHandle xDisplayFreqQueue = NULL;
xQueueHandle xDisplayDutyQueue = NULL;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	button_exti = GPIO_Pin;
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_NVIC_DisableIRQ(BUTTON_DOWN_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BUTTON_UP_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BUTTON_LEFT_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BUTTON_RIGHT_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BUTTON_RETURN_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(BUTTON_OK_EXTI_IRQn);
	/*	Запустили таймер на отсчёт 100 мс для защиты от дребезга.	*/
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  max7219_init_t cfg = {
  		  .SPI_Handle = &hspi1,
  		  .decode_mode = MAX7219_NO_DECODE,
  		  .indicator_intensity = MAX7219_INTENSITY_25_OF_32,
  		  .digits_quantity = MAX7219_DISPLAY_0_TO_7,
		  .CS_PORT = SPI_CS_GPIO_Port,
  		  .CS_PIN = SPI_CS_Pin
    };
  MAX7219_init(&cfg);
  MAX7219_clearAll();
  MAX7219_sendString("SET MODE");

  xButtonQueue = xQueueCreate(10, sizeof(uint16_t));
  xDisplayStringQueue = xQueueCreate(10, sizeof(char[9]));
  xDisplayFreqQueue = xQueueCreate(10, sizeof(float));
  xDisplayDutyQueue = xQueueCreate(10, sizeof(float));

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 10, 20);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, buttonTask, osPriorityNormal, 0, 1024);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, displayTask, osPriorityBelowNormal, 0, 1024);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, dacTask, osPriorityAboveNormal, 0, 1024);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, pwmTask, osPriorityAboveNormal, 0, 1024);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 639;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 9999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC_D0_Pin|DAC_D1_Pin|DAC_D2_Pin|DAC_D3_Pin
                          |DAC_D4_Pin|DAC_D5_Pin|DAC_D6_Pin|DAC_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_LEFT_Pin BUTTON_RIGHT_Pin BUTTON_OK_Pin BUTTON_RETURN_Pin
                           BUTTON_UP_Pin BUTTON_DOWN_Pin */
  GPIO_InitStruct.Pin = BUTTON_LEFT_Pin|BUTTON_RIGHT_Pin|BUTTON_OK_Pin|BUTTON_RETURN_Pin
                          |BUTTON_UP_Pin|BUTTON_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_D0_Pin DAC_D1_Pin DAC_D2_Pin DAC_D3_Pin
                           DAC_D4_Pin DAC_D5_Pin DAC_D6_Pin DAC_D7_Pin */
  GPIO_InitStruct.Pin = DAC_D0_Pin|DAC_D1_Pin|DAC_D2_Pin|DAC_D3_Pin
                          |DAC_D4_Pin|DAC_D5_Pin|DAC_D6_Pin|DAC_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_buttonTask */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buttonTask */
void buttonTask(void const * argument)
{
  /* USER CODE BEGIN buttonTask */
	uint16_t button_num;
	float freq_buff = 1.0f;
	float duty = 50.0f;
	char str_buff[9] = {};
	uint8_t mode = PWM_MODE;
  /* Infinite loop */
	for(;;)
	{
		button_num = 0;
		xQueueReceive(xButtonQueue, &button_num, 100 / portTICK_RATE_MS);
		switch(button_num)
		{
			case BUTTON_LEFT_Pin:
				if( set_mode_stage )
				{
					sprintf(str_buff, "%s", (mode == PWM_MODE)? "DAC MODE" : "PWM MODE");
					mode = (mode == PWM_MODE)? DAC_MODE : PWM_MODE;
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
				}
				if( set_signal_stage )
				{
					sign++;
					if( sign > REVERSE_SAW )  sign = TRIANGLE;
					sprintf(str_buff, "%s", signal_msg[sign - 1]);
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
				}
				if( set_freq_stage )
				{
					digit_position++;
					if( digit_position > digits_amount ) digit_position = POINT_TENS;
					/*	Если достигли старшего разряда то возвращаемся к младшему	*/
					freq_buff = (mode == PWM_MODE)? freq_pwm : freq_dac;
					xQueueSendToBack(xDisplayFreqQueue, &freq_buff, 50 / portTICK_RATE_MS);
				}
				if( set_duty_stage )
				{
					digit_position++;
					if( digit_position > digits_amount ) digit_position = POINT_TENS;
					xQueueSendToBack(xDisplayDutyQueue, &duty, 50 / portTICK_RATE_MS);
				}
				break;

			case BUTTON_RIGHT_Pin:
				if( set_mode_stage )	// Если мы на выборе режима то мы должны его выбрать
				{
					sprintf(str_buff, "%s", (mode == PWM_MODE)? "DAC MODE" : "PWM MODE");
					mode = (mode == PWM_MODE)? DAC_MODE : PWM_MODE;
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
				}
				if( set_signal_stage )
				{
					sign--;
					if( sign < TRIANGLE )  sign = REVERSE_SAW;
					sprintf(str_buff, "%s", signal_msg[sign - 1]);
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
				}
				if( set_freq_stage )
				{
					digit_position--;
					if( digit_position < POINT_TENS)	digit_position = digits_amount;
					/*	Если мы с самого краю справа, то нужно переместиться на самый старший разряд слева */
					freq_buff = (mode == PWM_MODE)? freq_pwm : freq_dac;
					xQueueSendToBack(xDisplayFreqQueue, &freq_buff, 50 / portTICK_RATE_MS);
				}
				if( set_duty_stage )
				{
					digit_position--;
					if( digit_position < POINT_TENS)	digit_position = digits_amount;
					xQueueSendToBack(xDisplayDutyQueue, &duty, 50 / portTICK_RATE_MS);
				}
				break;

			case BUTTON_UP_Pin:
				if( set_freq_stage )
				{
					if( mode == PWM_MODE)
					{
						freq_pwm += (digit_position == POINT_TENS) ? 0.1 : 0;
						freq_pwm += (digit_position == UNITS) ? 1 : 0;
						freq_pwm += (digit_position == TENS) ? 10 : 0;
						freq_pwm += (digit_position == HUNDREDS) ? 100 : 0;
						/*	В зависимости от того, на каком разряде мы остановились, будем прибавлять величину на
						 	 1 нужного нам разряда, будь то десятые, еидинцы или десятки */
						if( freq_pwm > 9.9 && freq_pwm < 100 )	digits_amount = TENS;
						if( freq_pwm > 99.9 )					digits_amount = HUNDREDS;
						if( freq_pwm > 999.9 )					freq_pwm = 999.9;
						xQueueSendToBack(xDisplayFreqQueue, &freq_pwm, 50 / portTICK_RATE_MS);
					}
					if( mode == DAC_MODE )
					{
						freq_dac += (digit_position == POINT_TENS) ? 0.1 : 0;
						freq_dac += (digit_position == UNITS) ? 1 : 0;
						freq_dac +=(digit_position == TENS) ? 10 : 0;
						if(freq_dac > 9.9)	digits_amount = TENS;
						if( freq_dac > 5 && sign == SIN )							freq_dac = 5.0;
						if( freq_dac > 30 && (sign == SAW || sign == REVERSE_SAW) )	freq_dac = 30.0;
						if( freq_dac > 15 && sign == TRIANGLE )						freq_dac = 15.0;
						xQueueSendToBack(xDisplayFreqQueue, &freq_dac, 50 / portTICK_RATE_MS);
					}
				}
				if( set_duty_stage )
				{
					duty += (digit_position == POINT_TENS) ? 0.1 : 0;
					duty += (digit_position == UNITS) ? 1 : 0;
					duty += (digit_position == TENS) ? 10 : 0;
					if(duty > 9.9)	digits_amount = TENS;
					if(duty > 99.9)	duty = 99.9;
					xQueueSendToBack(xDisplayDutyQueue, &duty, 50 / portTICK_RATE_MS);
				}
				break;

			case BUTTON_DOWN_Pin:
				if( set_freq_stage )
				{
					if( mode == PWM_MODE)
					{
						freq_pwm -= (digit_position == POINT_TENS) ? 0.1 : 0;
						freq_pwm -= (digit_position == UNITS) ? 1 : 0;
						freq_pwm -= (digit_position == TENS) ? 10 : 0;
						freq_pwm -= (digit_position == HUNDREDS) ? 100 : 0;
						if( freq_pwm < 10 )	digits_amount = UNITS;
						if( freq_pwm <= 0.01 )	freq_pwm = 0.1;
						xQueueSendToBack(xDisplayFreqQueue, &freq_pwm, 50 / portTICK_RATE_MS);
					}
					if( mode == DAC_MODE )
					{
						freq_dac -= (digit_position == POINT_TENS) ? 0.1 : 0;
						freq_dac -= (digit_position == UNITS) ? 1 : 0;
						freq_dac -= (digit_position == TENS) ? 10 : 0;
						if( freq_dac < 10 )	digits_amount = UNITS;
						if( freq_dac <= 0.01 )	freq_dac = 0.1;
					}
				}
				if( set_duty_stage )
				{
					duty -= (digit_position == POINT_TENS) ? 0.1 : 0;
					duty -= (digit_position == UNITS) ? 1 : 0;
					duty -= (digit_position == TENS) ? 10 : 0;
					if(duty < 10)	digits_amount = UNITS;
					if(duty <= 0.01 )	duty = 0.1;
					xQueueSendToBack(xDisplayDutyQueue, &duty, 50 / portTICK_RATE_MS);
				}
				break;

			case BUTTON_OK_Pin:
				if( start_stage )
				{
					start_stage = false;
					set_mode_stage = true;
					sprintf(str_buff, "PWM MODE");
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);	//
					continue;
				}
				if( set_mode_stage )
				{
					set_mode_stage = false;
					if( mode == PWM_MODE)
					{
						set_signal_stage = false;
						digits_amount = UNITS;
						set_freq_stage = true;
						sprintf(str_buff, "SET FREQ");
						xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
						vTaskDelay(2000 / portTICK_RATE_MS);
						xQueueSendToBack(xDisplayFreqQueue, &freq_pwm, 50 / portTICK_RATE_MS);
						continue;
					}
					else if( mode == DAC_MODE)
					{
						set_signal_stage = true;
						sprintf(str_buff, "SET SIGN");
						xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
						vTaskDelay(2000 / portTICK_RATE_MS);
						sprintf(str_buff, signal_msg[sign - 1]);
						xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
						continue;
					}
				}
				if( set_signal_stage )
				{
					set_signal_stage = false;
					set_freq_stage = true;
					sprintf(str_buff, "SET FREQ");
					xQueueSendToBack(xDisplayStringQueue, str_buff, 10 / portTICK_RATE_MS);
					vTaskDelay(2000 / portTICK_RATE_MS);
					xQueueSendToBack(xDisplayFreqQueue, &freq_dac, 10 / portTICK_RATE_MS);
					continue;
				}
				if( set_freq_stage )
				{
					if( mode == PWM_MODE )
					{
						digits_amount = TENS;
						sprintf(str_buff, "%s", "SET DUTY");
						xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
						vTaskDelay(2000 / portTICK_RATE_MS);
						xQueueSendToBack(xDisplayDutyQueue, &duty, 50 / portTICK_RATE_MS);
						set_duty_stage = true;
						set_freq_stage = false;
						continue;
					}
					else
					{
						set_duty_stage = false;
						sprintf(str_buff, "%s", "DAC RUN");
						xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
						dac_is_running = true;
					}
					set_freq_stage = false;
				}
				if( set_duty_stage )
				{
					set_duty_stage = false;
					sprintf(str_buff, "%s", "PWM RUN");
					xQueueSendToBack(xDisplayStringQueue, str_buff, 50 / portTICK_RATE_MS);
					pwm_is_running = true;
				}
				break;

			case BUTTON_RETURN_Pin:
				set_mode_stage = false;
				set_signal_stage = false;
				set_duty_stage = false;
				set_freq_stage = false;
				start_stage = true;
				mode = PWM_MODE;
				sprintf(str_buff, "%s", "SET MODE");
				xQueueSendToBack(xDisplayStringQueue,str_buff, 10 / portTICK_RATE_MS);
				break;
		}
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
  /* USER CODE END buttonTask */
}

/* USER CODE BEGIN Header_displayTask */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayTask */
void displayTask(void const * argument)
{
  /* USER CODE BEGIN displayTask */
  /* Infinite loop */
	char str[9] = {};
	uint8_t offset;
	float freq = 0.0f;
	float duty = 0.0f;
	for(;;)
	{
		if( xQueueReceive(xDisplayStringQueue, str, 10 / portTICK_RATE_MS) == pdPASS )
		{
			MAX7219_clearAll();
			MAX7219_sendString(str);
		}
		if( set_duty_stage )
		{
			if( xQueueReceive(xDisplayDutyQueue, &duty, 10 / portTICK_RATE_MS) != pdPASS ) continue;
			MAX7219_clearAll();
			/*	Реализована подсветка нужного разряда при его выборе путем подсвечивания цифры на 0,3 сек	*/
			offset = MAX7219_sendDuty(duty);
			MAX7219_clearOneSegment( offset - (1 + digit_position) );
			vTaskDelay(300 / portTICK_RATE_MS);
			MAX7219_clearAll();
			MAX7219_sendDuty(duty);
		}
		else if( set_freq_stage )
		{
			if( xQueueReceive(xDisplayFreqQueue, &freq, 10 / portTICK_RATE_MS) != pdPASS) continue;
			MAX7219_clearAll();
			offset = MAX7219_sendFreq(freq);
			MAX7219_clearOneSegment( offset - (1 + digit_position) );
			vTaskDelay(300 / portTICK_RATE_MS);
			MAX7219_clearAll();
			MAX7219_sendFreq(freq);
		}
	}
  /* USER CODE END displayTask */
}

/* USER CODE BEGIN Header_dacTask */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dacTask */
void dacTask(void const * argument)
{
  /* USER CODE BEGIN dacTask */
  /* Infinite loop */
	for(;;)
	{
		if( dac_is_running )
		{
			DAC_init(sign, freq_dac);
		}
		while( dac_is_running )
		{
			switch(sign) {
				case TRIANGLE:
					DAC_writeTriangle();
					break;
				case SIN:
					DAC_writeSin();
					break;
				case SAW:
					DAC_writeSaw();
					break;
				case REVERSE_SAW:
					DAC_writeReverseSaw();
					break;
			}
		}
		DAC_stop();
		vTaskDelay(500 / portTICK_RATE_MS);
  }
  /* USER CODE END dacTask */
}

/* USER CODE BEGIN Header_pwmTask */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pwmTask */
void pwmTask(void const * argument)
{
  /* USER CODE BEGIN pwmTask */
  /* Infinite loop */
	for(;;)
	{
		if( pwm_is_running )
		{
			PWM_start(100, 10);
		}
		while( pwm_is_running ) {};
		PWM_stop();
		vTaskDelay(100 / portTICK_RATE_MS);
	}
  /* USER CODE END pwmTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM9)
	{
		HAL_TIM_Base_Stop_IT(&htim9);
		BaseType_t high_task_awoken = 0;
		if( HAL_GPIO_ReadPin(GPIOA, button_exti) == GPIO_PIN_RESET )
		{
			uint16_t button_num = button_exti;
			switch(button_exti)
			{
				case BUTTON_LEFT_Pin:
					button_num = BUTTON_LEFT_Pin;
					break;
				case BUTTON_RIGHT_Pin:
					button_num = BUTTON_RIGHT_Pin;
					break;
				case BUTTON_UP_Pin:
					button_num = BUTTON_UP_Pin;
					break;
				case BUTTON_DOWN_Pin:
					button_num = BUTTON_DOWN_Pin;
					break;
				case BUTTON_OK_Pin:
					button_num = BUTTON_OK_Pin;
					break;
				case BUTTON_RETURN_Pin:
					button_num = BUTTON_RETURN_Pin;
					dac_is_running = false;
					pwm_is_running = false;
					break;
			}
			xQueueSendToBackFromISR(xButtonQueue, &button_num, &high_task_awoken);
		}
		EXTI->PR = (1<<1);
		EXTI->PR = (1<<2);
		EXTI->PR = (1<<3);
		EXTI->PR = (1<<4);
		EXTI->PR = (1<<9);
		EXTI->PR = (1<<15);
		/*	Чистим флаги прерываний которые могли произойти во время задрежки таймером, чтобы не попасть в колбек
		 * ещё лишний раз	*/
		HAL_NVIC_EnableIRQ(BUTTON_DOWN_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(BUTTON_UP_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(BUTTON_LEFT_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(BUTTON_RIGHT_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(BUTTON_RETURN_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(BUTTON_OK_EXTI_IRQn);
		if( high_task_awoken == pdTRUE )
		{
			portYIELD_FROM_ISR(high_task_awoken);
		}
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
