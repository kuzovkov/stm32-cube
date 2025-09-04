/* USER CODE BEGIN Header */
/**
 * Использование таймеров в STM32
Настройка проекта в CubeMX:
1. Создайте новый проект или выполните базовые настройки.
2. Переключите пин PB4 в режим
TIM3_CH1, PB5 в режим TIM3_CH2,  PB3 в TIM2_CH2,

В настройках первого таймера выберите режим для первого канала – «PWM Generation
CH2», и задайте следующие опции:
– Counter Settings:
Prescaler: 5;
Counter Period: 999;
Internal Clock Division: No Division;
auto-reload preload: Enable.
– PWM Generation Channel 1:
Pulse: 200.
7. Для третьего канала укажите «PWM Generation CH3» и следующие опции:
– PWM Generation Channel 2:
Pulse: 800.
8. Аналогично откройте опции третьего таймера («Сategories»→
«Timers» → «TIM3»), выберите для второго канала режим «PWM
Generation CH2» и задайте следующие опции:
– Counter Settings:
Prescaler: 47;
Counter Period: 999;
Internal Clock Division: No Division;
auto-reload preload: Enable.
– PWM Generation Channel 2:
Pulse: 500.
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // local variable with pointer to timer struct
  TIM_HandleTypeDef *tim3 = &htim3;
  TIM_HandleTypeDef *tim4 = &htim4;
  // timer channel
  uint32_t tim_led_channel2 = TIM_CHANNEL_2;
  uint32_t tim_led_channel1 = TIM_CHANNEL_1;

  // timer tick frequency CNT
  uint32_t tim_led_cnt_tick_freq;
  // helper variables for timer parameters calculations
  uint32_t arr_value;
  uint32_t cc_value;
  // reset CCR register
  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, 0);
  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, 0);
  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, 0);
  // run timer
  HAL_TIM_PWM_Start(tim3, tim_led_channel2);
  HAL_TIM_PWM_Start(tim4, tim_led_channel1);
  HAL_TIM_PWM_Start(tim4, tim_led_channel2);
  // calculate timer tick frequency: f_tick = f_sysclcock
  // f_tick = f_sysclcock / (CDK * (PSK + 1));
  tim_led_cnt_tick_freq = SystemCoreClock;
  switch (__HAL_TIM_GET_CLOCKDIVISION(tim3)) {
	  case TIM_CLOCKDIVISION_DIV1:
		  tim_led_cnt_tick_freq /= 1;
		  break;
	  case TIM_CLOCKDIVISION_DIV2:
		  tim_led_cnt_tick_freq /= 2;
		  break;
	  case TIM_CLOCKDIVISION_DIV4:
		  tim_led_cnt_tick_freq /= 4;
		  break;
  }
  tim_led_cnt_tick_freq /= (tim3->Instance->PSC + 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // PWN signal with 0.0 duty cycle (i.e. output is always 0)
	  arr_value = 999;
	  cc_value = 0;
	  __HAL_TIM_SET_AUTORELOAD(tim3, arr_value);
	  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, cc_value);



	  __HAL_TIM_SET_AUTORELOAD(tim4, arr_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, cc_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, cc_value);
	  HAL_Delay(2000);
	  // PWM signal
	  // frequency: 1000 Hz
	  // duty cycle: 0.5
	  arr_value = tim_led_cnt_tick_freq / 1000 - 1;
	  cc_value = (arr_value + 1) * 0.5f;
	  __HAL_TIM_SET_AUTORELOAD(tim3, arr_value);
	  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, cc_value);

	  __HAL_TIM_SET_AUTORELOAD(tim4, arr_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, cc_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, cc_value);


	  HAL_Delay(2000);
	  // PWM signal
	  // frequency: 2000 Hz
	  // duty cycle: 0.2
	  arr_value = tim_led_cnt_tick_freq / 2000 - 1;
	  cc_value = (arr_value + 1) * 0.2f;
	  __HAL_TIM_SET_AUTORELOAD(tim3, arr_value);
	  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, cc_value);
	  __HAL_TIM_SET_AUTORELOAD(tim4, arr_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, cc_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, cc_value);
	  HAL_Delay(2000);
	  // PWM signal
	  // frequency: 2000 Hz
	  // duty cycle: 0.8
	  arr_value = tim_led_cnt_tick_freq / 2000 - 1;
	  cc_value = (arr_value + 1) * 0.8f;
	  __HAL_TIM_SET_AUTORELOAD(tim3, arr_value);
	  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, cc_value);
	  __HAL_TIM_SET_AUTORELOAD(tim4, arr_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, cc_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, cc_value);
	  HAL_Delay(2000);
	  // PWN signal with 1.0 duty cycle (i.e. output is always 1)
	  arr_value = 999;
	  cc_value = 1000;
	  __HAL_TIM_SET_AUTORELOAD(tim3, arr_value);
	  __HAL_TIM_SET_COMPARE(tim3, tim_led_channel2, cc_value);
	  __HAL_TIM_SET_AUTORELOAD(tim4, arr_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel1, cc_value);
	  __HAL_TIM_SET_COMPARE(tim4, tim_led_channel2, cc_value);
	  HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
