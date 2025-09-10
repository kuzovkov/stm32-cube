/* USER CODE BEGIN Header */
/**
 *
 * https://github.com/Majid-Derhambakhsh/ST7789/

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

#include "st7789.h"
#include "st7789_font.h"
#include "stdlib.h"
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
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
void ST7789_DrawBitmap(uint16_t, uint16_t, uint16_t, uint16_t, const uint16_t*);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint16_t smiley_bitmap[16*16] = {
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFE0,0xFFE0,0xFFFF,0xFFFF,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFE0,0xFFFF,0xFFE0,0xFFE0,0xFFFF,
    0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF
};

void test_display()
{
	// Сброс дисплея
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //BLK
	// Sleep Out
	uint8_t cmd = 0x11;
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
	HAL_Delay(120);

	// Display ON
	cmd = 0x29;
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

	// Заливка красным (очень упрощённо)
	cmd = 0x2C; // Memory Write
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET); // Data
	for (int i=0; i<240*240; i++) {
	  uint8_t color[2] = {0xF8, 0x00}; // красный (RGB565)
	  HAL_SPI_Transmit(&hspi1, color, 2, HAL_MAX_DELAY);
	}
}

void ST7789_Demo(void)
{
    // 1. Тест заливки цветами
    ST7789_FillScreen(ST7789_COLOR_RED);
    HAL_Delay(500);
    ST7789_FillScreen(ST7789_COLOR_GREEN);
    HAL_Delay(500);
    ST7789_FillScreen(ST7789_COLOR_BLUE);
    HAL_Delay(500);
    ST7789_FillScreen(ST7789_COLOR_BLACK);
    HAL_Delay(500);
    ST7789_FillScreen(ST7789_COLOR_WHITE);
    HAL_Delay(500);
    ST7789_FillScreen(ST7789_COLOR_BLACK);

    // 2. Текст
    ST7789_PutString(10, 10, "ST7789 Demo", Font_16x26, ST7789_COLOR_YELLOW, ST7789_COLOR_BLACK);
    HAL_Delay(1000);

    // 3. Геометрические фигуры
    ST7789_DrawRectangle(20, 60, 100, 100, ST7789_COLOR_RED);        // прямоугольник
    ST7789_DrawFilledRectangle(130, 60, 210, 100, ST7789_COLOR_BLUE);// залитый прямоугольник
    HAL_Delay(1000);

    ST7789_DrawCircle(60, 160, 40, ST7789_COLOR_GREEN);             // окружность
    ST7789_DrawFilledCircle(180, 160, 30, ST7789_COLOR_MAGENTA);    // залитая окружность
    HAL_Delay(1000);

    // 4. Линии
    ST7789_DrawLine(0, 0, 239, 239, ST7789_COLOR_CYAN);   // диагональ
    ST7789_DrawLine(0, 239, 239, 0, ST7789_COLOR_ORANGE); // другая диагональ
    HAL_Delay(1000);

    // 5. "Шахматка"
    for (uint16_t y = 0; y < 240; y += 30)
    {
        for (uint16_t x = 0; x < 240; x += 30)
        {
            if (((x + y) / 30) % 2 == 0)
                ST7789_DrawFilledRectangle(x, y, x + 29, y + 29, ST7789_COLOR_WHITE);
            else
                ST7789_DrawFilledRectangle(x, y, x + 29, y + 29, ST7789_COLOR_BLACK);
        }
    }
    HAL_Delay(2000);
    ST7789_FillScreen(ST7789_COLOR_BLACK);
    ST7789_DrawBitmap(112, 112, 16, 16, smiley_bitmap);
    HAL_Delay(3000);


    // 6. Возврат к чёрному экрану
    ST7789_FillScreen(ST7789_COLOR_BLACK);
    ST7789_PutString(30, 100, "Demo Done!", Font_16x26, ST7789_COLOR_GREEN, ST7789_COLOR_BLACK);
}

void ST7789_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *bitmap)
{
	ST7789_SetWindowAddress(x, y, x + w - 1, y + h - 1);

    // отправляем массив как данные
    for (uint32_t i = 0; i < w * h; i++)
    {
        uint8_t data[2];
        data[0] = bitmap[i] >> 8;      // старший байт
        data[1] = bitmap[i] & 0xFF;    // младший байт
        ST7789_TransmitData(data, 2);
    }
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //test_display();
  //HAL_Delay(5000);
  /* USER CODE END 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //BLK (включаем подсветку)
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ST7789_Init();

  //ST7789_FillScreen(ST7789_COLOR_BLACK);
  //ST7789_PutString(10, 10, "Hello", Font_16x26, ST7789_COLOR_RED, ST7789_COLOR_BLACK);

  ST7789_Demo();

  while (1)
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3); //экран мигает
    //HAL_Delay(2000);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
