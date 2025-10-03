/* USER CODE BEGIN Header */
/**
 *  пример на HAL/C, который:

    инициализирует ADXL345 в full-resolution ±16g (как у тебя),

    настраивает SPI2 и CS (PB12),

    читает сырой int16_t по трем осям,

    переводит в g и в м/с²,

    отслеживает и держит максимальные пики,

    содержит простую логику детекции удара (порог + защита по времени).
 *
 *
 * Пояснения и практические советы

    Преобразование в g и m/s²

        В full-res используем g = raw * 0.0039.

        m/s² = g * 9.80665.
        Это приближённое значение, но для ударов/пиков достаточно точно.

    Выбор порога IMPACT_THRESHOLD_G

        6g — просто пример. Подбери эмпирически: сначала понаблюдай обычное движение (ходьба/стряхивание) и
        поставь порог выше шума, но ниже реального удара.

        Можно сделать адаптивный порог: усреднять фон и считать отклонение.

    Защита от ложных срабатываний

        Я добавил IMPACT_DEBOUNCE_MS — после удара игнорируем новые в течение N миллисекунд.

        Альтернатива — требовать, чтобы значение превысило порог N последовательных выборок.

    Tap/Activity детекция аппаратно

        ADXL345 умеет аппаратно детектировать single/double tap и activity/inactivity, и выдавать прерывание на INT1/INT2. Это точнее и не нагружает CPU. Если нужно — дам пример настройки регистров THRESH_TAP, DUR, INT_ENABLE и обработку внешнего прерывания.

    Частота выборки (BW_RATE)

        Для ударов полезно повысить частоту (чем выше — тем короче импульс можно захватить). Но это увеличивает поток данных. ADXL345 может давать высокие частоты (см. даташит). Подбери значение, совместимое с твоим SAMPLE_PERIOD_MS.

    Проверка направлений и смещения

        Прибор может иметь смещение (offset). Для точных измерений сделай нулевую калибровку: при неподвижной установке вычисли среднее по нескольким секундам и вычти из измерений.
 *
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "st7789.h"
#include "fonts.h"
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
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define CS_Pin GPIO_PIN_12
#define CS_Port GPIOB

#define SAMPLE_COUNT 256   // количество выборок на удар
#define SAMPLE_RATE_HZ 800 // частота дискретизации

typedef struct {
    float g, a;
} AccelSample;

volatile uint8_t recording = 0;
volatile uint16_t sample_index = 0;
AccelSample samples[SAMPLE_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Функции работы с CS
void ADXL345_CS_Select()   { HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET); }
void ADXL345_CS_Deselect() { HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET); }

// Функции SPI записи/чтения
void ADXL345_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg & 0x3F, data};  // MSB=0 для записи
    ADXL345_CS_Select();
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
    ADXL345_CS_Deselect();
}

uint8_t ADXL345_ReadReg(uint8_t reg) {
    uint8_t tx = 0x80 | (reg & 0x3F); // MSB=1 для чтения
    uint8_t rx = 0;
    ADXL345_CS_Select();
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
    ADXL345_CS_Deselect();
    return rx;
}

void ADXL345_ReadAxes(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t tx[7];
    uint8_t rx[7];
    tx[0] = 0x80 | 0x40 | 0x32; // чтение многобайтно с DATAX0
    for(int i=1; i<7; i++) tx[i]=0x00;

    ADXL345_CS_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, HAL_MAX_DELAY);
    ADXL345_CS_Deselect();

    *x = (int16_t)((rx[2]<<8)|rx[1]);
    *y = (int16_t)((rx[4]<<8)|rx[3]);
    *z = (int16_t)((rx[6]<<8)|rx[5]);
}

uint8_t ADXL_ReadReg(uint8_t reg)
{
    uint8_t tx = reg | 0x80; // R/W=1
    uint8_t rx = 0;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);       // отправили адрес
    HAL_SPI_Receive(&hspi2, &rx, 1, HAL_MAX_DELAY);        // читаем 1 байт
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // CS HIGH

    return rx;
}

void ADXL_TestLoop(void)
{
    char message[128];
	while (1)
    {
        uint8_t int_enable = ADXL_ReadReg(0x2E);
        uint8_t int_map    = ADXL_ReadReg(0x2F);
        uint8_t int_src    = ADXL_ReadReg(0x30);

        sprintf(message, "INT_ENABLE=0x%02X, INT_MAP=0x%02X, INT_SOURCE=0x%02X", int_enable, int_map, int_src);
        HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
        HAL_Delay(500);
    }
}

// Инициализация ADXL345
/* Инициализация ADXL345 */
/* Примечание: здесь устанавливаем full resolution и диапазон ±16g (DATA_FORMAT = 0x0B),
   включаем измерения POWER_CTL = 0x08.
   При желании можно настроить частоту в регистре BW_RATE (0x2C) */
void ADXL345_Init(void) {
    HAL_Delay(10);
    ADXL345_WriteReg(0x2D, 0x00); // сброс POWER_CTL чтобы быть уверенным
    HAL_Delay(5);
    ADXL345_WriteReg(0x31, 0x0B); // DATA_FORMAT: FULL_RES=1, range=11 (±16g)
    HAL_Delay(2);
    // опционально: установить скорость выборки (BW_RATE). По умолчанию 100 Hz.
    ADXL345_WriteReg(0x2C, 0x0D); // пример: установить 100 Hz (см. datasheet для нужного значения)
    ADXL345_WriteReg(0x2D, 0x08); // POWER_CTL: Measure = 1
    HAL_Delay(5);

    // --- Настройка Tap ---
	ADXL345_WriteReg(0x1D, 0x10); // THRESH_TAP ~ 3g (48*62.5mg)
	ADXL345_WriteReg(0x21, 0x10); // DUR ~ 6.25ms
	ADXL345_WriteReg(0x2A, 0x07); // TAP_AXES: X,Y,Z

	// --- Настройка прерываний ---
	ADXL345_WriteReg(0x2E, 0x40); // INT_ENABLE: bit6=SingleTap
	ADXL345_WriteReg(0x2F, 0x00); // INT_MAP: SingleTap → INT1

}

/* Константы преобразования:
   В full-resolution ADXL345 масштаб ~3.9 mg/LSB (часто приводят 0.0039 g/LSB).
   Используем коэффициент 0.0039 (g per LSB). */
#define ADXL_SCALE_G 0.0039f
#define G_TO_MS2 9.80665f

/* Параметры детектора удара */
#define IMPACT_THRESHOLD_G 6.0f     // порог в g (пример: 6g — подбери экспериментально)
#define IMPACT_DEBOUNCE_MS 200      // окно, в течение которого игнорируем новые пики (ms)
#define SAMPLE_PERIOD_MS 10          // период опроса в ms (зависит от BW_RATE)

//Получение ID устройства, должно вывести 0xE5 (проверка устройства)
uint8_t ADXL345_ReadID() {
    uint8_t tx[2] = {0x80 | 0x00, 0x00}; // запрос регистра 0x00
    uint8_t rx[2] = {0};
    ADXL345_CS_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
    ADXL345_CS_Deselect();
    return rx[1]; // второй байт = ответ
}

/* Простая структура для хранения максимумов */
typedef struct {
    float max_g_x;
    float max_g_y;
    float max_g_z;
    uint32_t last_impact_tick; // HAL_GetTick() временная метка
} PeakData;

PeakData peaks = {0};

void process_sample_and_detect(int16_t rx, int16_t ry, int16_t rz)
{
	char message[255];
	char message_display[128];
	// Преобразование raw -> g -> m/s^2
    float gx = (float)rx * ADXL_SCALE_G;
    float gy = (float)ry * ADXL_SCALE_G;
    float gz = (float)rz * ADXL_SCALE_G;
    float abs_gx = fabsf(gx), abs_gy = fabsf(gy), abs_gz = fabsf(gz);
    float abs_g = sqrt(gx*gx + gy*gy + gz*gz);
    float a_ms2 = abs_g * G_TO_MS2;

    // Обновляем пики по осям (в g)
    if (abs_gx > peaks.max_g_x) peaks.max_g_x = abs_gx;
    if (abs_gy > peaks.max_g_y) peaks.max_g_y = abs_gy;
    if (abs_gz > peaks.max_g_z) peaks.max_g_z = abs_gz;

    // Простая детекция удара: если любая ось превысила порог и прошло достаточно времени —
    // регистрируем событие.
    uint32_t now = HAL_GetTick();
	peaks.last_impact_tick = now;
	// Реагируем на удар: печатаем, ставим светодиод, сохраняем в память и т.д.

	sprintf(message, "! IMPACT detected at tick %lu: g=%.2fg m/s2=%.2fg\n",
		   (unsigned long)now, abs_g, a_ms2);
	sprintf(message_display, "%.2f g", abs_g);
	ST7789_WriteString(20, 80, message_display, Font_16x26, GRED, BLACK);
	sprintf(message_display, "%.2fg m/s2", a_ms2);
	ST7789_WriteString(20, 120, message_display, Font_16x26, GREEN, BLACK);
	HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);


    // Вывод текущих величин и максимумов (пример)
//    float ax_ms2 = gx * G_TO_MS2;
//    float ay_ms2 = gy * G_TO_MS2;
//    float az_ms2 = gz * G_TO_MS2;
//    sprintf(message, "g: X=%.2f Y=%.2f Z=%.2f | m/s2: X=%.2f Y=%.2f Z=%.2f | peaks g(X/Y/Z)=%.2f/%.2f/%.2f\r\n",
//           gx, gy, gz, ax_ms2, ay_ms2, az_ms2, peaks.max_g_x, peaks.max_g_y, peaks.max_g_z);
//    HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int16_t rx, ry, rz;
	char message[128];

	strcpy(message, "HAL_GPIO_EXTI_Callback");
	HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
	if (GPIO_Pin == GPIO_PIN_0) {
        // Чтение INT_SOURCE для сброса флага
        uint8_t src = ADXL_ReadReg(0x30);

        if (src & 0x40) { // bit6 = SINGLE_TAP
        	recording = 1;
            sample_index = 0;
        }
    }
}

// --- таймер для дискретизации ---
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    char message[128];
    char message_display[128];

	if (htim->Instance == TIM2) {
        if (recording && sample_index < SAMPLE_COUNT) {
            int16_t x, y, z;
            ADXL345_ReadAxes(&x, &y, &z);
            // Преобразование raw -> g -> m/s^2
			float gx = (float)x * ADXL_SCALE_G;
			float gy = (float)y * ADXL_SCALE_G;
			float gz = (float)z * ADXL_SCALE_G;
			float abs_gx = fabsf(gx), abs_gy = fabsf(gy), abs_gz = fabsf(gz);
			float abs_g = sqrt(gx*gx + gy*gy + gz*gz);
			float a_ms2 = abs_g * G_TO_MS2;

            samples[sample_index].g = abs_g;
            samples[sample_index].a = a_ms2;

            sample_index++;
            if (sample_index >= SAMPLE_COUNT) {
                recording = 0;
                sprintf(message, "Recording finished!\r\n");
                HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
                // выводим в UART для графика
                float g_max, a_max = 0.0;
                for (int i = 0; i < SAMPLE_COUNT; i++) {
                    if (samples[i].g > g_max)
                    	g_max = samples[i].g;
                    if (samples[i].a > a_max)
                    	a_max = samples[i].a;
                }
                sprintf(message, "%.2f g, %.2f m/s2\r\n", g_max, a_max);
                HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
                sprintf(message_display, "%.2f g", abs_g);
				ST7789_WriteString(20, 80, message_display, Font_16x26, GRED, BLACK);
				sprintf(message_display, "%.2fg m/s2", a_ms2);
				ST7789_WriteString(20, 120, message_display, Font_16x26, GREEN, BLACK);
            }
        }
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
  char message[255];
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  ADXL345_Init();
  //init display
  ST7789_Init();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  ST7789_Fill_Color(BLACK);

  // Быстрый тест: прочитать ID
  uint8_t id = ADXL345_ReadID();
  sprintf(message, "ADXL345 ID = 0x%02X", id);
  HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
  ST7789_WriteString(20, 20, message, Font_11x18, GBLUE, BLACK);

  //ADXL_TestLoop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  GPIO_PinState s = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//	  sprintf(message, "INT1 raw: %d\r\n", (GPIO_PIN_SET));
//	  HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
//	  HAL_Delay(200);
	  //HAL_Delay(500);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
