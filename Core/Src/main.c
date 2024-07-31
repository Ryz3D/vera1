/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "fir_taps8_i.h"
#include "fir.h"
#include "sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define F_TEST 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
FIR_t hfir_pz[PIEZO_COUNT];

volatile uint8_t capture_running = 0;
volatile uint32_t ticks_counter = 0;

volatile uint16_t pz_dma_buffer[PIEZO_CHANNEL_COUNT * OVERSAMPLING_RATIO];

// TODO: are these volatile???
volatile a_data_point_t a_buffer_1[A_BUFFER_LEN];
volatile a_data_point_t a_buffer_2[A_BUFFER_LEN];
volatile a_data_point_t *a_buffer_current = a_buffer_1;
volatile p_data_point_t p_buffer_1[P_BUFFER_LEN];
volatile p_data_point_t p_buffer_2[P_BUFFER_LEN];
volatile p_data_point_t *p_buffer_current = p_buffer_1;

volatile uint32_t a_write_index = 0;
volatile uint32_t p_write_index = 0;

volatile uint8_t flag_save_a_buffer_1 = 0;
volatile uint8_t flag_save_a_buffer_2 = 0;
volatile uint8_t flag_overflow_a_buffer = 0;
volatile uint8_t flag_save_p_buffer_1 = 0;
volatile uint8_t flag_save_p_buffer_2 = 0;
volatile uint8_t flag_overflow_p_buffer = 0;

volatile uint8_t flag_complete_a_mems = 0;
volatile uint8_t flag_complete_a_pz = 0;
volatile uint8_t flag_complete_p_gps = 0;

const char *str_sd_log_error = "ERROR: sd_flush_log failed\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void switch_buffers(void **buffer_current, void *buffer_1, void *buffer_2, volatile uint8_t *flag_save_buffer_1, volatile uint8_t *flag_save_buffer_2, volatile uint8_t *flag_overflow);
void a_buffer_write_inc(void);
void p_buffer_write_inc(void);
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
	MX_DMA_Init();
	MX_SDMMC1_SD_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1);

	printf("(%lu) Booting...\r\n", HAL_GetTick());
	printf("(%lu) Config: ", HAL_GetTick());
	printf("OVERSAMPLING_RATIO=%i ", OVERSAMPLING_RATIO);
	printf("PIEZO_COUNT=%i ", PIEZO_COUNT);
	printf("A_BUFFER_LEN=%i ", A_BUFFER_LEN);
	printf("P_BUFFER_LEN=%i\r\n", P_BUFFER_LEN);

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	a_buffer_1[0].timestamp = ticks_counter;
	p_buffer_1[0].timestamp = ticks_counter;

	uint8_t do_format_sd = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET;
	if (do_format_sd)
	{
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}
	if (SD_Init(do_format_sd) != HAL_OK)
	{
		Error_Handler();
	}

	// Start FIR filter
	for (uint8_t i_ch = 0; i_ch < PIEZO_COUNT; i_ch++)
	{
		FIR_Init(&hfir_pz[i_ch], fir_taps);
	}

#if F_TEST == 1
	float freqs[400];
	float amps[400];
	float delays[400];

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		freqs[f_i] = 10 + f_i * 10;
		amps[f_i] = 0;
		delays[f_i] = 0;
	}

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		uint32_t sim_t = 0;
		float a_max = 0;
		uint8_t delay_set = 0;
		for (uint16_t iter = 0; iter < 400; iter++)
		{
			for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
			{
				hfir_pz[0].In[i] = arm_sin_f32(2.0f * PI * freqs[f_i] / 16000.0f * (float)sim_t);
				sim_t++;
			}
			FIR_Update(&hfir_pz[0]);
			for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
			{
				a_max = hfir_pz[0].Out[i] > a_max ? hfir_pz[0].Out[i] : a_max;
				if (a_max > 0.1 && delay_set == 0)
				{
					delays[f_i] = (float)sim_t / 16000.0f;
					delay_set = 1;
				}
			}
		}
		amps[f_i] = a_max;
		for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
		{
			hfir_pz[0].In[i] = 0;
		}
		for (uint8_t j = 0; j < 10; j++)
		{
			FIR_Update(&hfir_pz[0]);
		}
	}

	/*
	 printf("%f", hfir_pz[0].Out[i]);
	 int16_t k;
	 if (hfir_pz[0].Out[i] < -1.0f)
	 hfir_pz[0].Out[i] = -1.0f;
	 else if (hfir_pz[0].Out[i] > -1.0f)
	 hfir_pz[0].Out[i] = 1.0f;
	 k = (int16_t)(hfir_pz[0].Out[i] * 0xFFF); // 13 bit?, 24 bit -> 0x7FFFFF (do this and bit shift?), 12 bit -> 0x7FF
	 printf(" (%hi)\r\n", k);
	 */

	printf("[");
	for (uint16_t i = 0; i < 400; i++)
	{
		printf("(%hu, %.4f)", (uint16_t)freqs[i], amps[i]);
		if (i < 399)
			printf(",");
	}
	printf("]\r\n\r\n");

	printf("[");
	for (uint16_t i = 0; i < 400; i++)
	{
		printf("(%hu, %.4f)", (uint16_t)freqs[i], delays[i]);
		if (i < 399)
			printf(",");
	}
	printf("]\r\n\r\n");
#endif

	// Start piezo ADC
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pz_dma_buffer, PIEZO_CHANNEL_COUNT * OVERSAMPLING_RATIO) != HAL_OK)
	{
		printf("(%lu) ERROR: main: ADC1 HAL_ADC_Start_DMA failed\r\n", HAL_GetTick());
		Error_Handler();
	}
	// Start oversampling timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		printf("(%lu) ERROR: main: HAL_TIM_Base_Start_IT failed\r\n", HAL_GetTick());
		Error_Handler();
	}
	// Start regular piezo timer
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		printf("(%lu) ERROR: main: HAL_TIM_Base_Start_IT failed\r\n", HAL_GetTick());
		Error_Handler();
	}
	// Start GPS timer
	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
	{
		printf("(%lu) ERROR: main: HAL_TIM_Base_Start_IT failed\r\n", HAL_GetTick());
		Error_Handler();
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	capture_running = 1;
	printf("(%lu) Capture started\r\n", HAL_GetTick());
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (capture_running)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// If a_buffer_1 is ready to save
		if (flag_save_a_buffer_1)
		{
			DEBUG_A_BUFFER_1_SD
#if F_TEST == 2
			int16_t min = 5000, max = -5000;
			for (uint16_t i = 0; i < A_BUFFER_LEN; i++)
			{
				min = (int16_t)a_buffer_1[i].a_piezo[1] < min ? (int16_t)a_buffer_1[i].a_piezo[1] : min;
				max = (int16_t)a_buffer_1[i].a_piezo[1] > max ? (int16_t)a_buffer_1[i].a_piezo[1] : max;
			}
			printf("A_PZ = %.3f V    (offset %.3f V)\r\n", (max - min) / 4095.0f * 3.3f, (min + max) / 4095.0f * 1.65f);
#endif
			if (SD_Save_a_data(a_buffer_1, sizeof(a_buffer_1)) != HAL_OK)
			{
				Error_Handler();
			}
			// Flag a_buffer_1 as saved
			flag_save_a_buffer_1 = 0;
			DEBUG_A_BUFFER_1_SD
		}
		// If a_buffer_2 is ready to save
		if (flag_save_a_buffer_2)
		{
			DEBUG_A_BUFFER_2_SD
#if F_TEST == 2
			int16_t min = 5000, max = -5000;
			for (uint16_t i = 0; i < A_BUFFER_LEN; i++)
			{
				min = (int16_t)a_buffer_2[i].a_piezo[1] < min ? (int16_t)a_buffer_2[i].a_piezo[1] : min;
				max = (int16_t)a_buffer_2[i].a_piezo[1] > max ? (int16_t)a_buffer_2[i].a_piezo[1] : max;
			}
			printf("A_PZ = %.3f V    (offset %.3f V)\r\n", (max - min) / 4095.0f * 3.3f, (min + max) / 4095.0f * 1.65f);
#endif
			if (SD_Save_a_data(a_buffer_2, sizeof(a_buffer_2)) != HAL_OK)
			{
				Error_Handler();
			}
			// Flag a_buffer_2 as saved
			flag_save_a_buffer_2 = 0;
			DEBUG_A_BUFFER_2_SD
		}
		if (flag_overflow_a_buffer)
		{
			printf("(%lu) WARNING: main: a_buffer overflow\r\n", HAL_GetTick());
		}

		// If p_buffer_1 is ready to save
		if (flag_save_p_buffer_1)
		{
			DEBUG_P_BUFFER_1_SD
			if (SD_Save_p_data(p_buffer_1, sizeof(p_buffer_1)) != HAL_OK)
			{
				Error_Handler();
			}
			// Flag p_buffer_1 as saved
			flag_save_p_buffer_1 = 0;
			DEBUG_P_BUFFER_1_SD
		}
		// If p_buffer_2 is ready to save
		if (flag_save_p_buffer_2)
		{
			DEBUG_P_BUFFER_2_SD
			if (SD_Save_p_data(p_buffer_2, sizeof(p_buffer_2)) != HAL_OK)
			{
				Error_Handler();
			}
			// Flag p_buffer_2 as saved
			flag_save_p_buffer_2 = 0;
			DEBUG_P_BUFFER_2_SD
		}
		if (flag_overflow_p_buffer)
		{
			printf("(%lu) WARNING: main: p_buffer overflow\r\n", HAL_GetTick());
		}

		if (SD_FlushLog() != HAL_OK)
		{
			HAL_UART_Transmit(&huart3, (const uint8_t*)str_sd_log_error, strlen(str_sd_log_error), 1000);
		}

		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		{
			capture_running = 0;
			break;
		}
	}

	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_ADC_Stop_IT(&hadc1);

	// Save remaining data
	if (SD_Save_a_data(a_buffer_current, a_write_index * sizeof(a_data_point_t)) != HAL_OK)
	{
		Error_Handler();
	}
	if (SD_Save_p_data(p_buffer_current, p_write_index * sizeof(p_data_point_t)) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	printf("(%lu) Capture stopped\r\n", HAL_GetTick());

	if (SD_FlushLog() != HAL_OK)
	{
		HAL_UART_Transmit(&huart3, (const uint8_t*)str_sd_log_error, strlen(str_sd_log_error), 1000);
	}

	SD_Uninit();

	while (1)
		;
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_SD_Init(void)
{

	/* USER CODE BEGIN SDMMC1_Init 0 */

	/* USER CODE END SDMMC1_Init 0 */

	/* USER CODE BEGIN SDMMC1_Init 1 */

	/* USER CODE END SDMMC1_Init 1 */
	hsd1.Instance = SDMMC1;
	hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDMMC1_Init 2 */

	/* USER CODE END SDMMC1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 6750 - 1;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 400 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Debug2_GPIO_Port, Debug2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Debug2_Pin */
	GPIO_InitStruct.Pin = Debug2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Debug2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Debug1_Pin */
	GPIO_InitStruct.Pin = Debug1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Debug1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : uSD_Detect_Pin USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin | USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_DRDY_Pin MEMS_INT2_Pin MEMS_INT1_Pin */
	GPIO_InitStruct.Pin = MEMS_DRDY_Pin | MEMS_INT2_Pin | MEMS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD0_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 *   None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	if (sd_log_write_index < SD_LOG_LEN)
	{
		sd_log[sd_log_write_index++] = (char)ch;
	}

	return ch;
}

void switch_buffers(void **buffer_current, void *buffer_1, void *buffer_2, volatile uint8_t *flag_save_buffer_1, volatile uint8_t *flag_save_buffer_2, volatile uint8_t *flag_overflow)
{
	// If buffer_2 has been filled
	if (*buffer_current == buffer_2)
	{
		// If buffer_1 has been emptied
		if (!*flag_save_buffer_1)
		{
			// buffer_2 full
			*flag_save_buffer_2 = 1;
			// Reset buffer_1 to use
			*buffer_current = buffer_1;
		}
		else
		{
			// Both buffers are full, flag as overflow
			*flag_overflow = 1;
		}
	}
	// If buffer_1 has been filled
	else
	{
		// If buffer_2 has been emptied
		if (!*flag_save_buffer_2)
		{
			// buffer_1 full
			*flag_save_buffer_1 = 1;
			// Reset buffer_2 to use
			*buffer_current = buffer_2;
		}
		else
		{
			// Both buffers are full, flag as overflow
			*flag_overflow = 1;
		}
	}
}

void a_buffer_write_inc()
{
	// Set complete bits of last data point
	a_buffer_current[a_write_index].complete |= (1 << A_COMPLETE_TIMESTAMP) | (flag_complete_a_mems << A_COMPLETE_MEMS) | (flag_complete_a_pz << A_COMPLETE_PZ);

	// If next index would overflow
	if (++a_write_index >= A_BUFFER_LEN)
	{
		// Switch buffers
		a_write_index = 0;
		switch_buffers((void**)&a_buffer_current, (void*)a_buffer_1, (void*)a_buffer_2, &flag_save_a_buffer_1, &flag_save_a_buffer_2, &flag_overflow_a_buffer);
	}

	a_buffer_current[a_write_index].timestamp = ticks_counter;
	a_buffer_current[a_write_index].x_mems1 = a_write_index;
}

void p_buffer_write_inc()
{
	// Set complete bits of last data point
	p_buffer_current[p_write_index].complete |= (1 << P_COMPLETE_TIMESTAMP) | (flag_complete_p_gps << P_COMPLETE_GPS);

	// If next index would overflow
	if (++p_write_index >= P_BUFFER_LEN)
	{
		// Switch buffers
		p_write_index = 0;
		switch_buffers((void**)&p_buffer_current, (void*)p_buffer_1, (void*)p_buffer_2, &flag_save_p_buffer_1, &flag_save_p_buffer_2, &flag_overflow_p_buffer);
	}

	p_buffer_current[p_write_index].timestamp = ticks_counter;
	p_buffer_current[p_write_index].gps_time = p_write_index;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		DEBUG_A_TIMER

		if (capture_running && ticks_counter > 0)
		{
			// Increment data point index
			a_buffer_write_inc();
			// TODO: ADXL sync
		}
		ticks_counter++;

		DEBUG_A_TIMER
	}
	else if (htim->Instance == TIM4)
	{
		DEBUG_P_TIMER

		flag_complete_p_gps = 1;
		p_buffer_write_inc();

		DEBUG_P_TIMER
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (capture_running)
	{
		// ADC for piezos
		if (hadc->Instance == ADC1)
		{
			DEBUG_ADC_PZ_CONV

			for (uint8_t i_sample = 0; i_sample < OVERSAMPLING_RATIO; i_sample++)
			{
				for (uint8_t i_ch = 0; i_ch < PIEZO_COUNT; i_ch++)
				{
					hfir_pz[i_ch].In[i_sample] = (int16_t)(pz_dma_buffer[i_sample * PIEZO_CHANNEL_COUNT + i_ch] << 1) - 4094;
				}
			}
			for (uint8_t i_ch = 0; i_ch < PIEZO_COUNT; i_ch++)
			{
				FIR_Update(&hfir_pz[i_ch]);
			}
			uint8_t sample_delay = 3;
			a_buffer_current[a_write_index].a_piezo[0] = pz_dma_buffer[0];
			a_buffer_current[a_write_index].a_piezo[1] = pz_dma_buffer[1];
			a_buffer_current[a_write_index].a_piezo[2] = hfir_pz[0].Out[sample_delay];
			a_buffer_current[a_write_index].a_piezo[3] = hfir_pz[1].Out[sample_delay];
			flag_complete_a_pz = 1;

			DEBUG_ADC_PZ_CONV
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	printf("(%lu) Fatal Error\r\n", HAL_GetTick());
	SD_FlushLog();
	SD_Uninit();
	__disable_irq();
	while (1)
		;
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
