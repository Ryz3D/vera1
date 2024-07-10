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
#include "sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define BUFFER_FLAG_BUFFER_1_FULL (1 << 0)
#define BUFFER_FLAG_BUFFER_1_EMPTIED (1 << 1)
#define BUFFER_FLAG_BUFFER_2_FULL (1 << 2)
#define BUFFER_FLAG_BUFFER_2_EMPTIED (1 << 3)

#define A_BUFFER_LEN 4096
#define P_BUFFER_LEN 128

#define DO_FORMAT 1

#define DEBUG1_A_BUFFER_1_SD 0
#define DEBUG2_A_BUFFER_2_SD 0
#define DEBUG1_ADC1_CONV 1
#define DEBUG2_ADC2_CONV 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t capture_running = 1;

a_data_point_t a_buffer_1[A_BUFFER_LEN];
a_data_point_t a_buffer_2[A_BUFFER_LEN];
p_data_point_t p_buffer_1[P_BUFFER_LEN];
p_data_point_t p_buffer_2[P_BUFFER_LEN];
uint8_t a_buffer_flags = BUFFER_FLAG_BUFFER_2_EMPTIED;
uint8_t p_buffer_flags = BUFFER_FLAG_BUFFER_2_EMPTIED;
uint32_t a_write_index = 0;
uint32_t p_write_index = 0;

uint32_t test = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
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
	MX_ADC1_Init();
	MX_SDMMC1_SD_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1);

	printf("\r\n(%lu) Booting...\r\n", HAL_GetTick());

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	a_buffer_1[0].complete = 0;
	p_buffer_1[0].complete = 0;

	if (sd_init(DO_FORMAT) != HAL_OK)
	{
		Error_Handler();
	}

	// ADC
	if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
	{
		printf("ADC1 HAL_ADC_Start_IT failed\r\n");
		Error_Handler();
	}
	if (HAL_ADC_Start_IT(&hadc2) != HAL_OK)
	{
		printf("ADC2 HAL_ADC_Start_IT failed\r\n");
		Error_Handler();
	}

	// Data capture timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		printf("HAL_TIM_Base_Start_IT failed\r\n");
		Error_Handler();
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	printf("(%lu) Capture started\r\n", HAL_GetTick());
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (capture_running)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*
		 uint8_t buffer_1_full = a_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL;
		 uint8_t buffer_1_emptied = a_buffer_flags & BUFFER_FLAG_BUFFER_1_EMPTIED;
		 if (buffer_1_full != 0 && buffer_1_emptied == 0)
		 */
		// If a_buffer_1 is full and has not been saved
		if ((a_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) && !(a_buffer_flags & BUFFER_FLAG_BUFFER_1_EMPTIED))
		{
#if DEBUG1_A_BUFFER_1_SD
			HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#endif
			if (sd_log_a_data(a_buffer_1, sizeof(a_buffer_1)) != HAL_OK)
			{
				Error_Handler();
			}
			a_buffer_flags |= BUFFER_FLAG_BUFFER_1_EMPTIED;
#if DEBUG1_A_BUFFER_1_SD
			HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#endif
		}
		// If a_buffer_2 is full and has not been saved
		if ((a_buffer_flags & BUFFER_FLAG_BUFFER_2_FULL) && !(a_buffer_flags & BUFFER_FLAG_BUFFER_2_EMPTIED))
		{
#if DEBUG2_A_BUFFER_2_SD
			HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);
#endif
			if (sd_log_a_data(a_buffer_2, sizeof(a_buffer_2)) != HAL_OK)
			{
				Error_Handler();
			}
			a_buffer_flags |= BUFFER_FLAG_BUFFER_2_EMPTIED;
#if DEBUG2_A_BUFFER_2_SD
			HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);
#endif
		}

		// If p_buffer_1 is full and has not been saved
		if ((p_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) && !(p_buffer_flags & BUFFER_FLAG_BUFFER_1_EMPTIED))
		{
#if DEBUG1_P_BUFFER_1_SD
			HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#endif
			if (sd_log_p_data(p_buffer_1, sizeof(p_buffer_1)) != HAL_OK)
			{
				Error_Handler();
			}
			printf("p_buffer_1\r\n");
			p_buffer_flags |= BUFFER_FLAG_BUFFER_1_EMPTIED;
#if DEBUG1_P_BUFFER_1_SD
			HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#endif
		}
		// If p_buffer_2 is full and has not been saved
		if ((p_buffer_flags & BUFFER_FLAG_BUFFER_2_FULL) && !(p_buffer_flags & BUFFER_FLAG_BUFFER_2_EMPTIED))
		{
#if DEBUG2_P_BUFFER_2_SD
			HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);
#endif
			if (sd_log_p_data(p_buffer_2, sizeof(p_buffer_2)) != HAL_OK)
			{
				Error_Handler();
			}
			printf("p_buffer_2\r\n");
			p_buffer_flags |= BUFFER_FLAG_BUFFER_2_EMPTIED;
#if DEBUG2_P_BUFFER_2_SD
			HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);
#endif
		}

		if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		{
			capture_running = 0;
			break;
		}
	}

	if (sd_uninit() != HAL_OK)
	{
		Error_Handler();
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	printf("(%lu) Capture stopped\r\n", HAL_GetTick());
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 12;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
	htim2.Init.Period = 24000;
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
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
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

	/*Configure GPIO pin : uSD_Detect_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

	return ch;
}

void switch_double_buffers(uint8_t *buffer_flags)
{
	// If buffer_2 has been filled
	if (*buffer_flags & BUFFER_FLAG_BUFFER_1_FULL)
	{
		// If buffer_1 has been emptied
		if (*buffer_flags & BUFFER_FLAG_BUFFER_1_EMPTIED)
		{
			// buffer_2 full
			*buffer_flags |= BUFFER_FLAG_BUFFER_2_FULL;
			// Reset buffer_1 to use
			*buffer_flags &= ~BUFFER_FLAG_BUFFER_1_FULL & ~BUFFER_FLAG_BUFFER_1_EMPTIED;
		}
		else
		{
			printf("(%lu) WARNING: buffer_2 overflowing! (Flags in %p)\r\n", HAL_GetTick(), buffer_flags);
		}
	}
	// If buffer_1 has been filled
	else
	{
		// If buffer_2 has been emptied
		if (*buffer_flags & BUFFER_FLAG_BUFFER_2_EMPTIED)
		{
			// buffer_1 full
			*buffer_flags |= BUFFER_FLAG_BUFFER_1_FULL;
			// Reset buffer_2 to use
			*buffer_flags &= ~BUFFER_FLAG_BUFFER_2_FULL & ~BUFFER_FLAG_BUFFER_2_EMPTIED;
		}
		else
		{
			printf("(%lu) WARNING: buffer_1 overflowing! (Flags in %p)\r\n", HAL_GetTick(), buffer_flags);
		}
	}
}

void a_buffer_write_inc()
{
	a_data_point_t *a_buffer_current = (a_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? a_buffer_2 : a_buffer_1;

	// If data of current datapoint is complete
	if (a_buffer_current[a_write_index].complete >= 0b111)
	{
		// Set timestamp to time of completion
		a_buffer_current[a_write_index].timestamp = HAL_GetTick();
		a_buffer_current[a_write_index].x_mems1 = a_write_index;

		// If next index would overflow
		if (++a_write_index >= A_BUFFER_LEN)
		{
			// Switch buffers
			a_write_index = 0;
			switch_double_buffers(&a_buffer_flags);
		}

		a_buffer_current = (a_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? a_buffer_2 : a_buffer_1;
		a_buffer_current[a_write_index].complete = 0;
	}
}

void p_buffer_write_inc()
{
	p_data_point_t *p_buffer_current = (p_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? p_buffer_2 : p_buffer_1;

	// If data of current data point is complete
	if (p_buffer_current[p_write_index].complete >= 0b1)
	{
		// Set timestamp to time of completion
		p_buffer_current[p_write_index].timestamp = HAL_GetTick();
		p_buffer_current[p_write_index].height = p_write_index;

		// If next index would overflow
		if (++p_write_index >= P_BUFFER_LEN)
		{
			// Switch buffers
			p_write_index = 0;
			switch_double_buffers(&p_buffer_flags);
		}

		p_buffer_current = (p_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? p_buffer_2 : p_buffer_1;
		p_buffer_current[p_write_index].complete = 0;
	}
}

void Timer2_Handler(void)
{
	if (capture_running)
	{
#if DEBUG1_TIMER
	HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_SET);
#endif
		// TODO: GPS request, ADXL sync
		test++;
		if (test > 100)
		{
			p_data_point_t *p_buffer_current = (p_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? p_buffer_2 : p_buffer_1;
			p_buffer_current[p_write_index].complete |= P_COMPLETE_GPS;
			p_buffer_write_inc();
			test = 0;
		}
#if DEBUG1_TIMER
	HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_RESET);
#endif
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (capture_running)
	{
		// Write to a_buffer_2 if a_buffer_1 is full
		a_data_point_t *a_buffer_current = (a_buffer_flags & BUFFER_FLAG_BUFFER_1_FULL) ? a_buffer_2 : a_buffer_1;

		// ADC for Piezo 1
		if (hadc->Instance == ADC1)
		{
#if DEBUG1_ADC1_CONV
			HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_SET);
#endif
			// TODO: unsigned/signed? mems? 16-bit adc?
			a_buffer_current[a_write_index].a_piezo1 = HAL_ADC_GetValue(hadc);
			a_buffer_current[a_write_index].complete |= A_COMPLETE_PZ1 | A_COMPLETE_MEMS;
			a_buffer_write_inc();
#if DEBUG1_ADC1_CONV
			HAL_GPIO_WritePin(Debug1_GPIO_Port, Debug1_Pin, GPIO_PIN_RESET);
#endif
		}
		// ADC for Piezo 2
		else if (hadc->Instance == ADC2)
		{
#if DEBUG2_ADC2_CONV
			HAL_GPIO_WritePin(Debug2_GPIO_Port, Debug2_Pin, GPIO_PIN_SET);
#endif
			a_buffer_current[a_write_index].a_piezo2 = HAL_ADC_GetValue(hadc);
			a_buffer_current[a_write_index].complete |= A_COMPLETE_PZ2;
			a_buffer_write_inc();
#if DEBUG2_ADC2_CONV
			HAL_GPIO_WritePin(Debug2_GPIO_Port, Debug2_Pin, GPIO_PIN_RESET);
#endif
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
	printf("Fatal Error\r\n");
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
