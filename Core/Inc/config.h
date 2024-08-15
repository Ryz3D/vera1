/*
 * config.h
 *
 *  Created on: Jul 26, 2024
 *      Author: mirco
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"

#define VERSION 1
// Load config from SD
// default: 1
#define LOAD_CONFIG 1

#define OVERSAMPLING_RATIO_MAX 20
#define PIEZO_COUNT_MAX 5
#define A_BUFFER_LEN_MAX 4096
#define P_BUFFER_LEN_MAX 128

#define C_F_A_SAMPLING_RATE "a_sampling_rate=%lu"
#define C_F_P_SAMPLING_RATE "p_sampling_rate=%lu"
#define C_F_OVERSAMPLING_RATIO "oversampling_ratio=%u"
#define C_F_PIEZO_COUNT "piezo_count=%u"
#define C_F_A_BUFFER_LEN "a_buffer_len=%lu"
#define C_F_P_BUFFER_LEN "p_buffer_len=%lu"
#define C_F_PAGE_DURATION_MS "page_duration_ms=%lu"

typedef struct
{
	// Rate of saved acceleration samples
	uint32_t a_sampling_rate;
	// Rate of saved position samples
	uint32_t p_sampling_rate;
	// ADC sampling rate = a_sampling_rate * oversampling_ratio
	uint8_t oversampling_ratio;
	// Number of ADC channels
	uint8_t piezo_count;
	// Length of acceleration data point buffer (write to SD-card every (4096 Sa) / (4 kSa/s) = 1.024 s)
	// This option can have a huge impact on RAM usage, it can be reduced if the main function has enough time to save the buffer before the next interrupt
	uint32_t a_buffer_len;
	// Length of position data point buffer (write to SD-card every (128 Sa) / (40 Sa/s) = 3.2 s)
	uint32_t p_buffer_len;
	// Duration before switching to next page (file) in milliseconds
	uint32_t page_duration_ms;
} config_t;

extern config_t default_config, config;

#define DEBUG1 HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#define DEBUG2 HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);

// Following defines are called at start and end of a function -> Debug pin is high for entire duration
// Saving a_buffer_1 to a file
#define DEBUG_A_BUFFER_1_SD ;
// Saving a_buffer_2 to a file
#define DEBUG_A_BUFFER_2_SD ;
// Saving p_buffer_1 to a file
#define DEBUG_P_BUFFER_1_SD ;
// Saving p_buffer_2 to a file
#define DEBUG_P_BUFFER_2_SD ;
// Piezo timer interrupt (@ 4 kHz)
#define DEBUG_A_TIMER ;
// Piezo ADC result (@ 4 kHz)
#define DEBUG_ADC_PZ_CONV DEBUG2
// GPS timer interrupt (@ 40 Hz)
#define DEBUG_P_TIMER ;

#define DEBUG_TEST_PRINT_CONFIG 1
#define DEBUG_TEST_ALWAYS_FORMAT_SD 0
#define DEBUG_TEST_NEVER_FORMAT_SD 0
#define DEBUG_TEST_BOOT_WITHOUT_GPS 1
#define DEBUG_TEST_FIR_FREQUENCY_SWEEP 0
#define DEBUG_TEST_FIR_DAC 1
#define DEBUG_TEST_PRINT_A 1
#define DEBUG_TEST_PRINT_P 0
#define DEBUG_TEST_PRINT_NEW_PAGE 1

void Config_Default(void);
void Config_Load(char *buffer, uint32_t size);
void Config_Save(char *buffer, uint32_t size);
HAL_StatusTypeDef Config_Init(ADC_HandleTypeDef *hadc1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);

#endif /* INC_CONFIG_H_ */
