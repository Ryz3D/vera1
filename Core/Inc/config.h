/*
 * config.h
 *
 *  Created on: Jul 26, 2024
 *      Author: mirco
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

// Sampling rate = 4 kSa/s * OVERSAMPLING_RATIO
// default: 4 -> 16 kSa/s
#define OVERSAMPLING_RATIO 4
// Number of ADC channels configured
// default: 5
#define PIEZO_CHANNEL_COUNT 5
// Number of ADC channels to filter and save
// default: 2
#define PIEZO_COUNT 5
// Length of acceleration data point buffer (write to SD-card every (4096 Sa) / (4 kSa/s) = 1.024 s)
// This option has a huge impact on RAM usage, it can be reduced if the main function has enough time to save the buffer before the next interrupt
// default: 4096
#define A_BUFFER_LEN 4096
// Length of position data point buffer (write to SD-card every (128 Sa) / (40 Sa/s) = 3.2 s)
// default: 128
#define P_BUFFER_LEN 128

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
#define DEBUG_A_TIMER DEBUG1
// Piezo ADC result (@ 4 kHz)
#define DEBUG_ADC_PZ_CONV DEBUG2
// GPS timer interrupt (@ 40 Hz)
#define DEBUG_P_TIMER ;

#define DEBUG_TEST_PRINT_CONFIG 1
#define DEBUG_TEST_ALWAYS_FORMAT_SD 0
#define DEBUG_TEST_NEVER_FORMAT_SD 0
#define DEBUG_TEST_FIR_FREQUENCY_SWEEP 0
#define DEBUG_TEST_FIR_DAC 1
#define DEBUG_TEST_PRINT_A 1
#define DEBUG_TEST_PRINT_P 1

#endif /* INC_CONFIG_H_ */
