/*
 * config.h
 *
 *  Created on: Jul 14, 2024
 *      Author: mirco
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define A_BUFFER_LEN 4096
#define P_BUFFER_LEN 128
#define ADC_BUFFER_LEN 3

#define DO_FORMAT_SD 1

#define ENABLE_ADC3 0

#define DEBUG1 HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#define DEBUG2 HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);

// Data capture timer interrupt
#define DEBUG_TIMER DEBUG1
// Saving a_buffer_1 to a file
#define DEBUG_A_BUFFER_1_SD ;
// Saving a_buffer_2 to a file
#define DEBUG_A_BUFFER_2_SD ;
// Saving p_buffer_1 to a file
#define DEBUG_P_BUFFER_1_SD ;
// Saving p_buffer_2 to a file
#define DEBUG_P_BUFFER_2_SD ;
// Writing ADC1 result into a_data_point_t
#define DEBUG_ADC1_CONV DEBUG2

#endif /* INC_CONFIG_H_ */
