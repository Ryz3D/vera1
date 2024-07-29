/*
 * config.h
 *
 *  Created on: Jul 14, 2024
 *      Author: mirco
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define OVERSAMPLING_RATIO 4
#define PIEZO_COUNT 5
#define A_BUFFER_LEN 4096
#define P_BUFFER_LEN 128

// Reserved for BOD testing
#define DEBUG1 HAL_GPIO_TogglePin(Debug1_GPIO_Port, Debug1_Pin);
#define DEBUG2 HAL_GPIO_TogglePin(Debug2_GPIO_Port, Debug2_Pin);

// Saving a_buffer_1 to a file
#define DEBUG_A_BUFFER_1_SD ;
// Saving a_buffer_2 to a file
#define DEBUG_A_BUFFER_2_SD ;
// Saving p_buffer_1 to a file
#define DEBUG_P_BUFFER_1_SD ;
// Saving p_buffer_2 to a file
#define DEBUG_P_BUFFER_2_SD ;
// Data capture timer interrupt
#define DEBUG_TIMER DEBUG1
// Writing ADC1 result into a_data_point_t
#define DEBUG_ADC_PZ_CONV DEBUG2

#endif /* INC_CONFIG_H_ */
