/*
 * sdhc.h
 *
 *  Created on: Jun 26, 2024
 *      Author: mirco
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include <stdio.h>
#include <string.h>
#include "fatfs.h"

extern int32_t adc_file_num;
extern TCHAR adc_file_path[];

HAL_StatusTypeDef sd_touch_file();
void sd_update_adc_file_path();

HAL_StatusTypeDef sd_init(uint8_t do_format);
HAL_StatusTypeDef sd_uninit();
HAL_StatusTypeDef sd_test();
HAL_StatusTypeDef sd_log_adc(uint16_t *adc_buffer, uint32_t adc_buffer_size);

#endif /* INC_SD_H_ */
