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
#include "data_points.h"

#define A_FILE_FORMAT "A_%li.BIN"
#define P_FILE_FORMAT "P_%li.BIN"

extern int32_t file_num;
extern TCHAR a_file_path[];
extern TCHAR p_file_path[];

HAL_StatusTypeDef sd_touch_file();
void sd_update_file_paths();

HAL_StatusTypeDef sd_init(uint8_t do_format);
HAL_StatusTypeDef sd_uninit();
HAL_StatusTypeDef sd_test();
HAL_StatusTypeDef sd_log_a_data(volatile a_data_point_t *a_data_buffer, uint32_t a_data_size);
HAL_StatusTypeDef sd_log_p_data(volatile p_data_point_t *p_data_buffer, uint32_t p_data_size);

#endif /* INC_SD_H_ */
