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

#define A_FILE_FORMAT "%li_A.BIN"
#define P_FILE_FORMAT "%li_P.BIN"
#define LOG_FILE_FORMAT "%li_LOG.TXT"

#define SD_LOG_LEN 512

extern int32_t file_num;
extern TCHAR a_file_path[];
extern TCHAR p_file_path[];
extern TCHAR log_file_path[];

extern char sd_log[];
extern uint32_t sd_log_write_index;

HAL_StatusTypeDef sd_touch_file(TCHAR *path);
void sd_update_file_paths(void);

HAL_StatusTypeDef sd_init(uint8_t do_format);
HAL_StatusTypeDef sd_uninit(void);
HAL_StatusTypeDef sd_log_a_data(volatile a_data_point_t *a_data_buffer, uint32_t a_data_size);
HAL_StatusTypeDef sd_log_p_data(volatile p_data_point_t *p_data_buffer, uint32_t p_data_size);
HAL_StatusTypeDef sd_flush_log(void);

#endif /* INC_SD_H_ */
