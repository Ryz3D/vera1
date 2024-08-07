/*
 * sdhc.h
 *
 *  Created on: Jul 26, 2024
 *      Author: mirco
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"
#include "fatfs.h"
#include "data_points.h"

#define DIR_FORMAT "%04hu_%02hu_%02hu_%li"
#define A_FILE_FORMAT DIR_FORMAT "/A_%li.BIN"
#define P_FILE_FORMAT DIR_FORMAT "/P_%li.BIN"
#define LOG_FILE_FORMAT DIR_FORMAT "/_LOG.TXT"

#define PATH_LEN 50

#define SD_LOG_LEN 512

extern int32_t dir_num, page_num;
extern uint16_t sd_year, sd_month, sd_day;
extern TCHAR dir_path[];
extern TCHAR a_file_path[];
extern TCHAR p_file_path[];

extern char sd_log[];
extern uint32_t sd_log_write_index;

HAL_StatusTypeDef SD_Init(uint8_t do_format);
HAL_StatusTypeDef SD_Uninit(void);
HAL_StatusTypeDef SD_TouchFile(TCHAR *path);
HAL_StatusTypeDef SD_WriteBuffer(TCHAR *path, void *data, uint32_t size);
HAL_StatusTypeDef SD_WriteFileHeaders(uint32_t boot_duration, uint32_t fir_taps_len);
HAL_StatusTypeDef SD_FlushLog(void);
HAL_StatusTypeDef SD_IncrementPage(void);

#endif /* INC_SD_H_ */
