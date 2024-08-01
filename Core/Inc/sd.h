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

HAL_StatusTypeDef SD_Init(uint8_t do_format);
HAL_StatusTypeDef SD_Uninit(void);
HAL_StatusTypeDef SD_TouchFile(TCHAR *path);
HAL_StatusTypeDef SD_WriteBuffer(TCHAR *path, void *data, uint32_t size);
HAL_StatusTypeDef SD_WriteFileHeaders(uint32_t boot_duration, uint32_t fir_taps_len);
HAL_StatusTypeDef SD_FlushLog(void);

#endif /* INC_SD_H_ */
