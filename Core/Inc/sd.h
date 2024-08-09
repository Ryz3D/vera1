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

#define CONFIG_FILE_PATH "config.txt"
#define DIR_FORMAT "%04hu_%02hu_%02hu-%li"
#define A_FILE_FORMAT DIR_FORMAT "/a_%li.bin"
#define P_FILE_FORMAT DIR_FORMAT "/p_%li.bin"
#define LOG_FILE_FORMAT DIR_FORMAT "/_log.txt"

#define PATH_LEN 50

#define SD_LOG_LEN 512

extern int32_t dir_num, page_num;
extern uint16_t sd_year, sd_month, sd_day;
extern TCHAR dir_path[];
extern TCHAR a_file_path[];
extern TCHAR p_file_path[];

extern char sd_log[];
extern uint32_t sd_log_write_index;

extern a_data_header_t a_header;
extern p_data_header_t p_header;

HAL_StatusTypeDef SD_Init(uint8_t do_format);
HAL_StatusTypeDef SD_InitDir(void);
HAL_StatusTypeDef SD_FlushLog(void);
HAL_StatusTypeDef SD_NewPage(uint8_t init);
HAL_StatusTypeDef SD_Uninit(void);
HAL_StatusTypeDef SD_TouchFile(TCHAR *path);
uint8_t SD_FileExists(TCHAR *path);
HAL_StatusTypeDef SD_ReadBuffer(TCHAR *path, void *data, UINT size, UINT *size_read);
HAL_StatusTypeDef SD_WriteBuffer(TCHAR *path, void *data, UINT size);

#endif /* INC_SD_H_ */
