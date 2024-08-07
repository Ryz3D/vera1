/*
 * sdhc.c
 *
 *  Created on: Jun 26, 2024
 *      Author: mirco
 */

#include "sd.h"

int32_t dir_num = 0, page_num = 0;
uint16_t sd_year, sd_month, sd_day;
TCHAR dir_path[PATH_LEN];
TCHAR a_file_path[PATH_LEN];
TCHAR p_file_path[PATH_LEN];
TCHAR log_file_path[PATH_LEN];

char sd_log[SD_LOG_LEN];
uint32_t sd_log_write_index = 0;

a_data_header_t a_header;
p_data_header_t p_header;

HAL_StatusTypeDef SD_UpdateFilepaths(void)
{
	sprintf(a_file_path, A_FILE_FORMAT, sd_year, sd_month, sd_day, dir_num, page_num);
	sprintf(p_file_path, P_FILE_FORMAT, sd_year, sd_month, sd_day, dir_num, page_num);
	sprintf(log_file_path, LOG_FILE_FORMAT, sd_year, sd_month, sd_day, dir_num);

	if (SD_TouchFile(a_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_UpdateFilepaths: Accel. file (\"%s\") touch failed\r\n", HAL_GetTick(), a_file_path);
		return HAL_ERROR;
	}
	if (SD_TouchFile(p_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_UpdateFilepaths: Pos. file (\"%s\") touch failed\r\n", HAL_GetTick(), p_file_path);
		return HAL_ERROR;
	}
	if (SD_TouchFile(log_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: SD_UpdateFilepaths: Log file (\"%s\") touch failed\r\n", HAL_GetTick(), log_file_path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef SD_Init(uint8_t do_format)
{
	dir_path[0] = '\0';
	a_file_path[0] = '\0';
	p_file_path[0] = '\0';
	log_file_path[0] = '\0';

	if (HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port, uSD_Detect_Pin) == GPIO_PIN_SET)
	{
		printf("(%lu) WARNING: SD_Init: No SD card detected!\r\n", HAL_GetTick());
	}

	if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		printf("(%lu) ERROR: SD_Init: Failed to mount\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	if (do_format)
	{
		printf("(%lu) Formatting SD card...\r\n", HAL_GetTick());
		uint8_t rtext[_MAX_SS];
		if (f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			printf("(%lu) ERROR: SD_Init: Failed to create FAT volume (can be caused by code generation)\r\n", HAL_GetTick());
			return HAL_ERROR;
		}
	}

	DIR dir_root;
	if (f_opendir(&dir_root, "") != FR_OK)
	{
		printf("(%lu) ERROR: SD_Init: Root dir open failed (can be caused by code generation)\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	FILINFO file_info;
	int32_t dir_num = -1;
	while (f_readdir(&dir_root, &file_info) == FR_OK)
	{
		if (file_info.fname[0] == '\0')
		{
			break;
		}
		if (file_info.fattrib & AM_DIR)
		{
			uint16_t test_y, test_m, test_d;
			int32_t test_num;
			sscanf(file_info.fname, DIR_FORMAT, &test_y, &test_m, &test_d, &test_num);
			if (test_y == sd_year && test_m == sd_month && test_d == sd_day)
			{
				dir_num = test_num > dir_num ? test_num : dir_num;
			}
		}
	}
	f_closedir(&dir_root);
	dir_num++;

	sprintf(dir_path, DIR_FORMAT, sd_year, sd_month, sd_day, dir_num);
	FRESULT res;
	if ((res = f_mkdir(dir_path)) != FR_OK)
	{
		printf("error %u\r\n", res);
		printf("(%lu) ERROR: SD_Init: Create dir (\"%s\") failed\r\n", HAL_GetTick(), dir_path);
		return HAL_ERROR;
	}

	if (SD_UpdateFilepaths() != HAL_OK)
	{
		return HAL_ERROR;
	}

	printf("(%lu) Writing dir '%s'\r\n", HAL_GetTick(), dir_path);

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_Delay(250);

	for (int32_t i = 0; i < dir_num; i++)
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		HAL_Delay(250);
	}

	return HAL_OK;
}

HAL_StatusTypeDef SD_Uninit(void)
{
	f_close(&SDFile);
	f_mount(NULL, (TCHAR const*)SDPath, 0);

	return HAL_OK;
}

HAL_StatusTypeDef SD_TouchFile(TCHAR *path)
{
	if (f_open(&SDFile, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		return HAL_ERROR;
	}
	f_close(&SDFile);

	return HAL_OK;
}

HAL_StatusTypeDef SD_WriteBuffer(TCHAR *path, void *data, uint32_t size)
{
	if (size == 0)
	{
		return HAL_OK;
	}

	if (f_open(&SDFile, path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file open failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	UINT bytes_written;
	FRESULT res = f_write(&SDFile, (void*)data, size, &bytes_written);
	if (res != FR_OK || bytes_written != size)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file write failed (%hu / %lu bytes)\r\n", HAL_GetTick(), path, bytes_written, size);
		f_close(&SDFile);
		return HAL_ERROR;
	}

	if (f_close(&SDFile) != FR_OK)
	{
		printf("(%lu) ERROR: SD_WriteBuffer: SD File \"%s\": file close failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef SD_FlushLog(void)
{
	if (sd_log_write_index == 0)
	{
		return HAL_OK;
	}

	if (SD_WriteBuffer(log_file_path, (void*)sd_log, sd_log_write_index) != HAL_OK)
	{
		return HAL_ERROR;
	}
	sd_log_write_index = 0;

	return HAL_OK;
}

HAL_StatusTypeDef SD_NewPage(uint8_t init)
{
	if (init == 0)
	{
		page_num++;
	}
	if (SD_UpdateFilepaths() != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (SD_WriteBuffer(a_file_path, (void*)&a_header, sizeof(a_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (SD_WriteBuffer(p_file_path, (void*)&p_header, sizeof(p_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	printf("(%lu) Page %li (\"%s\", \"%s\")\r\n", HAL_GetTick(), page_num, a_file_path, p_file_path);
	return HAL_OK;
}
