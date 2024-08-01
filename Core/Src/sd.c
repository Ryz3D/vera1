/*
 * sdhc.c
 *
 *  Created on: Jun 26, 2024
 *      Author: mirco
 */

#include "sd.h"

int32_t file_num = 0;
TCHAR a_file_path[32];
TCHAR p_file_path[32];
TCHAR log_file_path[32];

char sd_log[SD_LOG_LEN];
uint32_t sd_log_write_index = 0;

void SD_UpdateFilepaths(void)
{
	sprintf(a_file_path, A_FILE_FORMAT, file_num);
	sprintf(p_file_path, P_FILE_FORMAT, file_num);
	sprintf(log_file_path, LOG_FILE_FORMAT, file_num);
}

HAL_StatusTypeDef SD_Init(uint8_t do_format)
{
	if (HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port,
	uSD_Detect_Pin) == GPIO_PIN_SET)
	{
		printf("(%lu) WARNING: sd_init: No SD card detected!\r\n", HAL_GetTick());
	}

	if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		printf("(%lu) ERROR: sd_init: Failed to mount\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	if (do_format)
	{
		printf("(%lu) Formatting SD card...\r\n", HAL_GetTick());
		uint8_t rtext[_MAX_SS];
		if (f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			printf("(%lu) ERROR: sd_init: Failed to create FAT volume (can be caused by code generation)\r\n", HAL_GetTick());
			return HAL_ERROR;
		}
	}

	DIR dir_root;
	if (f_opendir(&dir_root, "") != FR_OK)
	{
		printf("(%lu) ERROR: sd_init: Root dir open failed (can be caused by code generation)\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	FILINFO file_info;
	file_num = -1;
	while (f_readdir(&dir_root, &file_info) == FR_OK)
	{
		if (file_info.fname[0] == '\0')
		{
			break;
		}
		if (!(file_info.fattrib & AM_DIR))
		{
			int32_t test_num = -1;
			sscanf(file_info.fname, A_FILE_FORMAT, &test_num);
			file_num = test_num > file_num ? test_num : file_num;
			sscanf(file_info.fname, P_FILE_FORMAT, &test_num);
			file_num = test_num > file_num ? test_num : file_num;
			sscanf(file_info.fname,
			LOG_FILE_FORMAT, &test_num);
			file_num = test_num > file_num ? test_num : file_num;
		}
	}
	f_closedir(&dir_root);
	file_num++;
	SD_UpdateFilepaths();
	printf("(%lu) First files '%s' / '%s'\r\n", HAL_GetTick(), a_file_path, p_file_path);

	if (SD_TouchFile(a_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Accel. file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (SD_TouchFile(p_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Pos. file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (SD_TouchFile(log_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Log file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	printf("(%lu) SD card initialized successfully\r\n", HAL_GetTick());

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_Delay(250);

	for (int32_t i = 0; i < file_num; i++)
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
	if (f_open(&SDFile, path,
	FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		return HAL_ERROR;
	}
	f_close(&SDFile);

	return HAL_OK;
}

HAL_StatusTypeDef SD_WriteBuffer(TCHAR *path, void *data, uint32_t size)
{
	if (size == 0)
		return HAL_OK;

	if (f_open(&SDFile, path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("(%lu) ERROR: SD File \"%s\": file open failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	UINT bytes_written;
	FRESULT res = f_write(&SDFile, (void*)data, size, &bytes_written);
	if (res != FR_OK || bytes_written != size)
	{
		printf("(%lu) ERROR: SD File \"%s\": file write failed (%hu / %lu bytes)\r\n", HAL_GetTick(), path, bytes_written, size);
		f_close(&SDFile);
		return HAL_ERROR;
	}

	if (f_close(&SDFile) != FR_OK)
	{
		printf("(%lu) ERROR: SD File \"%s\": file close failed\r\n", HAL_GetTick(), path);
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef SD_WriteFileHeaders(uint32_t boot_duration, uint32_t fir_taps_len)
{
	a_data_header_t a_header = { .boot_duration = boot_duration, .a_buffer_len = A_BUFFER_LEN, .piezo_count = PIEZO_COUNT, .oversampling_ratio = OVERSAMPLING_RATIO, .fir_taps_len = fir_taps_len };
	if (SD_WriteBuffer(a_file_path, (void*)&a_header, sizeof(a_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}

	p_data_header_t p_header = { .boot_duration = boot_duration, .p_buffer_len = P_BUFFER_LEN, };
	if (SD_WriteBuffer(p_file_path, (void*)&p_header, sizeof(p_data_header_t)) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef SD_FlushLog(void)
{
	if (sd_log_write_index == 0)
		return HAL_OK;

	if (SD_WriteBuffer(log_file_path, (void*)sd_log, sd_log_write_index) != HAL_OK)
	{
		return HAL_ERROR;
	}
	sd_log_write_index = 0;

	return HAL_OK;
}
