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

HAL_StatusTypeDef sd_touch_file(TCHAR *path)
{
	if (f_open(&SDFile, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		return HAL_ERROR;
	}
	f_close(&SDFile);

	return HAL_OK;
}

void sd_update_file_paths(void)
{
	sprintf(a_file_path, A_FILE_FORMAT, file_num);
	sprintf(p_file_path, P_FILE_FORMAT, file_num);
	sprintf(log_file_path, LOG_FILE_FORMAT, file_num);
}

HAL_StatusTypeDef sd_init(uint8_t do_format)
{
	if (HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port, uSD_Detect_Pin) == GPIO_PIN_SET)
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
			sscanf(file_info.fname, LOG_FILE_FORMAT, &test_num);
			file_num = test_num > file_num ? test_num : file_num;
		}
	}
	f_closedir(&dir_root);
	file_num++;
	sd_update_file_paths();
	printf("(%lu) First files '%s' / '%s'\r\n", HAL_GetTick(), a_file_path, p_file_path);

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_Delay(250);

	/*
	 for (int32_t i = 0; i < file_num; i++)
	 {
	 HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	 HAL_Delay(250);
	 HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	 HAL_Delay(250);
	 }
	 */

	if (sd_touch_file(a_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Accel. file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (sd_touch_file(p_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Pos. file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	if (sd_touch_file(log_file_path) != HAL_OK)
	{
		printf("(%lu) ERROR: sd_init: Log file touch failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	printf("(%lu) SD card initialized successfully\r\n", HAL_GetTick());
	return HAL_OK;
}

HAL_StatusTypeDef sd_uninit(void)
{
	f_close(&SDFile);
	f_mount(NULL, (TCHAR const*)SDPath, 0);

	return HAL_OK;
}

HAL_StatusTypeDef sd_log_a_data(volatile a_data_point_t *a_data_buffer, uint32_t a_data_size)
{
	if (f_open(&SDFile, a_file_path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("(%lu) ERROR: sd_log_a_data: Accel. file open failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	UINT bytes_written;
	FRESULT res = f_write(&SDFile, (void*)a_data_buffer, a_data_size, &bytes_written);
	if (res != FR_OK || bytes_written != a_data_size)
	{
		printf("(%lu) ERROR: sd_log_a_data: Accel. file binary write failed (%hu / %lu bytes)\r\n", HAL_GetTick(), bytes_written, a_data_size);
		f_close(&SDFile);
		return HAL_ERROR;
	}

	if (f_close(&SDFile) != FR_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef sd_log_p_data(volatile p_data_point_t *p_data_buffer, uint32_t p_data_size)
{
	if (f_open(&SDFile, p_file_path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("(%lu) ERROR: sd_log_p_data: Pos. file open failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	UINT bytes_written;
	FRESULT res = f_write(&SDFile, (void*)p_data_buffer, p_data_size, &bytes_written);
	if (res != FR_OK || bytes_written != p_data_size)
	{
		printf("(%lu) ERROR: sd_log_p_data: Pos. file binary write failed (%hu / %lu bytes)\r\n", HAL_GetTick(), bytes_written, p_data_size);
		f_close(&SDFile);
		return HAL_ERROR;
	}

	if (f_close(&SDFile) != FR_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef sd_flush_log(void)
{
	if (sd_log_write_index == 0)
		return HAL_OK;

	if (f_open(&SDFile, log_file_path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("(%lu) ERROR: sd_flush_log: Log file open failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	uint32_t sd_log_size = sd_log_write_index;
	UINT bytes_written;
	FRESULT res = f_write(&SDFile, (void*)sd_log, sd_log_size, &bytes_written);
	sd_log_write_index -= bytes_written;
	if (res != FR_OK || bytes_written != sd_log_size)
	{
		printf("(%lu) ERROR: sd_flush_log: Log file write failed (%hu / %lu bytes)\r\n", HAL_GetTick(), bytes_written, sd_log_size);
		f_close(&SDFile);
		return HAL_ERROR;
	}

	if (f_close(&SDFile) != FR_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}
