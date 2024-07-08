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

HAL_StatusTypeDef sd_touch_file(TCHAR *path)
{
	if (f_open(&SDFile, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		return HAL_ERROR;
	}
	f_close(&SDFile);

	return HAL_OK;
}

void sd_update_file_paths()
{
	sprintf(a_file_path, A_FILE_FORMAT, file_num);
	sprintf(p_file_path, P_FILE_FORMAT, file_num);
}

HAL_StatusTypeDef sd_init(uint8_t do_format)
{
	// TODO: GPIO speed SDMMC high (bis 100 MHz) https://fastbitlab.com/gpio-output-speed-register-applicability/
	// TODO: SDMMC CLK 48Mhz

	if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		printf("ERROR: sd_init: Failed to mount\r\n");
		return HAL_ERROR;
	}

	if (do_format)
	{
		uint8_t rtext[_MAX_SS];
		if (f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
		{
			printf("ERROR: sd_init: Failed to create FAT volume (did you generate code?)\r\n");
			return HAL_ERROR;
		}
	}

	DIR dir_root;
	if (f_opendir(&dir_root, "") != FR_OK)
	{
		printf("sd_init: Root dir open failed (did you generate code?)\r\n");
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
			if (test_num > file_num)
			{
				file_num = test_num;
			}
		}
	}
	f_closedir(&dir_root);
	file_num++;
	sd_update_file_paths();
	printf("sd_init: First files '%s' / '%s'\r\n", a_file_path, p_file_path);

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

	if (sd_touch_file(a_file_path) != HAL_OK)
	{
		printf("sd_init: Accel. file touch failed\r\n");
		return HAL_ERROR;
	}
	if (sd_touch_file(p_file_path) != HAL_OK)
	{
		printf("sd_init: Pos. file touch failed\r\n");
		return HAL_ERROR;
	}

	printf("sd_init: Initialized successfully\r\n");
	return HAL_OK;
}

HAL_StatusTypeDef sd_uninit()
{
	if (f_mount(NULL, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		printf("ERROR: sd_init: Failed to unmount\r\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef sd_test()
{
	if (f_open(&SDFile, "stm32.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		printf("stm32.txt f_open failed\r\n");
		return HAL_ERROR;
	}

	// write to text file
	char wtext[] = "hallo ich bin mirco und ich glaube das funktioniert jetzt\r\n...\r\n\r\nhoffentlich\r\n\0";
	uint32_t bytes_written;
	FRESULT res = f_write(&SDFile, wtext, strlen((char*)wtext), (void*)&bytes_written);
	if ((bytes_written == 0) || (res != FR_OK))
	{
		return HAL_ERROR;
	}
	f_close(&SDFile);

	// read back file
	if (f_open(&SDFile, "stm32.txt", FA_OPEN_EXISTING | FA_READ) != FR_OK)
	{
		return HAL_ERROR;
	}
	char read_buffer[256];
	uint32_t bytes_read = 0;
	res = f_read(&SDFile, read_buffer, sizeof(read_buffer), (void*)&bytes_read);
	if ((bytes_read == 0) || (res != FR_OK))
	{
		return HAL_ERROR;
	}
	printf("sd_test: read %lu bytes (strlen %u):\r\n%s\r\n(eof)\r\n", bytes_read, strlen(read_buffer), read_buffer);

	return HAL_OK;
}

struct p_data_point data_buffer_2[5096];

HAL_StatusTypeDef sd_log_a_data(struct a_data_point *a_data_buffer, uint32_t a_data_len)
{
	memcpy((void*)data_buffer_2, (void*)a_data_buffer, a_data_len * sizeof(struct a_data_point));

	if (f_open(&SDFile, a_file_path, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	{
		printf("Accel. file open failed\r\n");
		return HAL_ERROR;
	}

	/*
	 char file_buffer[64];
	 // ASCII write
	 for (uint32_t i = 0; i < sizeof(adc_buffer) / sizeof(uint16_t); i++)
	 {
	 sprintf(file_buffer, "%hu\r\n\0", adc_buffer[i]);
	 uint32_t byteswritten;
	 FRESULT res = f_write(&SDFile, file_buffer, strlen(file_buffer), (void*)&byteswritten);
	 if ((byteswritten == 0) || (res != FR_OK))
	 {
	 printf("ADC file write failed\r\n");
	 f_close(&SDFile);
	 Error_Handler();
	 }
	 }
	 */

	// timestamp
	uint8_t timestamp_buffer[24] = { 'T', 'I', 'M', 'E', 'S', 'T', 'A', 'M', 'P', ' ' };
	uint32_t ticks = HAL_GetTick();
	memcpy((void*)timestamp_buffer + 10, (void*)&ticks, sizeof(uint32_t));
	UINT bytes_written;
	FRESULT res = f_write(&SDFile, timestamp_buffer, sizeof(timestamp_buffer), (void*)&bytes_written);
	if ((bytes_written != sizeof(timestamp_buffer)) || (res != FR_OK))
	{
		printf("Accel. file timestamp write failed\r\n");
		f_close(&SDFile);
		return HAL_ERROR;
	}

	// binary write
	res = f_write(&SDFile, (void*)data_buffer_2, a_data_len * sizeof(struct a_data_point), &bytes_written);
	if ((bytes_written != a_data_len * sizeof(struct a_data_point)) || (res != FR_OK))
	{
		printf("Accel. file binary write failed (%hu / %lu bytes)\r\n", bytes_written, a_data_len * sizeof(struct a_data_point));
		f_close(&SDFile);
		return HAL_ERROR;
	}

	f_close(&SDFile);
	return HAL_OK;
}
