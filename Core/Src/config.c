/*
 * config.c
 *
 *  Created on: Aug 9, 2024
 *      Author: mirco
 */

#include "config.h"

config_t default_config =
	{
		.a_sampling_rate = 4000, // 4000 (Sa/s)
		.p_sampling_rate = 10, // 10 (Sa/s)
		.oversampling_ratio = 4, // 4 (16 kSa/s)
		.piezo_count = 5, // 3
		.a_buffer_len = 4096, // 4096 (Sa)
		.p_buffer_len = 128, // 128 (Sa)
		.page_duration_ms = 1 * 60 * 1000 // default: 10 * 60 * 1000
	};

config_t config;

#define C_READ_VAR(format, var) \
	if (sscanf(buffer + i, format "%n", &var, &n)) \
		i += n; \

#define C_CHECK_VAR(format, var, v_min, v_max) \
	if (var < v_min || var > v_max) \
	{ \
		typeof(var) v_def = *(typeof(var)*)((void*)&var - (void*)&config + (void*)&default_config); \
		printf("(%lu) WARNING: Config_Load: Invalid value " format ", resetting to " format "\r\n", HAL_GetTick(), var, v_def); \
		var = v_def; \
	}

#define C_WRITE_VAR(format, var) \
	i += sprintf(buffer + i, format "\r\n", var); \
	if (i >= size) \
		return;

void Config_Default()
{
	memcpy(&config, &default_config, sizeof(config_t));
}

void Config_Load(char *buffer, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
	{
		if (buffer[i] == '\0')
			break;
		if (buffer[i] == '\r' || buffer[i] == '\n')
			continue;

		int n = 0;
		C_READ_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate);
		C_READ_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate);
		C_READ_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio);
		C_READ_VAR(C_F_PIEZO_COUNT, config.piezo_count);
		C_READ_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len);
		C_READ_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len);
		C_READ_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms);
	}

	C_CHECK_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate, 1, 100000);
	C_CHECK_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate, 1, 100000);
	C_CHECK_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio, 1, 100000);
	C_CHECK_VAR(C_F_PIEZO_COUNT, config.piezo_count, 1, 100000);
	C_CHECK_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len, 1, 100000);
	C_CHECK_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len, 1, 100000);
	C_CHECK_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms, 1, 100000);
}

void Config_Save(char *buffer, uint32_t size)
{
	uint32_t i = 0;
	C_WRITE_VAR(C_F_A_SAMPLING_RATE, config.a_sampling_rate);
	C_WRITE_VAR(C_F_P_SAMPLING_RATE, config.p_sampling_rate);
	C_WRITE_VAR(C_F_OVERSAMPLING_RATIO, config.oversampling_ratio);
	C_WRITE_VAR(C_F_PIEZO_COUNT, config.piezo_count);
	C_WRITE_VAR(C_F_A_BUFFER_LEN, config.a_buffer_len);
	C_WRITE_VAR(C_F_P_BUFFER_LEN, config.p_buffer_len);
	C_WRITE_VAR(C_F_PAGE_DURATION_MS, config.page_duration_ms);
}
