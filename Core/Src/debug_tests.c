/*
 * debug_tests.c
 *
 *  Created on: Aug 6, 2024
 *      Author: mirco
 */

#include "debug_tests.h"

void Debug_test_print_config()
{
	printf("(%lu) Compiled Config:\r\n", HAL_GetTick());
	printf("    VERSION=%u\r\n", VERSION);
	printf("    LOAD_CONFIG=%u\r\n", LOAD_CONFIG);
	printf("    OVERSAMPLING_RATIO_MAX=%u\r\n", OVERSAMPLING_RATIO_MAX);
	printf("    PIEZO_COUNT_MAX=%u\r\n", PIEZO_COUNT_MAX);
	printf("    A_BUFFER_LEN_MAX=%u\r\n", A_BUFFER_LEN_MAX);
	printf("    P_BUFFER_LEN_MAX=%u\r\n", P_BUFFER_LEN_MAX);
	char config_buffer[500];
	Config_Save(config_buffer, sizeof(config_buffer));
	printf("(%lu) Loaded Config:\r\n    ", HAL_GetTick());
	for (uint32_t i = 0; i < strlen(config_buffer); i++)
	{
		if (config_buffer[i] == '\n' && i < strlen(config_buffer) - 1)
			printf("\n    ");
		else
			printf("%c", config_buffer[i]);
	}
}

void Debug_test_FIR_frequency_sweep(FIR_t *hfir)
{
	float freqs[400];
	float amps[400];
	float delays[400];

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		freqs[f_i] = 10 + f_i * 10;
		amps[f_i] = 0;
		delays[f_i] = 0;
	}

	for (uint16_t f_i = 0; f_i < 400; f_i++)
	{
		uint32_t sim_t = 0;
		float a_max = 0;
		uint8_t delay_set = 0;
		for (uint16_t iter = 0; iter < 400; iter++)
		{
			for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
			{
				hfir->In[i] = sin(2.0f * PI * freqs[f_i] / 16000.0f * (float)sim_t);
				sim_t++;
			}
			FIR_Update(hfir);
			for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
			{
				a_max = hfir->Out[i] > a_max ? hfir->Out[i] : a_max;
				if (a_max > 0.1 && delay_set == 0)
				{
					delays[f_i] = (float)sim_t / 16000.0f;
					delay_set = 1;
				}
			}
		}
		amps[f_i] = a_max;
		for (uint8_t i = 0; i < FIR_BLOCK_LEN; i++)
		{
			hfir->In[i] = 0;
		}
		for (uint8_t j = 0; j < 10; j++)
		{
			FIR_Update(hfir);
		}
	}

	/*
	 printf("%f", hfir_pz[0].Out[i]);
	 int16_t k;
	 if (hfir_pz[0].Out[i] < -1.0f)
	 hfir_pz[0].Out[i] = -1.0f;
	 else if (hfir_pz[0].Out[i] > -1.0f)
	 hfir_pz[0].Out[i] = 1.0f;
	 k = (int16_t)(hfir_pz[0].Out[i] * 0xFFF); // 13 bit?, 24 bit -> 0x7FFFFF (do this and bit shift?), 12 bit -> 0x7FF
	 printf(" (%hi)\r\n", k);
	 */

	printf("[");
	for (uint16_t i = 0; i < 400; i++)
	{
		printf("(%hu, %.4f)", (uint16_t)freqs[i], amps[i]);
		if (i < 399)
			printf(",");
	}
	printf("]\r\n\r\n");

	printf("[");
	for (uint16_t i = 0; i < 400; i++)
	{
		printf("(%hu, %.4f)", (uint16_t)freqs[i], delays[i]);
		if (i < 399)
			printf(",");
	}
	printf("]\r\n\r\n");
}

void Debug_test_print_a(volatile a_data_point_t *buffer)
{
	uint8_t ch = 0;
	int16_t pz_min = 5000, pz_max = -5000;
	int32_t mems_min = 1000000000, mems_max = -1000000000;
	for (uint16_t i = 0; i < config.a_buffer_len; i++)
	{
		pz_min = buffer[i].a_piezo[ch] < pz_min ? buffer[i].a_piezo[ch] : pz_min;
		pz_max = buffer[i].a_piezo[ch] > pz_max ? buffer[i].a_piezo[ch] : pz_max;
		mems_min = buffer[i].y_mems1 < pz_min ? buffer[i].y_mems1 : pz_min;
		mems_max = buffer[i].y_mems1 > pz_max ? buffer[i].y_mems1 : pz_max;
	}
	float pz_ampl = (pz_max - pz_min) / 4095.0f * 3.3f;
	float pz_offs = (pz_min + pz_max) / 4095.0f * 1.65f;
	float mems_ampl = mems_max - mems_min;
	float mems_offs = mems_min - mems_max;
	printf("(%lu) Peak-peak: Piezo: %.3f V\tMEMS: %.3f\tOffset: Piezo: %.3f V\tMEMS: %.3f\r\n", HAL_GetTick(), pz_ampl, mems_ampl, pz_offs, mems_offs);
}

void Debug_test_print_p(volatile p_data_point_t *buffer)
{
	printf("(%lu) idk what to print here at %.3f km/h\r\n", HAL_GetTick(), buffer[0].speed);
}
