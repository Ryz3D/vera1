/*
 * fir.c
 *
 * Wrapper for CMSIS-DSP FIR filter (16-bit int)
 *
 *  Created on: Jul 29, 2024
 *      Author: mirco
 */

#include <fir.h>

// TODO: pre-reverse taps
#define FIR_REV 1

q15_t taps_rev[FIR_TAPS_LEN];

HAL_StatusTypeDef FIR_Init(FIR_t *filter, const int16_t taps[])
{
#if FIR_REV
	for (uint16_t i = 0; i < FIR_TAPS_LEN; i++)
	{
		taps_rev[i] = taps[FIR_TAPS_LEN - 1 - i];
	}
	filter->Taps = taps_rev;
#else
	filter->Taps = taps;
#endif
	for (uint16_t i = 0; i < FIR_TAPS_LEN + config.oversampling_ratio - 1; i++)
	{
		filter->State[i] = 0;
	}
	for (uint16_t i = 0; i < config.oversampling_ratio; i++)
	{
		filter->In[i] = filter->Out[i] = 0;
	}
	arm_fir_init_q15(&filter->Instance, FIR_TAPS_LEN, (q15_t*)filter->Taps, filter->State, config.oversampling_ratio);
	return HAL_OK;
}

HAL_StatusTypeDef FIR_Update(FIR_t *filter)
{
	arm_fir_q15(&filter->Instance, filter->In, filter->Out, config.oversampling_ratio);
	return HAL_OK;
}
