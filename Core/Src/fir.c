/*
 * DigitalFilter.c
 *
 *  Created on: Jul 29, 2024
 *      Author: mirco
 */

#include <fir.h>

#define FIR_REV 1

float32_t taps_rev[FIR_TAPS_LEN];

HAL_StatusTypeDef FIR_Init(FIR_t *filter, const float32_t *taps)
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
	arm_fir_init_f32(&filter->Instance, FIR_TAPS_LEN, (float32_t*)filter->Taps, filter->State, FIR_BLOCK_LEN);
	return HAL_OK;
}

HAL_StatusTypeDef FIR_Update(FIR_t *filter)
{
	arm_fir_f32(&filter->Instance, filter->In, filter->Out, FIR_BLOCK_LEN);
	return HAL_OK;
}
