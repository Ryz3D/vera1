/*
 * DigitalFilter.h
 *
 *  Created on: Jul 29, 2024
 *      Author: mirco
 */

#ifndef INC_FIR_H_
#define INC_FIR_H_

#include "stm32f7xx_hal.h"
#include "arm_math.h"

#define FIR_TAPS_LEN 512
#define FIR_BLOCK_LEN 4

typedef struct
{
	arm_fir_instance_f32 Instance;
	const float32_t *Taps;
	float32_t State[FIR_TAPS_LEN + FIR_BLOCK_LEN - 1];
	float32_t In[FIR_BLOCK_LEN];
	float32_t Out[FIR_BLOCK_LEN];
} FIR_t;

HAL_StatusTypeDef FIR_Init(FIR_t *filter, const float32_t *taps);
HAL_StatusTypeDef FIR_Update(FIR_t *filter);

#endif /* INC_FIR_H_ */
