/*
 * adxl357.h
 *
 *  Created on: Jun 15, 2024
 *      Author: mirco
 */

#pragma once

#include "stm32f7xx_hal.h"

HAL_StatusTypeDef adxl357_init(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef adxl357_read(SPI_HandleTypeDef *hspi, int32_t *x, int32_t *y, int32_t *z, uint16_t *temp);
