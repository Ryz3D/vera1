/*
 * adxl357.h
 *
 *  Created on: Jul 30, 2024
 *      Author: mirco
 */

#pragma once

#include "config.h"
#include "stm32f7xx_hal.h"

#include <stdio.h>
#include <string.h>

HAL_StatusTypeDef adxl_get_reg(SPI_HandleTypeDef *hspi, uint8_t addr);
HAL_StatusTypeDef adxl_set_reg(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t val);
HAL_StatusTypeDef adxl_init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef adxl_read(SPI_HandleTypeDef *hspi, int32_t *x, int32_t *y, int32_t *z, uint16_t *temp);
