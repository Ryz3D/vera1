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

#define ADXL_REG_DEVID_AD 0x00
#define ADXL_REG_DEVID_MST 0x01
#define ADXL_REG_PARTID 0x02
#define ADXL_REG_REVID 0x03
#define ADXL_REG_STATUS 0x04
#define ADXL_REG_FIFO_ENTRIES 0x05
#define ADXL_REG_TEMP2 0x06
#define ADXL_REG_TEMP1 0x07
#define ADXL_REG_XDATA3 0x08
#define ADXL_REG_XDATA2 0x09
#define ADXL_REG_XDATA1 0x0A
#define ADXL_REG_YDATA3 0x0B
#define ADXL_REG_YDATA2 0x0C
#define ADXL_REG_YDATA1 0x0D
#define ADXL_REG_ZDATA3 0x0E
#define ADXL_REG_ZDATA2 0x0F
#define ADXL_REG_ZDATA1 0x10
#define ADXL_REG_FIFO_DATA 0x11
#define ADXL_REG_OFFSET_X_H 0x1E
#define ADXL_REG_OFFSET_X_L 0x1F
#define ADXL_REG_OFFSET_Y_H 0x20
#define ADXL_REG_OFFSET_Y_L 0x21
#define ADXL_REG_OFFSET_Z_H 0x22
#define ADXL_REG_OFFSET_Z_L 0x23
#define ADXL_REG_ACT_EN 0x24
#define ADXL_REG_ACT_THRESH_H 0x25
#define ADXL_REG_ACT_THRESH_L 0x26
#define ADXL_REG_ACT_COUNT 0x27
#define ADXL_REG_FILTER 0x28
#define ADXL_REG_FIFO_SAMPLES 0x29
#define ADXL_REG_INT_MAP 0x2A
#define ADXL_REG_SYNC 0x2B
#define ADXL_REG_RANGE 0x2C
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_SELF_TEST 0x2E
#define ADXL_REG_RESET 0x2F

typedef struct
{
	uint16_t temp;
	int32_t x, y, z;
} adxl_data_t;

HAL_StatusTypeDef adxl_read_reg_multi(SPI_HandleTypeDef *hspi, uint8_t count, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef adxl_write_reg_multi(SPI_HandleTypeDef *hspi, uint8_t count, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef adxl_read_reg(SPI_HandleTypeDef *hspi, uint8_t register_addr, uint8_t *buffer);
HAL_StatusTypeDef adxl_write_reg(SPI_HandleTypeDef *hspi, uint8_t register_addr, uint8_t data);
HAL_StatusTypeDef adxl_init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef adxl_request_data(SPI_HandleTypeDef *hspi);
adxl_data_t adxl_rx_callback();
