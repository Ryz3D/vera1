/*
 * adxl.c
 *
 *  Created on: Aug 02, 2024
 *      Author: mirco
 */

#include "adxl.h"

uint8_t adxl_request_buffer[12];
uint8_t adxl_data_buffer[12];

HAL_StatusTypeDef adxl_read_reg_multi(SPI_HandleTypeDef *hspi, uint8_t count, uint8_t register_addr, uint8_t *buffer)
{
	// TODO: Set hadxl state to busy

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);

	uint8_t addr_tx = (register_addr << 1) | 1;
	// TODO: hadxl timeout
	if (HAL_SPI_Transmit(hspi, &addr_tx, sizeof(uint8_t), 1000) != HAL_OK)
	{
		// TODO: Set hadxl state
		return HAL_ERROR;
	}
	if (HAL_SPI_Receive(hspi, buffer, count * sizeof(uint8_t), 1000) != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);

	return HAL_OK;
}

HAL_StatusTypeDef adxl_write_reg_multi(SPI_HandleTypeDef *hspi, uint8_t count, uint8_t register_addr, uint8_t *buffer)
{
	// TODO: Set hadxl state to busy

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);

	uint8_t addr_tx = (register_addr << 1) | 0;
	// TODO: hadxl timeout
	if (HAL_SPI_Transmit(hspi, &addr_tx, sizeof(uint8_t), 1000) != HAL_OK)
	{
		// TODO: Set hadxl state
		return HAL_ERROR;
	}
	if (HAL_SPI_Transmit(hspi, buffer, count, 1000) != HAL_OK)
	{
		// TODO: Set hadxl state
		return HAL_ERROR;
	}

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);

	return HAL_OK;
}

HAL_StatusTypeDef adxl_read_reg(SPI_HandleTypeDef *hspi, uint8_t register_addr, uint8_t *buffer)
{
	return adxl_read_reg_multi(hspi, 1, register_addr, buffer);
}

HAL_StatusTypeDef adxl_write_reg(SPI_HandleTypeDef *hspi, uint8_t register_addr, uint8_t buffer)
{
	return adxl_write_reg_multi(hspi, 1, register_addr, &buffer);
}

HAL_StatusTypeDef adxl_init(SPI_HandleTypeDef *hspi)
{
	// TODO: refactor to ADXL struct handle

	// Flush SPI
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	uint8_t ignore;
	for (uint8_t i = 0; i < 10; i++)
	{
		adxl_read_reg(hspi, 0x00, &ignore);
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);

	// Read identification registers
	uint8_t buffer_adxl_id[3];
	if (adxl_read_reg_multi(hspi, sizeof(buffer_adxl_id), ADXL_REG_DEVID_AD, buffer_adxl_id) != HAL_OK)
	{
		printf("(%lu) ERROR: adxl_init: Register read failed", HAL_GetTick());
		return HAL_ERROR;
	}

	if (buffer_adxl_id[0] != 0xAD)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect DEVID_AD (Expected: 0xAD, Received: %x)\r\n", HAL_GetTick(), buffer_adxl_id[0]);
	}
	if (buffer_adxl_id[1] != 0x1D)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect DEVID_MST (Expected: 0x1D, Received: %x)\r\n", HAL_GetTick(), buffer_adxl_id[1]);
	}
	if (buffer_adxl_id[2] != 0xED)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect PARTID (Expected: 0xED, Received: %x)\r\n", HAL_GetTick(), buffer_adxl_id[2]);
	}

	// Filter
	uint8_t hpf = 0b000; // HPF off
	uint8_t lpf = 0b0000; // ODR 4000 Hz LPF 1000 Hz
	if (adxl_write_reg(hspi, ADXL_REG_FILTER, (hpf << 4) | lpf) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// INT_MAP
	// DATA_RDY on INT1
	if (adxl_write_reg(hspi, ADXL_REG_INT_MAP, 0b00000001) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Sync
	// TODO: EXT_SYNC
	if (adxl_write_reg(hspi, ADXL_REG_SYNC, 0b00000001) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// POWER_CTL
	// Measurement mode
	if (adxl_write_reg(hspi, ADXL_REG_POWER_CTL, 0b00000000) != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Prepare buffer for data request
	for (uint8_t i = 0; i < sizeof(adxl_request_buffer); i++)
	{
		adxl_request_buffer[i] = 0;
	}
	adxl_request_buffer[0] = (ADXL_REG_TEMP2 << 1) | 1;

	return HAL_OK;
}

HAL_StatusTypeDef adxl_request_data(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	// Send adxl_request_buffer
	if (HAL_SPI_TransmitReceive_DMA(hspi, adxl_request_buffer, adxl_data_buffer, sizeof(adxl_data_buffer)) != HAL_OK)
	{
		// TODO: Set hadxl state
		return HAL_ERROR;
	}
	return HAL_OK;
}

adxl_data_t adxl_rx_callback()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);

	adxl_data_t data;
	data.temp = ((uint16_t)(adxl_data_buffer[1] & 0b00001111) << 8) | (adxl_data_buffer[2]);
	data.x = ((uint32_t)adxl_data_buffer[3] << 12) | ((uint32_t)adxl_data_buffer[4] << 4) | (adxl_data_buffer[5] >> 4);
	data.y = ((uint32_t)adxl_data_buffer[6] << 12) | ((uint32_t)adxl_data_buffer[7] << 4) | (adxl_data_buffer[8] >> 4);
	data.z = ((uint32_t)adxl_data_buffer[9] << 12) | ((uint32_t)adxl_data_buffer[10] << 4) | (adxl_data_buffer[11] >> 4);

	// 20-bit sign -> 32-bit sign
	if (data.x & 0x80000)
	{
		data.x -= 0x100000;
	}
	if (data.y & 0x80000)
	{
		data.y -= 0x100000;
	}
	if (data.z & 0x80000)
	{
		data.z -= 0x100000;
	}

	return data;
}
