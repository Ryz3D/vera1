#include "adxl357.h"

HAL_StatusTypeDef adxl357_init(SPI_HandleTypeDef *hspi)
{
	// TODO: setup 4kHz ODR, filter to highest frequency,
	const uint8_t spi_buf[] = { 0x00 };
	// HAL_SPI_Transmit(hspi, spi_buf, sizeof(spi_buf), HAL_MAX_DELAY);

	return HAL_OK;
}

HAL_StatusTypeDef adxl357_read(SPI_HandleTypeDef *hspi, int32_t *x, int32_t *y, int32_t *z, uint16_t *temp)
{
	// TODO: SPI Interrupt -> call when data is ready (or hardware interrupt on DRDY pin?)
	const uint8_t spi_buf[] = { 0x06 };
	HAL_SPI_Transmit(hspi, spi_buf, sizeof(spi_buf), HAL_MAX_DELAY);

	const uint8_t spi_rx_buf[12];
	HAL_SPI_Receive(hspi, spi_rx_buf, sizeof(spi_rx_buf), HAL_MAX_DELAY);

	if (temp != NULL)
		*temp = ((uint16_t)(spi_rx_buf[0] & 0b00001111) << 8) | (spi_rx_buf[1]);
	if (x != NULL)
		*x = ((uint32_t)spi_rx_buf[2] << 12) | ((uint32_t)spi_rx_buf[3] << 4) | (spi_rx_buf[4] >> 4);
	if (y != NULL)
		*y = ((uint32_t)spi_rx_buf[5] << 12) | ((uint32_t)spi_rx_buf[6] << 4) | (spi_rx_buf[7] >> 4);
	if (z != NULL)
		*z = ((uint32_t)spi_rx_buf[8] << 12) | ((uint32_t)spi_rx_buf[9] << 4) | (spi_rx_buf[10] >> 4);
	uint8_t fifo = spi_rx_buf[11];

	return HAL_OK;
}
