#include <adxl.h>

HAL_StatusTypeDef adxl_get_reg(SPI_HandleTypeDef *hspi, uint8_t addr)
{
	return HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef adxl_set_reg(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t val)
{
	uint8_t spi_buf[] = { addr, val };
	return HAL_SPI_Transmit(hspi, spi_buf, sizeof(spi_buf), HAL_MAX_DELAY);
}

HAL_StatusTypeDef adxl_init(SPI_HandleTypeDef *hspi)
{
	HAL_StatusTypeDef res;

	if ((res = adxl_get_reg(hspi, 0x00)) != HAL_OK)
	{
		return res;
	}
	uint8_t spi_rx_buf[3];
	if ((res = HAL_SPI_Receive(hspi, spi_rx_buf, sizeof(spi_rx_buf), HAL_MAX_DELAY)) != HAL_OK)
	{
		return res;
	}

	if (spi_rx_buf[0] != 0xAD)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect DEVID_AD (Expected: 0xAD, Received: %x)\r\n", HAL_GetTick(), spi_rx_buf[0]);
	}
	if (spi_rx_buf[1] != 0x1D)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect DEVID_MST (Expected: 0x1D, Received: %x)\r\n", HAL_GetTick(), spi_rx_buf[1]);
	}
	if (spi_rx_buf[2] != 0xED)
	{
		printf("(%lu) WARNING: adxl_init: Incorrect PARTID (Expected: 0xED, Received: %x)\r\n", HAL_GetTick(), spi_rx_buf[2]);
	}

	// Filter
	uint8_t hpf = 0b000; // HPF off
	uint8_t lpf = 0b0000; // ODR 4000 Hz LPF 1000 Hz
	if ((res = adxl_set_reg(hspi, 0x28, (hpf << 4) | lpf)) != HAL_OK)
	{
		return res;
	}

	// INT_MAP
	// DATA_RDY on INT1
	if ((res = adxl_set_reg(hspi, 0x2A, 0b00000001)) != HAL_OK)
	{
		return res;
	}

	// Sync
	// TODO: EXT_SYNC
	if ((res = adxl_set_reg(hspi, 0x2B, 0b00000001)) != HAL_OK)
	{
		return res;
	}

	// POWER_CTL
	// Measurement mode
	if ((res = adxl_set_reg(hspi, 0x2D, 0b00000000)) != HAL_OK)
	{
		return res;
	}

	return HAL_OK;
}

HAL_StatusTypeDef adxl_read(SPI_HandleTypeDef *hspi, int32_t *x, int32_t *y, int32_t *z, uint16_t *temp)
{
	// TODO: SPI Interrupt -> call when data is ready (or hardware interrupt on DRDY pin?)
	uint8_t spi_buf[] = { 0x06 };
	HAL_SPI_Transmit(hspi, spi_buf, sizeof(spi_buf), HAL_MAX_DELAY);

	uint8_t spi_rx_buf[12];
	HAL_SPI_Receive(hspi, spi_rx_buf, sizeof(spi_rx_buf), HAL_MAX_DELAY);

	if (temp != NULL)
		*temp = ((uint16_t)(spi_rx_buf[0] & 0b00001111) << 8) | (spi_rx_buf[1]);
	if (x != NULL)
		*x = ((uint32_t)spi_rx_buf[2] << 12) | ((uint32_t)spi_rx_buf[3] << 4) | (spi_rx_buf[4] >> 4);
	if (y != NULL)
		*y = ((uint32_t)spi_rx_buf[5] << 12) | ((uint32_t)spi_rx_buf[6] << 4) | (spi_rx_buf[7] >> 4);
	if (z != NULL)
		*z = ((uint32_t)spi_rx_buf[8] << 12) | ((uint32_t)spi_rx_buf[9] << 4) | (spi_rx_buf[10] >> 4);
	// uint8_t fifo = spi_rx_buf[11];

	return HAL_OK;
}
