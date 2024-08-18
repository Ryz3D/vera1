/*
 * nmea.c
 *
 * UART (RS-232) NMEA driver for u-blox 8 (Navilock 62529)
 *
 *  Created on: Aug 16, 2024
 *      Author: mirco
 */

#include "nmea.h"

const char nmea_pformat_rmc[] = "fcfcfcffifcc";
const char nmea_pformat_gll[] = "fcfcfcc";
const char nmea_pformat_vtg[] = "fcfcfcfcc";

void NMEA_ParsePacket(NMEA_t *hnmea, void *packet_buffer, const char format[]);
uint8_t NMEA_ChecksumValid(NMEA_t *hnmea);
uint8_t NMEA_Hex2Dec(char c);
void NMEA_ConvertTime(NMEA_Data_t *data, float time);
void NMEA_ConvertDate(NMEA_Data_t *data, int32_t date);
void NMEA_ConvertLatLon(NMEA_Data_t *data, float lat, char lat_dir, float lon, char lon_dir);

HAL_StatusTypeDef NMEA_SendPUBX(NMEA_t *hnmea, char *msg_buffer)
{
	char tx_buffer[100];
	sprintf(tx_buffer, "$PUBX,%s*00\r\n", msg_buffer);
	uint32_t tx_len = strlen(tx_buffer);
	uint8_t checksum = 0;
	for (uint32_t i = 1; i < tx_len - 5; i++)
	{
		checksum ^= tx_buffer[i];
	}
	tx_buffer[tx_len - 4] = '0' + checksum / 16;
	tx_buffer[tx_len - 3] = '0' + checksum % 16;
	if (tx_buffer[tx_len - 4] > '9')
	{
		tx_buffer[tx_len - 4] += 'A' - '0' - 10;
	}
	if (tx_buffer[tx_len - 3] > '9')
	{
		tx_buffer[tx_len - 3] += 'A' - '0' - 10;
	}
	if (HAL_UART_Transmit(hnmea->huart, (uint8_t*)tx_buffer, tx_len, hnmea->timeout) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef NMEA_SendUBX(NMEA_t *hnmea, uint32_t header, void *packet, size_t packet_size)
{
	uint8_t tx_buffer[100] = {
		0xB5, 0x62 };
	uint32_t tx_len = packet_size + 8;
	*(uint32_t*)(tx_buffer + 2) = header;
	memcpy(tx_buffer + 6, packet, packet_size);
	tx_buffer[tx_len - 2] = 0;
	tx_buffer[tx_len - 1] = 0;
	for (uint8_t i = 2; i < tx_len - 2; i++)
	{
		tx_buffer[tx_len - 2] += tx_buffer[i];
		tx_buffer[tx_len - 1] += tx_buffer[tx_len - 2];
	}
	if (HAL_UART_Transmit(hnmea->huart, tx_buffer, tx_len, hnmea->timeout) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea)
{
	if (hnmea->timeout == 0)
	{
		hnmea->timeout = 1000;
	}
	if (hnmea->baud == 0)
	{
		hnmea->baud = 9600;
	}
	if (hnmea->sampling_rate == 0)
	{
		hnmea->sampling_rate = 10;
	}
	hnmea->rx_buffer_write_index = 0;
	hnmea->line_ready = 0;
	hnmea->overflowing = 0;

	char pubx_buffer[100];

	// Set baud rate
	uint16_t inProto = 0b000011; // Module should accept NMEA and UBX via UART
	uint16_t outProto = 0b000010; // Module should transmit NMEA via UART
	sprintf(pubx_buffer, "41,1,%04hX,%04hX,%lu,0", inProto, outProto, hnmea->baud);
	if (NMEA_SendPUBX(hnmea, pubx_buffer) == HAL_ERROR)
	{
		printf("(%lu) ERROR: NMEA_Init: NMEA_SendPUBX failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	hnmea->huart->Init.BaudRate = hnmea->baud;
	if (HAL_UART_Init(hnmea->huart) != HAL_OK)
	{
		printf("(%lu) ERROR: NMEA_Init: HAL_UART_Init failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}
	printf("(%lu) Set NMEA baud rate to %lu\r\n", HAL_GetTick(), hnmea->baud);
	HAL_Delay(100);

	// Set output data rate
	NMEA_UBX_RATE_t ubx_rate = {
		.measRate = 1000 / hnmea->sampling_rate,
		.navRate = 1,
		.timeRef = 0,
	};
	NMEA_SendUBX(hnmea, NMEA_UBX_RATE_HEADER, &ubx_rate, sizeof(ubx_rate));

	// TODO: Switch to Galileo for accuracy?
	if (HAL_UART_Receive_DMA(hnmea->huart, (uint8_t*)&hnmea->dma_buffer, sizeof(char)) != HAL_OK)
	{
		printf("(%lu) ERROR: NMEA_Init: HAL_UART_Receive_DMA failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	return HAL_OK;
}

NMEA_Data_t NMEA_GetDate(NMEA_t *hnmea)
{
	printf("(%lu) Waiting for GNSS date (timeout after %i s)...\r\n", HAL_GetTick(), NMEA_DATE_WAIT_DURATION / 1000);

	NMEA_Data_t data = {
		.position_valid = 0,
		.speed_valid = 0,
		.altitude_valid = 0,
		.date_valid = 0,
		.time_valid = 0,
	};
	uint32_t time_end = HAL_GetTick() + NMEA_DATE_WAIT_DURATION;
	while (HAL_GetTick() <= time_end)
	{
		if (hnmea->line_ready)
		{
			NMEA_Data_t new_data = NMEA_ProcessLine(hnmea);
			if (new_data.date_valid)
			{
				data.date_valid = 1;
				data.year = new_data.year;
				data.month = new_data.month;
				data.day = new_data.day;
				break;
			}
		}
	}

	return data;
}

HAL_StatusTypeDef NMEA_ProcessChar(NMEA_t *hnmea)
{
	if (hnmea->dma_buffer == '\n')
	{
		if (hnmea->rx_buffer_write_index > 1)
		{
			if (hnmea->line_ready)
			{
				// TODO: circular buffering -> all messages come at once
				// hnmea->overflowing = 1;
			}
			else
			{
				// Remove \r
				memcpy((void*)hnmea->line_buffer, (void*)hnmea->rx_buffer, hnmea->rx_buffer_write_index - 1);
				hnmea->line_buffer[hnmea->rx_buffer_write_index - 1] = '\0';
				hnmea->line_ready = 1;
			}
		}
		hnmea->rx_buffer_write_index = 0;
	}
	else
	{
		if (hnmea->dma_buffer == '$' || hnmea->rx_buffer_write_index == sizeof(hnmea->rx_buffer))
		{
			hnmea->rx_buffer_write_index = 0;
		}
		hnmea->rx_buffer[hnmea->rx_buffer_write_index++] = hnmea->dma_buffer;
	}

	if (HAL_UART_Receive_DMA(hnmea->huart, (uint8_t*)&hnmea->dma_buffer, sizeof(char)) != HAL_OK)
	{
		printf("(%lu) ERROR: NMEA_ProcessChar: HAL_UART_Receive_DMA failed\r\n", HAL_GetTick());
		return HAL_ERROR;
	}

	return HAL_OK;
}

NMEA_Data_t NMEA_ProcessLine(NMEA_t *hnmea)
{
	NMEA_Data_t data = {
		.position_valid = 0,
		.speed_valid = 0,
		.altitude_valid = 0,
		.date_valid = 0,
		.time_valid = 0,
	};

	if (strncmp("$G", (char*)hnmea->line_buffer, 2) == 0)
	{
		if (NMEA_ChecksumValid(hnmea))
		{
			data.talker = hnmea->line_buffer[2];
			if (strncmp("RMC", (char*)hnmea->line_buffer + 3, 3) == 0)
			{
				NMEA_Packet_RMC_t packet;
				NMEA_ParsePacket(hnmea, &packet, nmea_pformat_rmc);

				if (packet.date > 0)
				{
					data.date_valid = 1;
					NMEA_ConvertDate(&data, packet.date);
				}
				if (packet.time > 0)
				{
					data.time_valid = 1;
					NMEA_ConvertTime(&data, packet.time);
				}
				if (packet.mode != 'N')
				{
					data.position_valid = 1;
					NMEA_ConvertLatLon(&data, packet.lat, packet.lat_dir, packet.lon, packet.lon_dir);

					data.speed_valid = 1;
					data.speed_kmh = packet.speed_kn * 1.852f;
				}
			}
			// TODO: altitude packet GGA
			/*
			 // GLL
			 if (strncmp("GLL", (char*)hnmea->line_buffer + 3, 3) == 0)
			 {
			 NMEA_Packet_GLL_t packet;
			 NMEA_ParsePacket(hnmea, &packet, nmea_pformat_gll);

			 if (packet.time > 0)
			 {
			 data.time_valid = 1;
			 NMEA_ConvertTime(&data, packet.time);
			 }
			 if (packet.mode != 'N')
			 {
			 data.pos_valid = 1;
			 NMEA_ConvertLatLon(&data, packet.lat, packet.lat_dir, packet.lon, packet.lon_dir);
			 }
			 }
			 */
			/*
			 // VTG
			 if (strncmp("VTG", (char*)hnmea->line_buffer + 3, 3) == 0)
			 {
			 NMEA_Packet_VTG_t packet;
			 NMEA_ParsePacket(hnmea, &packet, nmea_pformat_vtg);

			 if (packet.mode != 'N')
			 {
			 data.speed_valid = 1;
			 data.speed_kmh = packet.speed_kmh;
			 }
			 }
			 */
		}
		else
		{
			printf("WARNING: NMEA_ProcessLine: Invalid checksum in %s\r\n", hnmea->line_buffer);
		}
	}
	hnmea->line_ready = 0;

	return data;
}

uint8_t NMEA_ChecksumValid(NMEA_t *hnmea)
{
	uint8_t checksum_calc = 0;
	for (uint16_t i = 1; hnmea->line_buffer[i] != '\0'; i++)
	{
		if (hnmea->line_buffer[i] == '*')
		{
			uint8_t checksum = NMEA_Hex2Dec(hnmea->line_buffer[i + 1]) << 4 | NMEA_Hex2Dec(hnmea->line_buffer[i + 2]);
			if (checksum == checksum_calc)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		checksum_calc ^= hnmea->line_buffer[i];
	}
	return 0;
}

uint8_t NMEA_Hex2Dec(char c)
{
	if (c >= '0' && c <= '9')
	{
		return c - '0';
	}
	if (c >= 'A' && c <= 'F')
	{
		return c + 10 - 'A';
	}
	if (c >= 'a' && c <= 'f')
	{
		return c + 10 - 'a';
	}
	return 0;
}

char temp_buf[1000];

void NMEA_ParsePacket(NMEA_t *hnmea, void *packet_buffer, const char format[])
{
	// Skip message header
	volatile char *line_pointer = hnmea->line_buffer;
	while (*line_pointer != ',' && *line_pointer != '\0')
	{
		line_pointer++;
	}
	line_pointer++;

	// Find end of string to compare against
	volatile char *line_end_pointer = hnmea->line_buffer + strlen((char*)hnmea->line_buffer);
	// Pointer to currently processed character
	void *buffer_pointer = packet_buffer;
	for (uint8_t i_format = 0; i_format < strlen(format) && line_pointer < line_end_pointer; i_format++)
	{
		// Pointer to end of strtol/strtof conversion
		char *end_pointer;
		switch (format[i_format])
		{
		case 'i':
			// If pointer is unaligned for data type
			if ((uintptr_t)buffer_pointer % sizeof(int32_t) != 0)
			{
				// Align pointer (to match address of member in struct)
				buffer_pointer += sizeof(int32_t) - (uintptr_t)buffer_pointer % sizeof(int32_t);
			}
			// Check if value is present
			if (*line_pointer == ',')
			{
				*(int32_t*)buffer_pointer = 0;
			}
			else
			{
				*(int32_t*)buffer_pointer = strtol((char*)line_pointer, &end_pointer, 10);
				line_pointer = end_pointer;
			}
			line_pointer++;
			buffer_pointer += sizeof(int32_t);
			break;
		case 'x':
			if ((uintptr_t)buffer_pointer % sizeof(int32_t) != 0)
			{
				buffer_pointer += sizeof(int32_t) - (uintptr_t)buffer_pointer % sizeof(int32_t);
			}
			if (*line_pointer == ',')
			{
				*(int32_t*)buffer_pointer = 0;
			}
			else
			{
				*(int32_t*)buffer_pointer = strtol((char*)line_pointer, &end_pointer, 16);
				line_pointer = end_pointer;
			}
			line_pointer++;
			buffer_pointer += sizeof(int32_t);
			break;
		case 'f':
			if ((uintptr_t)buffer_pointer % sizeof(float) != 0)
			{
				buffer_pointer += sizeof(float) - (uintptr_t)buffer_pointer % sizeof(float);
				// https://stackoverflow.com/questions/4840410/how-to-align-a-pointer-in-c
				// buffer_pointer = (~(uintptr_t)buffer_pointer + 1) & (sizeof(float) - 1);
			}
			if (*line_pointer == ',')
			{
				*(float*)buffer_pointer = 0;
			}
			else
			{
				*(float*)buffer_pointer = strtof((char*)line_pointer, &end_pointer);
				line_pointer = end_pointer;
			}
			line_pointer++;
			buffer_pointer += sizeof(float);
			break;
		case 'c':
			if (*line_pointer == ',')
			{
				*(char*)buffer_pointer = '\0';
			}
			else
			{
				*(char*)buffer_pointer = *line_pointer;
				line_pointer++;
			}
			line_pointer++;
			buffer_pointer += sizeof(char);
			break;
		default:
			printf("(%lu) WARNING: NMEA_ParsePacket: Unknown format %c\r\n", HAL_GetTick(), format[i_format]);
			while (*line_pointer != ',' && *line_pointer != '\0')
			{
				line_pointer++;
			}
			line_pointer++;
			buffer_pointer++;
			break;
		}
	}
}

void NMEA_ConvertTime(NMEA_Data_t *data, float time)
{
	data->hour = (uint32_t)time / 10000;
	data->minute = ((uint32_t)time / 100) % 100;
	data->second = fmod(time, 100);
}

void NMEA_ConvertDate(NMEA_Data_t *data, int32_t date)
{
	data->day = date / 10000;
	data->month = (date / 100) % 100;
	data->year = date % 100;
}

void NMEA_ConvertLatLon(NMEA_Data_t *data, float lat, char lat_dir, float lon, char lon_dir)
{
	lat /= 100.0f;
	lon /= 100.0f;

	int lat_deg = (int)lat;
	int lon_deg = (int)lon;
	float lat_minutes = lat - lat_deg;
	float lon_minutes = lon - lon_deg;
	data->lat = lat_deg + lat_minutes / 0.6f;
	data->lon = lon_deg + lon_minutes / 0.6f;

	if (lat_dir == 'S')
	{
		data->lat *= -1;
	}
	if (lon_dir == 'W')
	{
		data->lon *= -1;
	}
}
