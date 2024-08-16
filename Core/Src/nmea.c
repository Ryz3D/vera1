/*
 * nmea.c
 *
 * UART (RS-232) NMEA driver for Navilock 62529 (u-blox 8)
 *
 *  Created on: Aug 16, 2024
 *      Author: mirco
 */

#include "nmea.h"

HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea)
{
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

	// Receive char from UART
	// TODO: send 115200 baud request (after receiving good data), test with/without automatic baud
	// UBX-CFG-PRT
	// UBX-CFG-RATE
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
	printf("(%lu) Waiting for GNSS date\r\n", HAL_GetTick());

	NMEA_Data_t data;
	// Wait for RMC packet
	// $GPRMC,HHMMSS,A,BBBB.BBBB,b,LLLLL.LLLL,l,GG.G,RR.R,DDMMYY,M.M,m,F*PP
	for (uint32_t i = 0; i < NMEA_DATE_ATTEMPT_DURATION; i++)
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
			}
			if (new_data.time_valid)
			{
				data.time_valid = 1;
				data.time = new_data.time;
			}
			if (data.date_valid && data.time_valid)
			{
				break;
			}
		}
		HAL_Delay(1);
	}

	return data;
}

HAL_StatusTypeDef NMEA_ProcessChar(NMEA_t *hnmea)
{
	if (hnmea->dma_buffer == '\n')
	{
		if (hnmea->rx_buffer_write_index > 1)
		{
			memcpy(hnmea->line_buffer, hnmea->rx_buffer, hnmea->rx_buffer_write_index);
			hnmea->line_buffer[hnmea->rx_buffer_write_index] = '\0';
			hnmea->line_ready = 1;
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
		.pos_valid = 0,
		.speed_valid = 0,
		.altitude_valid = 0,
		.date_valid = 0,
		.time_valid = 0,
	};

	if (strncmp("$G", hnmea->line_buffer, 2) == 0)
	{
		char system = hnmea->line_buffer[1];
		/*
		 mode:
		 A Autonomous
		 D Differential
		 E Estimated (dead reckoning) mode
		 M Manual input
		 N Data not valid
		 */
		/*
		 $GNRMC,225340.00,V,,,,,,,160824,,,N*68
		 $GNVTG,,,,,,,,,N*2E
		 $GNGGA,225340.00,,,,,0,05,83.80,,,,,,*7C
		 $GNGSA,A,1,25,32,28,,,,,,,,,,99.99,83.80,91.68*27
		 $GNGSA,A,1,76,67,,,,,,,,,,,99.99,83.80,91.68*2B
		 $GPGSV,3,1,10,03,03,358,,06,24,057,,11,24,104,,12,74,061,*72
		 $GPGSV,3,2,10,19,13,043,,24,35,144,,25,66,275,26,28,24,306,29*74
		 $GPGSV,3,3,10,29,30,200,,32,39,273,30*7C
		 $GLGSV,3,1,10,65,01,215,,66,27,254,,67,28,314,26,68,04,356,*67
		 $GLGSV,3,2,10,75,40,085,,76,66,326,30,77,23,292,19,84,16,022,*60
		 $GLGSV,3,3,10,85,49,077,,86,30,147,*6B
		 $GNGLL,,,,,225340.00,V,N*56
		 */
		if (strncmp("GGA", hnmea->line_buffer + 3, 3) == 0)
		{
			// TODO: altitude
		}
		if (strncmp("GLL", hnmea->line_buffer + 3, 3) == 0)
		{
			// TODO: More efficient (write types in char array, struct for each packet type, create struct in if, call function for parsing with void*)
			char lat_dir = '?', lon_dir = '?', status = 'V', mode = 'N';
			unsigned int checksum = 0;
			// TODO: Optional kann zusätzlich eine durch ein „*“ abgetrennte hexadezimale Prüfzahl angehängt werden. Diese wird durch die XOR-Verknüpfung der ASCII-Werte aller Zeichen zwischen dem $ und dem * errechnet.
			sscanf(hnmea->line_buffer + 3, "GLL,%f,%c,%f,%c,%f,%c,%c*%X", &data.lat, &lat_dir, &data.lon, &lon_dir, &data.time, &status, &mode, &checksum);
			if (mode != 'N')
			{
				data.pos_valid = 1;
				data.time_valid = 1;

				data.lat /= 100;
				data.lon /= 100;
				if (lat_dir == 'S')
				{
					data.lat *= -1;
				}
				if (lon_dir == 'W')
				{
					data.lon *= -1;
				}
				// WGS84 Deg/Min https://coordinates-converter.com/en/decimal/50.342025,8.753487?karte=OpenStreetMap&zoom=17
			}
		}
		if (strncmp("VTG", hnmea->line_buffer + 3, 3) == 0)
		{
			float track = 0, track_mag = 0, speed_kn = 0;
			char T = '?', M = '?', N = '?', K = '?', mode = 'N';
			unsigned int checksum = 0;
			sscanf(hnmea->line_buffer + 3, "VTG,,%c,,%c,%f,%c,%f,%c,%c*%X", &T, &M, &speed_kn, &N, &data.speed_kmh, &K, &mode, &checksum);
			// sscanf(gps_line_buffer + 3, "VTG,%f,%c,%f,%c,%f,%c,%f,%c,%c*%X", &track, &T, &track_mag, &M, &speed_kn, &N, &speed_kmh, &K, &mode, &checksum);

			if (mode != 'N')
			{
				data.speed_valid = 1;
			}
		}
	}
	hnmea->line_ready = 0;

	return data;
}

NMEA_Time_t NMEA_ParseTime(float time)
{
	NMEA_Time_t s_time;
	s_time.hour = (uint32_t)time / 10000;
	s_time.minute = ((uint32_t)time / 100) % 100;
	s_time.second = fmod(time, 100);
	return s_time;
}
