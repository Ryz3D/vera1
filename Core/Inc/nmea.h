/*
 * nmea.h
 *
 * UART (RS-232) NMEA driver for Navilock 62529 (u-blox 8)
 *
 *  Created on: Aug 16, 2024
 *      Author: mirco
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "config.h"
#include "stm32f7xx_hal.h"

#include <stdio.h>
#include <string.h>

#define NMEA_DATE_ATTEMPT_DURATION 10000

typedef struct
{
	UART_HandleTypeDef *huart;
	uint32_t baud;
	uint8_t sampling_rate;
	char dma_buffer;
	char rx_buffer[1000];
	uint16_t rx_buffer_write_index;
	char line_buffer[1000];
	uint8_t line_ready;
} NMEA_t;

typedef struct
{
	// position
	uint8_t pos_valid;
	float lat, lon;
	char lat_dir, lon_dir;
	// speed
	uint8_t speed_valid;
	float speed_kmh;
	// altitude
	uint8_t altitude_valid;
	float altitude;
	// date
	uint8_t date_valid;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	// time
	uint8_t time_valid;
	float time;
} NMEA_Data_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	float second;
} NMEA_Time_t;

typedef struct
{
// TODO
} NMEA_PACKET_RMC;

HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea);
NMEA_Data_t NMEA_GetDate(NMEA_t *hnmea);
HAL_StatusTypeDef NMEA_ProcessChar(NMEA_t *hnmea);
NMEA_Data_t NMEA_ProcessLine(NMEA_t *hnmea);

NMEA_Time_t NMEA_ParseTime(float time);

#endif /* INC_NMEA_H_ */
