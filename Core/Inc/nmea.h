/*
 * nmea.h
 *
 * UART (RS-232) NMEA driver for u-blox 8 (Navilock 62529)
 *
 *  Created on: Aug 16, 2024
 *      Author: mirco
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "config.h"
#include "stm32f7xx_hal.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define NMEA_TALKER_GPS 'P'
#define NMEA_TALKER_GLONASS 'L'
#define NMEA_TALKER_GALILEO 'A'
#define NMEA_TALKER_BEIDOU 'B'
#define NMEA_TALKER_ANY 'N'

#define NMEA_MODE_AUTONOMOUS 'A'
#define NMEA_MODE_DIFFERENTIAL 'D'
#define NMEA_MODE_ESTIMATED 'E'
#define NMEA_MODE_MANUAL 'M'
#define NMEA_MODE_NOT_VALID 'N'

#define NMEA_DATE_WAIT_DURATION 120000

typedef struct
{
	UART_HandleTypeDef *huart;
	uint32_t timeout;
	uint32_t baud;
	uint8_t sampling_rate;
	volatile char dma_buffer;
	volatile char rx_buffer[1000];
	volatile uint16_t rx_buffer_write_index;
	volatile char line_buffer[1000];
	volatile uint8_t line_ready;
	volatile uint8_t overflowing;
} NMEA_t;

typedef struct
{
	char talker;
	// position
	uint8_t position_valid;
	float lat, lon;
	// speed
	uint8_t speed_valid;
	float speed_kmh;
	// altitude
	uint8_t altitude_valid;
	float altitude;
	// date
	uint8_t date_valid;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	// time
	uint8_t time_valid;
	uint8_t hour;
	uint8_t minute;
	float second;
} NMEA_Data_t;

extern const char nmea_pformat_rmc[];

typedef struct
{
	float time;
	char status;
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	float speed_kn;
	float track;
	int32_t date;
	float magvar;
	char magvar_dir;
	char mode;
} NMEA_Packet_RMC_t;

extern const char nmea_pformat_gll[];

typedef struct
{
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	float time;
	char status;
	char mode;
} NMEA_Packet_GLL_t;

extern const char nmea_pformat_vtg[];

typedef struct
{
	float track;
	char T;
	float track_mag;
	char M;
	float speed_kn;
	char N;
	float speed_kmh;
	char K;
	char mode;
} NMEA_Packet_VTG_t;

// header = Class, ID, Length
#define NMEA_UBX_RATE_HEADER (0x06 | (0x08 << 8) | (6 << 16))
typedef struct
{
	uint16_t measRate;
	uint16_t navRate;
	uint16_t timeRef;
} NMEA_UBX_RATE_t;

#define NMEA_UBX_PRT_HEADER (0x06 | (0x00 << 8) | (20 << 16))
typedef struct
{
	uint8_t portID;
	uint16_t txReady;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint16_t reserved;
} NMEA_UBX_PRT_t;

HAL_StatusTypeDef NMEA_SendPUBX(NMEA_t *hnmea, char *msg_buffer);
HAL_StatusTypeDef NMEA_SendUBX(NMEA_t *hnmea, uint32_t header, void *packet, size_t packet_size);
HAL_StatusTypeDef NMEA_Init(NMEA_t *hnmea);
NMEA_Data_t NMEA_GetDate(NMEA_t *hnmea);
HAL_StatusTypeDef NMEA_ProcessChar(NMEA_t *hnmea);
NMEA_Data_t NMEA_ProcessLine(NMEA_t *hnmea);

#endif /* INC_NMEA_H_ */
