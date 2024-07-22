/*
 * data_points.h
 *
 *  Created on: Jul 1, 2024
 *      Author: mirco
 */

#ifndef INC_DATA_POINTS_H_
#define INC_DATA_POINTS_H_

#include "config.h"

#define A_COMPLETE_TIMESTAMP (1 << 0)
#define A_COMPLETE_MEMS (1 << 1)
#define A_COMPLETE_PZ (1 << 2)

#define P_COMPLETE_TIMESTAMP (1 << 0)
#define P_COMPLETE_GPS (1 << 1)

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	uint16_t temp_mems1;
	int32_t x_mems1;
	int32_t y_mems1;
	int32_t z_mems1;
	uint16_t a_piezo[PIEZO_COUNT];
} a_data_point_t;

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	uint32_t gps_time;
	double height;
	double speed;
	double lat;
	double lon;
} p_data_point_t;

#endif /* INC_DATA_POINTS_H_ */
