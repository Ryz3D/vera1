/*
 * data_points.h
 *
 *  Created on: Jul 1, 2024
 *      Author: mirco
 */

#ifndef INC_DATA_POINTS_H_
#define INC_DATA_POINTS_H_

#define A_COMPLETE_MEMS (1 << 0)
#define A_COMPLETE_PZ1 (1 << 1)
#define A_COMPLETE_PZ2 (1 << 2)
#define P_COMPLETE_GPS (1 << 0)

#define A_DATA_POINT_FORMAT "M %li\tP %li\tP %li\tC %u\r\n"
#define A_DATA_POINT_VALUES(dp) dp.y_mems1, dp.a_piezo1, dp.a_piezo2, dp.complete

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	int32_t a_piezo1;
	int32_t a_piezo2;
	int32_t x_mems1;
	int32_t y_mems1;
	int32_t z_mems1;
} a_data_point_t;

typedef struct
{
	uint8_t complete;
	uint32_t timestamp;
	double lat;
	double lon;
	double height;
	double speed;
} p_data_point_t;

#endif /* INC_DATA_POINTS_H_ */
