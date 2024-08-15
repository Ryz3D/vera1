/*
 * fir_taps.h
 *
 *  Created on: Jul 29, 2024
 *      Author: mirco
 */

#ifndef INC_FIR_TAPS_H_
#define INC_FIR_TAPS_H_

/*

 FIR filter designed with
 http://t-filter.appspot.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 1600 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.5866 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -90 dB
 actual attenuation = -92.1561 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	0,
	-1,
	-6,
	-16,
	-33,
	-57,
	-86,
	-111,
	-125,
	-121,
	-93,
	-46,
	11,
	61,
	87,
	81,
	41,
	-17,
	-73,
	-101,
	-88,
	-34,
	41,
	106,
	131,
	99,
	18,
	-81,
	-156,
	-168,
	-104,
	16,
	144,
	221,
	207,
	94,
	-77,
	-235,
	-304,
	-241,
	-56,
	180,
	364,
	404,
	258,
	-30,
	-349,
	-552,
	-525,
	-242,
	208,
	642,
	852,
	690,
	150,
	-604,
	-1269,
	-1497,
	-1024,
	223,
	2071,
	4130,
	5900,
	6922,
	6922,
	5900,
	4130,
	2071,
	223,
	-1024,
	-1497,
	-1269,
	-604,
	150,
	690,
	852,
	642,
	208,
	-242,
	-525,
	-552,
	-349,
	-30,
	258,
	404,
	364,
	180,
	-56,
	-241,
	-304,
	-235,
	-77,
	94,
	207,
	221,
	144,
	16,
	-104,
	-168,
	-156,
	-81,
	18,
	99,
	131,
	106,
	41,
	-34,
	-88,
	-101,
	-73,
	-17,
	41,
	81,
	87,
	61,
	11,
	-46,
	-93,
	-121,
	-125,
	-111,
	-86,
	-57,
	-33,
	-16,
	-6,
	-1,
	0
};

#endif /* INC_FIR_TAPS_H_ */
