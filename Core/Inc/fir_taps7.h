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

 * 0 Hz - 1500 Hz
 gain = 1
 desired ripple = 0.5 dB
 actual ripple = 0.0782 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -80 dB
 actual attenuation = -93.5395 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	1,
	2,
	2,
	1,
	-2,
	-8,
	-16,
	-23,
	-27,
	-25,
	-15,
	1,
	19,
	31,
	31,
	18,
	-7,
	-34,
	-51,
	-49,
	-24,
	17,
	58,
	81,
	72,
	28,
	-37,
	-96,
	-123,
	-98,
	-24,
	74,
	154,
	177,
	123,
	4,
	-136,
	-237,
	-245,
	-143,
	41,
	237,
	355,
	329,
	148,
	-133,
	-399,
	-528,
	-434,
	-121,
	310,
	682,
	811,
	584,
	22,
	-701,
	-1294,
	-1439,
	-906,
	353,
	2159,
	4139,
	5826,
	6796,
	6796,
	5826,
	4139,
	2159,
	353,
	-906,
	-1439,
	-1294,
	-701,
	22,
	584,
	811,
	682,
	310,
	-121,
	-434,
	-528,
	-399,
	-133,
	148,
	329,
	355,
	237,
	41,
	-143,
	-245,
	-237,
	-136,
	4,
	123,
	177,
	154,
	74,
	-24,
	-98,
	-123,
	-96,
	-37,
	28,
	72,
	81,
	58,
	17,
	-24,
	-49,
	-51,
	-34,
	-7,
	18,
	31,
	31,
	19,
	1,
	-15,
	-25,
	-27,
	-23,
	-16,
	-8,
	-2,
	1,
	2,
	2,
	1
};

#endif /* INC_FIR_TAPS_H_ */
