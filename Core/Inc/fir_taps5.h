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

 * 0 Hz - 1700 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.8219 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -70 dB
 actual attenuation = -69.2299 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	-6,
	7,
	27,
	63,
	109,
	154,
	185,
	185,
	147,
	77,
	-7,
	-79,
	-113,
	-97,
	-38,
	41,
	102,
	118,
	77,
	-4,
	-90,
	-139,
	-123,
	-45,
	63,
	151,
	172,
	109,
	-15,
	-145,
	-216,
	-186,
	-60,
	111,
	244,
	269,
	161,
	-41,
	-246,
	-350,
	-290,
	-75,
	205,
	417,
	443,
	246,
	-105,
	-454,
	-622,
	-494,
	-87,
	439,
	836,
	872,
	449,
	-323,
	-1134,
	-1573,
	-1277,
	-94,
	1835,
	4083,
	6065,
	7223,
	7223,
	6065,
	4083,
	1835,
	-94,
	-1277,
	-1573,
	-1134,
	-323,
	449,
	872,
	836,
	439,
	-87,
	-494,
	-622,
	-454,
	-105,
	246,
	443,
	417,
	205,
	-75,
	-290,
	-350,
	-246,
	-41,
	161,
	269,
	244,
	111,
	-60,
	-186,
	-216,
	-145,
	-15,
	109,
	172,
	151,
	63,
	-45,
	-123,
	-139,
	-90,
	-4,
	77,
	118,
	102,
	41,
	-38,
	-97,
	-113,
	-79,
	-7,
	77,
	147,
	185,
	185,
	154,
	109,
	63,
	27,
	7,
	-6
};

#endif /* INC_FIR_TAPS_H_ */
