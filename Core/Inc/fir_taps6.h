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

 * 0 Hz - 400 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 500 Hz - 800 Hz
 gain = 0.25
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 1000 Hz - 1500 Hz
 gain = 0.5
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 1700 Hz - 3000 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 2.3198 dB

 * 3200 Hz - 8000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -44.9924 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	-288,
	-469,
	-462,
	-140,
	315,
	565,
	422,
	40,
	-232,
	-190,
	48,
	217,
	208,
	158,
	231,
	391,
	448,
	305,
	96,
	17,
	99,
	186,
	165,
	116,
	180,
	327,
	356,
	149,
	-143,
	-258,
	-148,
	-32,
	-104,
	-255,
	-219,
	33,
	184,
	-61,
	-551,
	-810,
	-599,
	-225,
	-154,
	-382,
	-397,
	129,
	801,
	883,
	235,
	-380,
	-147,
	755,
	1281,
	833,
	162,
	551,
	2044,
	2932,
	1517,
	-1566,
	-3279,
	-994,
	4505,
	9158,
	9158,
	4505,
	-994,
	-3279,
	-1566,
	1517,
	2932,
	2044,
	551,
	162,
	833,
	1281,
	755,
	-147,
	-380,
	235,
	883,
	801,
	129,
	-397,
	-382,
	-154,
	-225,
	-599,
	-810,
	-551,
	-61,
	184,
	33,
	-219,
	-255,
	-104,
	-32,
	-148,
	-258,
	-143,
	149,
	356,
	327,
	180,
	116,
	165,
	186,
	99,
	17,
	96,
	305,
	448,
	391,
	231,
	158,
	208,
	217,
	48,
	-190,
	-232,
	40,
	422,
	565,
	315,
	-140,
	-462,
	-469,
	-288
};

#endif /* INC_FIR_TAPS_H_ */
