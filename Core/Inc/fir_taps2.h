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

 * 0 Hz - 1800 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.6606 dB

 * 2000 Hz - 8000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -41.1246 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	221,
	162,
	155,
	100,
	13,
	-74,
	-125,
	-120,
	-63,
	16,
	75,
	82,
	30,
	-55,
	-130,
	-151,
	-103,
	-6,
	90,
	134,
	97,
	-8,
	-128,
	-199,
	-177,
	-67,
	80,
	187,
	191,
	81,
	-93,
	-240,
	-279,
	-176,
	24,
	222,
	309,
	226,
	1,
	-256,
	-407,
	-358,
	-112,
	217,
	457,
	465,
	206,
	-212,
	-578,
	-681,
	-421,
	117,
	680,
	956,
	728,
	7,
	-925,
	-1595,
	-1528,
	-456,
	1536,
	3990,
	6218,
	7541,
	7541,
	6218,
	3990,
	1536,
	-456,
	-1528,
	-1595,
	-925,
	7,
	728,
	956,
	680,
	117,
	-421,
	-681,
	-578,
	-212,
	206,
	465,
	457,
	217,
	-112,
	-358,
	-407,
	-256,
	1,
	226,
	309,
	222,
	24,
	-176,
	-279,
	-240,
	-93,
	81,
	191,
	187,
	80,
	-67,
	-177,
	-199,
	-128,
	-8,
	97,
	134,
	90,
	-6,
	-103,
	-151,
	-130,
	-55,
	30,
	82,
	75,
	16,
	-63,
	-120,
	-125,
	-74,
	13,
	100,
	155,
	162,
	221
};

#endif /* INC_FIR_TAPS_H_ */
