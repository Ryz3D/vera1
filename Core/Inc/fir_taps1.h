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
 http://t-filter.engineerjs.com

 sampling frequency: 16000 Hz

 fixed point precision: 16 bits

 * 0 Hz - 2000 Hz
 gain = 1
 desired ripple = 1 dB
 actual ripple = 0.0981 dB

 * 2500 Hz - 8000 Hz
 gain = 0
 desired attenuation = -80 dB
 actual attenuation = -97.6857 dB

 */

#define FIR_TAPS_LEN 128

const int16_t fir_taps[FIR_TAPS_LEN] = {
	0, -2, -6, -14, -23, -30, -29, -17, 2, 20, 27, 16, -8, -30, -36, -16, 19, 47, 47, 12, -39, -71, -57, 1, 69, 98, 60, -29, -112, -127, -51, 76, 167, 150, 19,
	-147, -231, -157, 46, 246, 296, 134, -154, -373, -350, -61, 323, 531, 377, -90, -579, -725, -347, 384, 994, 987, 193, -1021, -1860, -1501, 418, 3494, 6663, 8668, 8668, 6663, 3494, 418, -1501,
	-1860, -1021, 193, 987, 994, 384, -347, -725, -579, -90, 377, 531, 323, -61, -350, -373, -154, 134, 296, 246, 46, -157, -231, -147, 19, 150, 167, 76, -51, -127, -112, -29, 60, 98, 69, 1, -57,
	-71, -39, 12, 47, 47, 19, -16, -36, -30, -8, 16, 27, 20, 2, -17, -29, -30, -23, -14, -6, -2, 0 };

#endif /* INC_FIR_TAPS_H_ */
