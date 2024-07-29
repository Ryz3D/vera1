/*
 * DigitalFilter.h
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

 * 0 Hz - 400 Hz
 gain = 1
 desired ripple = 5 dB
 actual ripple = 4.096319691387241 dB

 * 500 Hz - 8000 Hz
 gain = 0
 desired attenuation = -40 dB
 actual attenuation = -40.16108723816581 dB

 */

#define FILTER_TAP_NUM 163

const float fir_taps[FILTER_TAP_NUM] = { -0.003875308023407821, -0.005401051970651611, -0.0013482238825143346, -0.004804524175426426, -0.0035549456554710367, -0.005297283903381135,
		-0.005151111395853579, -0.006176731870133825, -0.006421790457196899, -0.007094695514005489, -0.007409576999371765, -0.007838840579605643, -0.00806598194669142, -0.008266661063141492,
		-0.008313955708014195, -0.008272439260482321, -0.008090016917461617, -0.0077910074558892066, -0.007356282945126238, -0.006800276480638527, -0.006119824922626544, -0.005329364639755203,
		-0.004434340795431579, -0.0034540004421141557, -0.002405044632227061, -0.0013061151601066502, -0.00018224733400742224, 0.000944430666696325, 0.002047544340205705, 0.003098760159264705,
		0.004076343504585989, 0.004949370427243804, 0.005699488983624832, 0.006297750598718603, 0.006731126771740684, 0.006976621939247575, 0.007026265869234929, 0.006870504739699273,
		0.00650284762284671, 0.005931485864153231, 0.005155065794738885, 0.004194235301717733, 0.0030625333411290604, 0.001784847082671486, 0.0003902219988417865, -0.0010905102275515209,
		-0.0026210089732541246, -0.004159075452681213, -0.005667069632020057, -0.007093487260929855, -0.008400433322762524, -0.00953745115765121, -0.01046547270286178, -0.011143204473040094,
		-0.011532834196359063, -0.01160390986305096, -0.011328418578341876, -0.010681658616108182, -0.009653716697569861, -0.008231649709865084, -0.006420562404738674, -0.00423051085106194,
		-0.0016732602963220747, 0.0012207732475666166, 0.004424253696015077, 0.007900433116111195, 0.011595159573906255, 0.015462647532267297, 0.019441634367315483, 0.023473243869538057,
		0.027500956097822982, 0.03145423462247397, 0.03526528818397904, 0.038877778881878114, 0.04222950182118469, 0.045261818849192205, 0.047923155169480834, 0.050164705974999245,
		0.05195109533873379, 0.05325333035540151, 0.05404107877840227, 0.05430296884061681, 0.05404107877840227, 0.05325333035540151, 0.05195109533873379, 0.050164705974999245, 0.047923155169480834,
		0.045261818849192205, 0.04222950182118469, 0.038877778881878114, 0.03526528818397904, 0.03145423462247397, 0.027500956097822982, 0.023473243869538057, 0.019441634367315483,
		0.015462647532267297, 0.011595159573906255, 0.007900433116111195, 0.004424253696015077, 0.0012207732475666166, -0.0016732602963220747, -0.00423051085106194, -0.006420562404738674,
		-0.008231649709865084, -0.009653716697569861, -0.010681658616108182, -0.011328418578341876, -0.01160390986305096, -0.011532834196359063, -0.011143204473040094, -0.01046547270286178,
		-0.00953745115765121, -0.008400433322762524, -0.007093487260929855, -0.005667069632020057, -0.004159075452681213, -0.0026210089732541246, -0.0010905102275515209, 0.0003902219988417865,
		0.001784847082671486, 0.0030625333411290604, 0.004194235301717733, 0.005155065794738885, 0.005931485864153231, 0.00650284762284671, 0.006870504739699273, 0.007026265869234929,
		0.006976621939247575, 0.006731126771740684, 0.006297750598718603, 0.005699488983624832, 0.004949370427243804, 0.004076343504585989, 0.003098760159264705, 0.002047544340205705,
		0.000944430666696325, -0.00018224733400742224, -0.0013061151601066502, -0.002405044632227061, -0.0034540004421141557, -0.004434340795431579, -0.005329364639755203, -0.006119824922626544,
		-0.006800276480638527, -0.007356282945126238, -0.0077910074558892066, -0.008090016917461617, -0.008272439260482321, -0.008313955708014195, -0.008266661063141492, -0.00806598194669142,
		-0.007838840579605643, -0.007409576999371765, -0.007094695514005489, -0.006421790457196899, -0.006176731870133825, -0.005151111395853579, -0.005297283903381135, -0.0035549456554710367,
		-0.004804524175426426, -0.0013482238825143346, -0.005401051970651611, -0.003875308023407821 };

#endif /* INC_FIR_TAPS_H_ */
