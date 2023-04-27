/*
 * lwb_adc.h
 *
 *  Created on: 15 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_ADC_H_
#define LWB_ADC_H_

#include "../libwindbird.h"

#ifdef LWB_PLATFORM_EFM32G
	#include "../platforms/efm32g/lwb_efm32g_adc.h"
#else
	#error Not implemented for this platform
#endif

#endif /* LWB_ADC_H_ */
