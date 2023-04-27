/**************************************************************************
 * @file wb_power.c
 * @brief POWER API for WINDBIRD's firmware
 * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2022 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of WINDBIRD WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/windbird-sensor/windbird-firmware/blob/main/README.md
 *
 ******************************************************************************/

#include "lwb_efm32g_adc.h"

#ifdef LWB_PLATFORM_EFM32G

#include "../../../wb_config.h"

#include <em_adc.h>
#include <em_cmu.h>

#define ADC_REFERENCE adcRef1V25
#define ADC_REFERENCE_MV 1250
#define ADC_RESOLUTION 4096

 // datasheet page 82


static volatile bool AdcLock = false;

uint32_t ToMillivolts(uint32_t value, uint32_t rTop, uint32_t rBottom) {
	uint32_t adcVoltage = value * ADC_REFERENCE_MV / ADC_RESOLUTION;
	return adcVoltage * (rTop + rBottom) / rBottom;
}

/*
 * SampleAdc is a copy of TD_MEASURE_SingleVoltage
 * with a higher acquisition time (adcAcqTime16) to account for
 * high impedance of the voltage dividers on Winbird board.
 */
uint32_t LWB_ADC_SampleRaw(ADC_SingleInput_TypeDef input /*, ADC_Ref_TypeDef ref*/)
{
	uint32_t setpoint;
	bool AlreadyLocked;
	volatile int timeout;

	// Base the ADC configuration on the default setup
	ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	// Timeout of 50 ms
	timeout = 100000;

	// Enter Critical Section
	__set_PRIMASK(1);
	if (AdcLock) {
		AlreadyLocked = true;
	} else {
		AlreadyLocked = false;
		AdcLock = true;
	}
	__set_PRIMASK(0);

	// Exit Critical Section
	if (!AlreadyLocked) {

		// Initialize timebase
		init.timebase = ADC_TimebaseCalc(0);
		init.prescale = ADC_PrescaleCalc(40000, 0);
		CMU_ClockEnable(cmuClock_ADC0, true);
		ADC_Init(ADC0, &init);

		// Set reference and input
		single_init.reference = ADC_REFERENCE;
		single_init.input = input;
		single_init.acqTime = adcAcqTime16; // added for Windbird
		ADC_InitSingle(ADC0, &single_init);

		// Start one ADC sample
		ADC_Start(ADC0, adcStartSingle);

		// Active wait for ADC to complete
		while ((timeout--) && ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0)) {
			;
		}
		setpoint = ADC_DataSingleGet(ADC0);
		CMU_ClockEnable(cmuClock_ADC0, false);

		/* Enter Critical Section */
		__set_PRIMASK(1);
		AdcLock = false;
		__set_PRIMASK(0);
		/* Exit Critical Section */

	} else {

		// ADC no is not free, return 0
		setpoint = 0;
	}
	return(setpoint);
}

uint32_t LWB_ADC_SampleMillivolts(ADC_SingleInput_TypeDef input, uint32_t rTop, uint32_t rBottom) {
	return ToMillivolts(LWB_ADC_SampleRaw(input), rTop, rBottom);
}

#endif /* LWB_PLATFORM_EFM32G */