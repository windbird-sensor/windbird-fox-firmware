/**************************************************************************
 * @file WB_propeller.c
 * @brief Propeller Sensor API for WINDBIRD's firmware
 * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2015 Bac Plus Zéro S.A.S.
 * (C) Copyright 2016 Altostratus SA
 * (C) Copyright 2021 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of WINDBIRD WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/windbird-sensor/windbird-firmware/blob/main/README.md
 *
 ******************************************************************************/

#include "../libwindbird.h"
#include "../../wb_config.h"

#include "../core/lwb_serial.h"

#include "lwb_pulsecounter.h"

#ifdef LWB_PLATFORM_EFM32G
	#include <em_cmu.h>
	#include <em_gpio.h>
	#include <em_pcnt.h>
#else
	#error "Not implemented for this platform"
#endif


#define NSAMPLES (SAMPLES_PER_SECOND*3)

static volatile uint8_t nPulsesRing[NSAMPLES] = {0};
static volatile int cursor = 0;


/*
 We expect no overflow on a 8 bit counter, since 256/0.25 -> 1kHz -> 700km/h
 */

void LWB_PULSECOUNTER_Init() {
	#ifdef LWB_PLATFORM_EFM32G
		CMU_ClockEnable(cmuClock_PCNT0, true);

		GPIO_PinModeSet(PULSECOUNTER_PORT, PULSECOUNTER_BIT, gpioModeInput, 1);
		PCNT0->ROUTE = (PCNT0->ROUTE & ~_PCNT_ROUTE_LOCATION_MASK) | PULSECOUNTER_LOCATION;

		PCNT_Init_TypeDef pcntInit = {
				.mode		= pcntModeOvsSingle,
				.counter	= 0,
				.top		= 0xFF,
				.negEdge	= true,
				.countDown	= false,
				.filter		= true
		};

		PCNT_Init(PCNT0, &pcntInit);
	#endif
}

void LWB_PULSECOUNTER_Reset() {
	int i;
	for (i=0; i<NSAMPLES; i++) nPulsesRing[i] = 0;
	#ifdef LWB_PLATFORM_EFM32G
		PCNT_CounterSet(PCNT0, 0);
	#endif
}

uint8_t LWB_PULSECOUNTER_SampleRaw() {
	#ifdef LWB_PLATFORM_EFM32G
		uint8_t nPulses = PCNT_CounterGet(PCNT0);
		LWB_SERIAL_Debugln("pulse: %d", nPulses);
		PCNT_CounterSet(PCNT0, 0);
	#endif

	return nPulses;
}


void LWB_PULSECOUNTER_Sample() {
	nPulsesRing[cursor++] = LWB_PULSECOUNTER_SampleRaw();
	cursor %= NSAMPLES;
}

uint16_t LWB_PULSECOUNTER_GetLastSamples(int count) {
	uint16_t nPulses = 0;
	int c = cursor;
	int i;
	for (i=0; i<count; i++) {
		c = (c - 1 + NSAMPLES) % NSAMPLES;
		nPulses += nPulsesRing[c];
	}
	return nPulses;
}
