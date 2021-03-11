/**************************************************************************
 * @file pp_propeller.c
 * @brief Propeller Sensor API for PIOUPIOU's firmware
 * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2015 Bac Plus Zéro S.A.S.
 * (C) Copyright 2016 Altostratus SA
 * (C) Copyright 2021 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of PIOUPIOU WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/pioupiou-archive/pioupiou-v1-firmware/blob/master/README.md
 *
 ******************************************************************************/
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_pcnt.h>
#include <em_rtc.h>
#include "pp_debug.h"
#include "pp_propeller.h"

#define PROPELLER_PORT	gpioPortC
#define PROPELLER_BIT	0

#define PROPELLER_CALIBRATION 0.696

static uint32_t lastRTC;
static volatile uint8_t overflowCount;

void PCNT0_IRQHandler(void) {
	overflowCount++;
	//PP_DEBUG("PCNT overflow\n");
	PCNT_IntClear(PCNT0, PCNT_IF_OF);
}

void PP_PROPELLER_Reset() {
	PCNT_CounterSet(PCNT0, 0);
	lastRTC = RTC_CounterGet();
	overflowCount = 0;
}

void PP_PROPELLER_Init() {

	CMU_ClockEnable(cmuClock_PCNT0, true);

	GPIO_PinModeSet(PROPELLER_PORT, PROPELLER_BIT, gpioModeInput, 1);
	PCNT0->ROUTE = (PCNT0->ROUTE & ~_PCNT_ROUTE_LOCATION_MASK) | PCNT_ROUTE_LOCATION_LOC2;

	PCNT_Init_TypeDef pcntInit = {
			.mode		= pcntModeOvsSingle,
			.counter	= 0,
			.top		= 0xFF,
			.negEdge	= true,
			.countDown	= false,
			.filter		= true
	};

	PCNT_Init(PCNT0, &pcntInit);

	PCNT_IntEnable(PCNT0, PCNT_IF_OF);
	NVIC_EnableIRQ(PCNT0_IRQn);

	PP_PROPELLER_Reset();

}



float PP_PROPELLER_GetSpeed() {

	uint32_t now;
	uint32_t dt;
	uint16_t count;
	float freq;

	now = RTC_CounterGet();
	count = PCNT_CounterGet(PCNT0) + 0xFF * overflowCount;

	if (now < lastRTC) {
		// rtc overflow
		dt = 512*32768 - lastRTC + now;
	} else {
		dt = now - lastRTC;
	}

	if (dt < 32768) {
		freq = -1;
	} else {
		freq = count / (dt / 32768.) * PROPELLER_CALIBRATION;
	}

	PP_PROPELLER_Reset();

	return freq;

}
