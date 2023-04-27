/**************************************************************************
 * @file WB_led.c
 * @brief LED API for WINDBIRD's firmware
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

#include "lwb_led.h"

#ifdef LWB_PLATFORM_EFM32G
	#include <em_gpio.h>
#else
	#error "Not implemented for this platform"
#endif

void LWB_LED_Init() {
	#ifdef LWB_PLATFORM_EFM32G
		GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPullDrive, 0);
	#endif
}

void LWB_LED_Clear() {
	#ifdef LWB_PLATFORM_EFM32G
		GPIO_PinOutClear(LED_PORT, LED_BIT);
	#endif
}

void LWB_LED_Set() {
	#ifdef LWB_PLATFORM_EFM32G
		GPIO_PinOutSet(LED_PORT, LED_BIT);
	#endif
}

void LWB_LED_Toggle() {
	#ifdef LWB_PLATFORM_EFM32G
		GPIO_PinOutToggle(LED_PORT, LED_BIT);
	#endif
}

float cubicEasing (float step, float start, float stop, float duration) {
	step /= duration;
	return stop * step * step * step + start;
}

void __attribute__((optimize("O0"))) LWB_LED_Fade(LWB_LED_FadeDirection_t direction, uint32_t duration) {
	// we use __attribute__((optimize("O0"))) to keep the right timing

	#ifndef LWB_PLATFORM_TD120X
		#error "pwmFreq and duration must be adjusted for this platform"
	#endif

	uint32_t pwmFreq = 1000;

	int i;
	int j;
	int on;
	int off;
	int val;
	for (i=0; i<duration; i++) {
		val = cubicEasing(i, 0, pwmFreq, duration);
		if (direction == LWB_LED_FADE_IN) {
			on = val;
			off = pwmFreq - val;
		} else {
			on = pwmFreq - val;
			off = val;
		}
		LWB_LED_Set();
		for (j=0; j<on; j++) {};
		LWB_LED_Clear();
		for (j=0; j<off; j++) {};
	}
}
