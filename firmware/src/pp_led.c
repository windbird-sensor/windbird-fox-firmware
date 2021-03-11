/**************************************************************************
 * @file pp_led.c
 * @brief LED API for PIOUPIOU's firmware
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

#include <em_gpio.h>
#include <em_cmu.h>
#include <em_letimer.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include "pp_debug.h"
#include "pp_led.h"

#define LED_GREEN_PORT gpioPortD
#define LED_GREEN_BIT 6

#define LED_RED_PORT gpioPortD
#define LED_RED_BIT 7

static uint8_t timer;
static bool isBlinking;

static void BlinkTimer (uint32_t argument, uint8_t repetition) {
	if (isBlinking) {
		GPIO_PinOutToggle(LED_RED_PORT, LED_RED_BIT);
	}
}

void PP_LED_Init() {
	GPIO_DriveModeSet(LED_RED_PORT, gpioDriveModeHigh);

	GPIO_PinModeSet(LED_GREEN_PORT, LED_GREEN_BIT, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_RED_PORT, LED_RED_BIT, gpioModePushPullDrive, 0);

	timer = 0xFF;
}

void PP_LED_Clear() {
	GPIO_PinOutClear(LED_RED_PORT, LED_RED_BIT);
}


void PP_LED_Set() {
	GPIO_PinOutSet(LED_RED_PORT, LED_RED_BIT);
}

void PP_LED_Test() {
	int i;
	for (i=0; i<6; i++) {
		GPIO_PinOutToggle(LED_RED_PORT, LED_RED_BIT);
		TD_RTC_Delay(T500MS);
	}
}

void PP_LED_StartBlink(uint8_t seconds, uint16_t ticks) {
	if (timer != 0xFF) {
		TD_SCHEDULER_Remove(timer);
		timer = 0xFF;
	}
	timer = TD_SCHEDULER_AppendIrq(seconds, ticks, 0, TD_SCHEDULER_INFINITE, BlinkTimer, 0);
	if (timer == 0xFF) PP_DEBUG("ERROR initializing led blink timer\n");
	isBlinking = true;
}

void PP_LED_StopBlink() {
	if (timer != 0xFF) {
		TD_SCHEDULER_Remove(timer);
		timer = 0xFF;
	}
	isBlinking = false;
	PP_LED_Clear();
}
