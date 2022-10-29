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

#include <em_gpio.h>
#include <em_cmu.h>
#include <em_letimer.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include "wb_debug.h"
#include "wb_led.h"

// pins are defined in wb_led.h

static uint8_t timer;
static bool isBlinking;

static void BlinkTimer (uint32_t argument, uint8_t repetition) {
	if (isBlinking) {
		GPIO_PinOutToggle(LED_PORT, LED_BIT);
	}
}

void WB_LED_Init() {
	// GPIO_DriveModeSet(LED_PORT, gpioDriveModeHigh); no longer needed with the MOS

	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPullDrive, 0);

	timer = 0xFF;
}

void WB_LED_Clear() {
	GPIO_PinOutClear(LED_PORT, LED_BIT);
}

void WB_LED_Set() {
	GPIO_PinOutSet(LED_PORT, LED_BIT);
}

void WB_LED_Test() {
	int i;
	for (i=0; i<6; i++) {
		GPIO_PinOutToggle(LED_PORT, LED_BIT);
		TD_RTC_Delay(T500MS);
	}
}

void WB_LED_StartBlink(uint8_t seconds, uint16_t ticks) {
	if (timer != 0xFF) {
		TD_SCHEDULER_Remove(timer);
		timer = 0xFF;
	}
	timer = TD_SCHEDULER_AppendIrq(seconds, ticks, 0, TD_SCHEDULER_INFINITE, BlinkTimer, 0);
	if (timer == 0xFF) WB_DEBUG("ERROR initializing led blink timer\n");
	isBlinking = true;
}

void WB_LED_StopBlink() {
	if (timer != 0xFF) {
		TD_SCHEDULER_Remove(timer);
		timer = 0xFF;
	}
	isBlinking = false;
	WB_LED_Clear();
}
