/**************************************************************************
 * @file production-test.c
 * @brief Test firmware for Pioupiou V1
 * @author Nicolas BALDECK
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
 
#include <efm32.h>
#include <td_rtc.h>
#include <td_measure.h>
#include <td_sigfox.h>
#include <td_uart.h>

#include "wb_debug.h"
#include "wb_led.h"
#include "wb_button.h"
#include "wb_i2c.h"
#include "wb_pressure.h"
#include "wb_compass.h"
#include "wb_gps.h"
#include "wb_propeller.h"

#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

void TD_USER_Setup(void) {

	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);


	WB_LED_StartBlink(0, TMS(100));


	WB_BUTTON_Init();
	WB_LED_Init();
	WB_I2C_Init();
	WB_PRESSURE_Init();
	WB_COMPASS_Init();
	WB_PROPELLER_Init();
	WB_GPS_Init();

	WB_DEBUG("\n\n==== PRODUCTION TEST ====\n");

	WB_DEBUG("Press the button...\n");
	while (WB_BUTTON_Loop() != WB_BUTTON_PRESSED_POWER_SWITCH) {}

	WB_DEBUG("Device ID:\t%x\n", TD_SIGFOX_GetId());

	WB_DEBUG("Propeller:\n");
	WB_PROPELLER_Reset();
	int i;
	for (i=0; i<5; i++) {
		TD_RTC_Delay(TMS(1000));
		float windSpeed = WB_PROPELLER_GetSpeed();
		WB_DEBUG("\t%d\n", (int)(10*windSpeed));
	}

	int voltage = TD_MEASURE_VoltageExtended();
	WB_DEBUG("Voltage:\t%d\n", voltage);
	if (voltage > 3350) {
		WB_DEBUG("VOLTAGE IS HIGH\n");
	} else if (voltage < 3250) {
		WB_DEBUG("VOLTAGE IS LOW\n");
	}

	if(WB_COMPASS_Test()) {
		int16_t x, y, z;
		WB_COMPASS_GetRaw(&x, &y, &z);
		WB_DEBUG("Compass:\tok (%d\t%d\t%d)\n", x, y, z);
	} else {
		WB_DEBUG("COMPASS:\tFAILED\n");
	}

	float pressure;
	if(WB_PRESSURE_Test(&pressure)) {
		WB_DEBUG("Pressure:\tok (%d)\n", (int)pressure);
	} else {
		WB_DEBUG("Pressure:\tFAILED\n");
	}

	WB_DEBUG("Gps:\n");
	WB_GPS_PowerOn(3);
	while (!WB_GPS_Locate()) {
		TD_RTC_Sleep();
	}
	WB_GPS_PowerOff();

	WB_LED_StopBlink();
	WB_DEBUG("==== TEST IS FINISHED ====\n\n");
}

void TD_USER_Loop(void) {}
