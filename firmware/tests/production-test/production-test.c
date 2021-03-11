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
 * This file is a part of PIOUPIOU WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/pioupiou-archive/pioupiou-v1-firmware/blob/master/README.md
 *
 ******************************************************************************/
 
#include <efm32.h>
#include <td_rtc.h>
#include <td_measure.h>
#include <td_sigfox.h>
#include <td_uart.h>

#include "pp_debug.h"
#include "pp_led.h"
#include "pp_button.h"
#include "pp_i2c.h"
#include "pp_pressure.h"
#include "pp_compass.h"
#include "pp_gps.h"
#include "pp_propeller.h"

#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

void TD_USER_Setup(void) {

	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);


	PP_LED_StartBlink(0, TMS(100));


	PP_BUTTON_Init();
	PP_LED_Init();
	PP_I2C_Init();
	PP_PRESSURE_Init();
	PP_COMPASS_Init();
	PP_PROPELLER_Init();
	PP_GPS_Init();

	PP_DEBUG("\n\n==== PRODUCTION TEST ====\n");

	PP_DEBUG("Press the button...\n");
	while (PP_BUTTON_Loop() != PP_BUTTON_PRESSED_POWER_SWITCH) {}

	PP_DEBUG("Device ID:\t%x\n", TD_SIGFOX_GetId());

	PP_DEBUG("Propeller:\n");
	PP_PROPELLER_Reset();
	int i;
	for (i=0; i<5; i++) {
		TD_RTC_Delay(TMS(1000));
		float windSpeed = PP_PROPELLER_GetSpeed();
		PP_DEBUG("\t%d\n", (int)(10*windSpeed));
	}

	int voltage = TD_MEASURE_VoltageExtended();
	PP_DEBUG("Voltage:\t%d\n", voltage);
	if (voltage > 3350) {
		PP_DEBUG("VOLTAGE IS HIGH\n");
	} else if (voltage < 3250) {
		PP_DEBUG("VOLTAGE IS LOW\n");
	}

	if(PP_COMPASS_Test()) {
		int16_t x, y, z;
		PP_COMPASS_GetRaw(&x, &y, &z);
		PP_DEBUG("Compass:\tok (%d\t%d\t%d)\n", x, y, z);
	} else {
		PP_DEBUG("COMPASS:\tFAILED\n");
	}

	float pressure;
	if(PP_PRESSURE_Test(&pressure)) {
		PP_DEBUG("Pressure:\tok (%d)\n", (int)pressure);
	} else {
		PP_DEBUG("Pressure:\tFAILED\n");
	}

	PP_DEBUG("Gps:\n");
	PP_GPS_PowerOn(3);
	while (!PP_GPS_Locate()) {
		TD_RTC_Sleep();
	}
	PP_GPS_PowerOff();

	PP_LED_StopBlink();
	PP_DEBUG("==== TEST IS FINISHED ====\n\n");
}

void TD_USER_Loop(void) {}
