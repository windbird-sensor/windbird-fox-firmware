/**************************************************************************
 * @file firmware.c
 * @brief Main program functions for WINDBIRD's firmware
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
 
#include <efm32.h>
#include <em_gpio.h>
#include <em_emu.h>
#include <em_cmu.h>
#include <td_core.h>
#include <td_uart.h>
#include <td_sigfox.h>
#include <td_utils.h>

#include "wb_debug.h"
#include "wb_led.h"
#include "wb_button.h"
#include "wb_gps.h"
#include "wb_i2c.h"
#include "wb_compass.h"
#include "wb_sigfox.h"
#include "wb_propeller.h"
#include "wb_reports.h"
#include "wb_monitoring.h"
#include "wb_power.h"



#define MODULE_REVISION REVISION_TD1208
#define PRODUCT_LED_PORT LED_PORT
#define PRODUCT_LED_BIT LED_BIT
#define PRODUCT_LED_BLINK 1
#define PRODUCT_LED_DRIVE gpioDriveModeLowest

// Disabling the bootloader because I believe there is
// a security issue in the current over-the-air
// firware upgrading function.
// See https://github.com/Telecom-Design/TD_RF_Module_SDK/issues/40
#define PRODUCT_BOOTLOADER_SKIP 1
#define PRODUCT_UART_BOOTLOADER_SKIP 1

// Needed to prevent a bug related to PRODUCT_BOOTLOADER_SKIP
// See https://github.com/Telecom-Design/TD_RF_Module_SDK/issues/41
#define TD_SIGFOX_TRANSMIT_LOCAL_ONLY 1

#include <td_config.h>

static void Shutdown() {

	// shutdown is engaged
	WB_DEBUG("turn off\n");

	WB_REPORTS_Stop();
	WB_GPS_PowerOff();

	int i;
	for (i=0; i<5; i++) {
		WB_LED_Clear();
		TD_RTC_Delay(TMS(200));
		WB_LED_Set();
		TD_RTC_Delay(TMS(200));
	}
	WB_LED_Clear();

	WB_SIGFOX_ShutdownMessage();

	EMU_EnterEM3(true);
	// ACTUAL "SHUTDOWN" is HERE
	// the system will wake next time we press the button

	WB_LED_Set(); // user feedback
	TD_RTC_Delay(TMS(2000));

	NVIC_SystemReset(); // so we start fresh
}

static void Calibration() {
	WB_LED_StartBlink(0, TMS(100));

	WB_GPS_PowerOff();
	WB_REPORTS_Stop();

	TD_RTC_Delay(TMS(5000));

	WB_LED_StopBlink();
	WB_LED_Set();

	TD_RTC_Delay(TMS(1000));
	WB_COMPASS_Calibrate();
	WB_LED_Clear();

	WB_COMPASS_SaveCalibration();

	//WB_COMPASS_TestCalibration();

	WB_REPORTS_Start();
}

static void ButtonLoop() {
	switch (WB_BUTTON_Loop()) {
		case WB_BUTTON_NO_ACTION:
			//WB_DEBUG("button no action\n");
			break;
		case WB_BUTTON_PRESSED_POWER_SWITCH:
			WB_DEBUG("button power switch\n");
			Shutdown();
			break;
		case WB_BUTTON_PRESSED_CALIBRATION:
			WB_DEBUG("button calibration\n");
			Calibration();
			break;
	}
}

void TD_USER_Setup(void) {

	// init serial port for GPS and DEBUG.
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);

	WB_DEBUG("*** HELLO ***\n");
	WB_DEBUG("Device ID : %x\n", TD_SIGFOX_GetId());

	WB_MONITORING_Init();
	WB_POWER_Init();
	WB_LED_Init();

	while (true) {
		uint32_t voltage = WB_POWER_GetCapacitorMillivolts();
		WB_DEBUG("vcap: %d mV\n", voltage);
		if (voltage > 2500) break;
		WB_LED_Set();
		TD_RTC_Delay(TMS(2));
		WB_LED_Clear();
		TD_WATCHDOG_Feed();
		TD_RTC_Delay(TMS(2000));
	}

	WB_REPORTS_Init();
	WB_BUTTON_Init();
	WB_GPS_Init();
	WB_I2C_Init();
	WB_COMPASS_Init();
	WB_SIGFOX_Init();
	WB_PROPELLER_Init();

	while (true) {
		uint32_t voltage = WB_POWER_GetCapacitorMillivolts();
		WB_DEBUG("vcap: %d mV\n", voltage);
		if (voltage > 3300) break;
		WB_LED_Set();
		TD_RTC_Delay(TMS(2));
		WB_LED_Clear();
		TD_WATCHDOG_Feed();
		TD_RTC_Delay(TMS(2000));
	}

	//WB_COMPASS_TestCalibration();

	WB_LED_Set();
	TD_RTC_Delay(TMS(3000)); // wait for windspeed acquisition
	float windSpeed = WB_PROPELLER_GetSpeed();
	float windHeading = WB_COMPASS_GetHeading();
	WB_SIGFOX_StartupMessage(windSpeed, windHeading);

	// WB_GPS_PowerOn(30);
	WB_GPS_PowerOn(300);
	while (!WB_GPS_Locate()) {
		WB_LED_Clear();
		WB_DEBUG("vcap: %d mV\n", WB_POWER_GetCapacitorMillivolts());
		WB_DEBUG("vbat: %d mV\n", WB_POWER_GetBatteryMillivolts());
		ButtonLoop();
		TD_RTC_Sleep();
		WB_LED_Set();
	}
	WB_LED_Clear();
	WB_GPS_PowerOff();
	WB_REPORTS_Start();
}

void TD_USER_Loop(void) {
	WB_DEBUG("*** loop ***\t%d\n", TD_STACK_Usage());
	ButtonLoop();
	WB_DEBUG("vcap: %d mV\n", WB_POWER_GetCapacitorMillivolts());
	WB_DEBUG("vbat: %d mV\n", WB_POWER_GetBatteryMillivolts());
}

