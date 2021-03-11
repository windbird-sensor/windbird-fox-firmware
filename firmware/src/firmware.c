/**************************************************************************
 * @file firmware.c
 * @brief Main program functions for PIOUPIOU's firmware
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
 
#include <efm32.h>
#include <em_gpio.h>
#include <em_emu.h>
#include <em_cmu.h>
#include <td_core.h>
#include <td_uart.h>
#include <td_sigfox.h>
#include <td_utils.h>

#include "pp_debug.h"
#include "pp_led.h"
#include "pp_button.h"
#include "pp_gps.h"
#include "pp_i2c.h"
#include "pp_pressure.h"
#include "pp_compass.h"
#include "pp_sigfox.h"
#include "pp_propeller.h"
#include "pp_reports.h"
#include "pp_monitoring.h"


#define MODULE_REVISION REVISION_TD1208

#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0

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
	PP_DEBUG("turn off\n");

	PP_REPORTS_Stop();
	PP_GPS_PowerOff();

	int i;
	for (i=0; i<5; i++) {
		PP_LED_Clear();
		TD_RTC_Delay(TMS(200));
		PP_LED_Set();
		TD_RTC_Delay(TMS(200));
	}
	PP_LED_Clear();

	PP_SIGFOX_ShutdownMessage();

	EMU_EnterEM3(true);
	// ACTUAL "SHUTDOWN" is HERE
	// the system will wake next time we press the button

	PP_LED_Set(); // user feedback
	TD_RTC_Delay(TMS(2000));

	NVIC_SystemReset(); // so we start fresh
}

static void Calibration() {
	PP_LED_StartBlink(0, TMS(100));

	PP_GPS_PowerOff();
	PP_REPORTS_Stop();

	TD_RTC_Delay(TMS(5000));

	PP_LED_StopBlink();
	PP_LED_Set();

	TD_RTC_Delay(TMS(1000));
	PP_COMPASS_Calibrate();
	PP_LED_Clear();

	PP_COMPASS_SaveCalibration();

	//PP_COMPASS_TestCalibration();

	PP_REPORTS_Start();
}

static void ButtonLoop() {
	switch (PP_BUTTON_Loop()) {
		case PP_BUTTON_NO_ACTION:
			//PP_DEBUG("button no action\n");
			break;
		case PP_BUTTON_PRESSED_POWER_SWITCH:
			PP_DEBUG("button power switch\n");
			Shutdown();
			break;
		case PP_BUTTON_PRESSED_CALIBRATION:
			PP_DEBUG("button calibration\n");
			Calibration();
			break;
	}
}

void TD_USER_Setup(void) {

	// init serial port for GPS and DEBUG.
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);

	PP_DEBUG("*** HELLO ***\n");
	PP_DEBUG("Device ID : %x\n", TD_SIGFOX_GetId());

	PP_MONITORING_Init();
	PP_REPORTS_Init();
	PP_LED_Init();
	PP_BUTTON_Init();
	PP_GPS_Init();
	PP_I2C_Init();
	PP_PRESSURE_Shutdown();
	PP_COMPASS_Init();
	PP_SIGFOX_Init();
	PP_PROPELLER_Init();

	//PP_COMPASS_TestCalibration();

	PP_LED_Set();
	TD_RTC_Delay(TMS(3000)); // wait for windspeed acquisition
	float windSpeed = PP_PROPELLER_GetSpeed();
	float windHeading = PP_COMPASS_GetHeading();
	PP_SIGFOX_StartupMessage(windSpeed, windHeading);

	PP_GPS_PowerOn(300);
	while (!PP_GPS_Locate()) {
		PP_LED_Clear();
		ButtonLoop();
		TD_RTC_Sleep();
		PP_LED_Set();
	}
	PP_LED_Clear();
	PP_GPS_PowerOff();

	PP_REPORTS_Start();

}

void TD_USER_Loop(void) {
	PP_DEBUG("*** loop ***\t%d\n", TD_STACK_Usage());
	ButtonLoop();
}
