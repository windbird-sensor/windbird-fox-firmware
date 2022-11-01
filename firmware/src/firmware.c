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
// PRODUCT_LED_DRIVE will be overwritten by VAUX's gpioDriveModeHigh

// enable VAUX in bootloader, to allow external UART communication
#define PRODUCT_INIT_DATA {PIP(VAUX_PORT, VAUX_BIT, PI_OUTPUT, 1),PIS(VAUX_PORT, gpioDriveModeHigh)}

#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0

#include <td_config.h>

static void Shutdown(bool earlyShutdown) {

	// shutdown is engaged
	WB_DEBUG("turn off\n");

	if (!earlyShutdown) {
	  WB_REPORTS_Stop();
	  WB_GPS_PowerOff();
	}

	int i;
	for (i=0; i<5; i++) {
		WB_LED_Clear();
		TD_RTC_Delay(TMS(200));
		WB_LED_Set();
		TD_RTC_Delay(TMS(200));
	}

	uint32_t batteryVoltageWithLedOn = WB_POWER_GetBatteryMillivolts();
	WB_DEBUG("batteryVoltageWithLedOn: %d\n", batteryVoltageWithLedOn);
	WB_LED_Fade(WB_LED_FADE_OUT, 1500);

	if (!earlyShutdown) WB_SIGFOX_ShutdownMessage(batteryVoltageWithLedOn);

	WB_POWER_DisableVAUX();

	EMU_EnterEM3(true);
	// ACTUAL "SHUTDOWN" is HERE
	// the system will wake next time we press the button

	WB_LED_Fade(WB_LED_FADE_IN, 1500); // user feedback
	WB_LED_Set();
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
			Shutdown(false);
			break;
		case WB_BUTTON_PRESSED_CALIBRATION:
			WB_DEBUG("button calibration\n");
			Calibration();
			break;
	}
}

void TD_USER_Setup(void) {

	WB_MONITORING_Init(); // start watchdog

	// init serial port for GPS and DEBUG.
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);

	WB_DEBUG("*** HELLO ***\n");
	WB_DEBUG("Device ID  : %x\n", TD_SIGFOX_GetId());
	WB_DEBUG("Firmware compilation  : %s %s\n", __DATE__, __TIME__);

	WB_POWER_Init();
	WB_LED_Init();
	WB_BUTTON_Init();

	while (true) {
		uint32_t voltage = WB_POWER_GetCapacitorMillivolts();
		WB_DEBUG("vcap: %d mV\n", voltage);
		if (voltage < 2500) {
			WB_LED_Set();
			TD_RTC_Delay(TMS(10));
			WB_LED_Clear();
			TD_RTC_Delay(TMS(1990));
		} else if (voltage < 3300) {
			WB_LED_Set();
			TD_RTC_Delay(TMS(10));
			WB_LED_Clear();
			TD_RTC_Delay(TMS(100));
			WB_LED_Set();
			TD_RTC_Delay(TMS(10));
			WB_LED_Clear();
			TD_RTC_Delay(TMS(1880));
		} else {
			break;
		}
		if (WB_BUTTON_Loop() == WB_BUTTON_PRESSED_POWER_SWITCH) {
			Shutdown(true);
		}
		TD_WATCHDOG_Feed();
	}

	WB_REPORTS_Init();
	WB_GPS_Init();
	WB_I2C_Init();
	WB_COMPASS_Init();
	WB_SIGFOX_Init();
	WB_PROPELLER_Init();

	//WB_COMPASS_TestCalibration();

	WB_LED_Set();
	TD_RTC_Delay(TMS(3000)); // wait for windspeed acquisition
	float windSpeed = WB_PROPELLER_GetSpeed();
	float windHeading = WB_COMPASS_GetHeading();
	uint32_t batteryVoltageWithLedOn = WB_POWER_GetBatteryMillivolts();
	WB_DEBUG("batteryVoltageWithLedOn: %d\n", batteryVoltageWithLedOn);
	WB_SIGFOX_StartupMessage(windSpeed, windHeading, batteryVoltageWithLedOn);

	// WB_GPS_PowerOn(30);
	WB_GPS_PowerOn(300);
	while (!WB_GPS_Locate()) {
		WB_LED_Clear();
		WB_DEBUG("*** gps ***\tvcap: %d\tvbat: %d\n",
				WB_POWER_GetCapacitorMillivolts(),
				WB_POWER_GetBatteryMillivolts());
		ButtonLoop();
		TD_RTC_Sleep();
		WB_LED_Set();
	}
	WB_LED_Clear();
	WB_GPS_PowerOff();
	WB_REPORTS_Start();
}

void TD_USER_Loop(void) {
	WB_DEBUG("*** loop ***\tvcap: %d\tvbat: %d\n",
			WB_POWER_GetCapacitorMillivolts(),
			WB_POWER_GetBatteryMillivolts());
	ButtonLoop();
}

