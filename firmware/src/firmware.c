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
#include <td_scheduler.h>
#include <td_watchdog.h>

#include "wb_debug.h"
#include "wb_led.h"
#include "wb_button.h"
#include "wb_gps.h"
#include "wb_i2c.h"
#include "wb_compass.h"
#include "wb_compass_calibration.h"
#include "wb_accelero.h"
#include "wb_sigfox.h"
#include "wb_propeller.h"
#include "wb_reports.h"
#include "wb_monitoring.h"
#include "wb_power.h"
#include "wb_runmode.h"
#include "wb_version.h"

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

static void StartupLed() {
	WB_LED_Fade(WB_LED_FADE_IN, 1500); // user feedback
	WB_LED_Set();
	TD_RTC_Delay(TMS(2000));
}

static void Shutdown(bool earlyShutdown) {

	// shutdown is engaged
	WB_DEBUG("turn off\n");

	if (!earlyShutdown) {
	  WB_REPORTS_Stop();
	  WB_GPS_PowerOff();
	}

	uint32_t vbatNoLed = WB_POWER_GetBatteryMillivolts();
	uint32_t vcap = WB_POWER_GetCapacitorMillivolts();

	int i;
	for (i=0; i<5; i++) {
		WB_LED_Clear();
		TD_RTC_Delay(TMS(200));
		WB_LED_Set();
		TD_RTC_Delay(TMS(200));
	}

	uint32_t vbatLed = WB_POWER_GetBatteryMillivolts();
	WB_LED_Fade(WB_LED_FADE_OUT, 1500);

	if (!earlyShutdown) {
		WB_SIGFOX_ShutdownMessage(vbatNoLed, vbatLed, vcap);
	}

	WB_POWER_DisableVAUX();

	EMU_EnterEM3(true);
	// ACTUAL "SHUTDOWN" is HERE
	// the system will wake next time we press the button

	StartupLed();

	NVIC_SystemReset(); // so we start fresh
}

static void Calibration() {
	WB_LED_Set();

	WB_COMPASS_CALIBRATION_Begin();

	float sample[3];
	bool sampling = true;
	int timeout = 3000; // 5 minutes, with 100ms delay per loop
	while (sampling) {
		if (WB_BUTTON_Loop() == WB_BUTTON_PRESSED_POWER_SWITCH || timeout == 0) {
			// abort calibration
			WB_LED_Clear();
			return;
		}
		WB_COMPASS_GetRaw(&sample[0], &sample[1], &sample[2]);
		TD_WATCHDOG_Feed();
		switch (WB_COMPASS_CALIBRATION_AddSample(sample)) {
			case WB_COMPASS_CALIBRATION_AQUISITION_COMPLETE:
				sampling = false;
				break;
			case WB_COMPASS_CALIBRATION_SAMPLE_OK:
				WB_LED_Clear();
				break;
			case WB_COMPASS_CALIBRATION_SAMPLE_DISCARD:
				// no nothing
				break;
		}

		TD_RTC_Delay(TMS(100));
		WB_LED_Set();
		//TD_RTC_Delay(TMS(50));
		timeout--;
	};

	WB_COMPASS_CALIBRATION_End();

	WB_LED_Clear();
}

static void SwitchMode() {
	WB_LED_Set();
	TD_RTC_Delay(TMS(100));
	WB_LED_Clear();
	TD_RTC_Delay(TMS(2000));
	int mode = WB_RUNMODE_Get();
	mode = (mode + 1) % WB_RUNMODE_MAX_SWITCHABLE;
	int nBlinks = 0;
	switch(mode) {
		case MODE_SIGFOX_5M:
			nBlinks = 5;
			break;
		case MODE_SIGFOX_10M:
			nBlinks = 10;
			break;
		default:
			WB_DEBUG("ERROR: unknown runmode %d\n", mode);
			return;
	}
	WB_RUNMODE_Set(mode);
	int i;
	for (i=0; i<nBlinks; i++) {
		WB_LED_Set();
		TD_RTC_Delay(TMS(250));
		WB_LED_Clear();
		TD_RTC_Delay(TMS(250));
	}
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
		case WB_BUTTON_PRESSED_SETTINGS:
			WB_DEBUG("button settings 1\n");
			WB_GPS_PowerOff();
			WB_REPORTS_Stop();
			int i;
			for (i=0; i<10; i++) {
				WB_LED_Set();
				TD_RTC_Delay(TMS(100));
				WB_LED_Clear();
				TD_RTC_Delay(TMS(100));
			}
			/*
			TD_WATCHDOG_Feed();
			if (WB_BUTTON_Loop() == WB_BUTTON_PRESSED_SETTINGS) {
				// did we continue pressing for a second period?
				WB_DEBUG("button settings 2\n");
				WB_DEBUG("mode change\n");
				SwitchMode();
			} else {
				// single period
				WB_DEBUG("calibration\n");
				Calibration();
			} */
			WB_DEBUG("calibration\n");
			Calibration();
			TD_RTC_Delay(TMS(2000));

			StartupLed();

			NVIC_SystemReset();
			break;
	}
}

void TD_USER_Setup(void) {

	WB_MONITORING_Init(); // start watchdog

	// init serial port for GPS and DEBUG.
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);

	tfp_printf("*** HELLO ***\n");
	tfp_printf("Device ID  : %x\n", TD_SIGFOX_GetId());
	tfp_printf("Firmware version  : %d\n", (uint32_t)WB_FIRMWARE_VERSION);
	tfp_printf("Compilation  : %s %s\n", __DATE__, __TIME__);

	WB_POWER_Init();
	WB_LED_Init();
	WB_BUTTON_Init();
	WB_RUNMODE_Init();

	uint32_t vcapEarlyBoot = WB_POWER_GetCapacitorMillivolts();

	while (true) {
		uint32_t vcap = WB_POWER_GetCapacitorMillivolts();
		uint32_t vbat = WB_POWER_GetBatteryMillivolts();
		WB_DEBUG("vcap: %d mV \t %d mV\n", vcap, vbat);
		if (vcap < 2900) {
			WB_LED_Set();
			TD_RTC_Delay(TMS(100));
			WB_LED_Clear();
			TD_RTC_Delay(TMS(500));
		} else {
			break;
		}
		if (WB_BUTTON_Loop() == WB_BUTTON_PRESSED_POWER_SWITCH) {
			Shutdown(true);
		}
		TD_WATCHDOG_Feed();
		TD_SCHEDULER_Process();
	}

	WB_REPORTS_Init();
	WB_GPS_Init();
	WB_I2C_Init();
	WB_COMPASS_Init();
	WB_ACCELERO_Init();
	WB_SIGFOX_Init();
	WB_PROPELLER_Init();
	tfp_printf("*** INIT OK ***\n");


	uint32_t vbatNoLed = WB_POWER_GetBatteryMillivolts();

	WB_LED_Set();
	TD_RTC_Delay(TMS(3000)); // wait for windspeed acquisition
	float windSpeed = WB_PROPELLER_GetSpeed();
	float windHeading = WB_COMPASS_GetHeading();

	uint32_t vbatLed = WB_POWER_GetBatteryMillivolts();
	uint32_t vcapNow = WB_POWER_GetCapacitorMillivolts();

	WB_SIGFOX_StartupMessage(windSpeed, windHeading, vbatNoLed, vbatLed, vcapEarlyBoot, vcapNow);

	WB_LED_Clear();

	//WB_GPS_PowerOn(30);
	WB_GPS_PowerOn(180);
	while (!WB_GPS_Locate()) {
		//WB_LED_Clear();
		ButtonLoop();
		TD_RTC_Sleep();
		//WB_LED_Set();
	}
	//WB_LED_Clear();
	WB_GPS_PowerOff();
	WB_REPORTS_Start();
}

void TD_USER_Loop(void) {
	WB_DEBUG("*** loop ***\tvcap: %d\tvbat: %d\n",
			WB_POWER_GetCapacitorMillivolts(),
			WB_POWER_GetBatteryMillivolts());
	ButtonLoop();
}

