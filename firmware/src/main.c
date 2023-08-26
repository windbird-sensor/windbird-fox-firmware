#include "wb_config.h"
#include "wb_config_module.h"

#include "libwindbird/core/lwb_scheduler.h"
#include "libwindbird/core/lwb_utils.h"
#include "libwindbird/core/lwb_serial.h"
#include "libwindbird/core/lwb_settings.h"
#include "libwindbird/core/lwb_console.h"

#include "libwindbird/drivers/lwb_button.h"
#include "libwindbird/drivers/lwb_led.h"

#include "wb_sampling.h"

#include <td_rtc.h>

#include <td_flash.h>

uint8_t message[12];

void TD_USER_Setup(void) {
	LWB_SERIAL_Init(9600);

	// wait for capacitor

	LWB_CONSOLE_BootMenu();
/*
	int mode = LWB_SETTINGS_Read(SETTING_RUNMODE);
	if (mode == RUNMODE_TRACKING) WB_TRACKING_InfiniteLoop();
	if (mode == RUNMODE_CALIBRATION) WB_CALIBRATION_InfiniteLoop();
*/
	/*LWB_GPS_PowerOn(0);
	int c;
	while (1) {
			c = LWB_SERIAL_GetChar();
			if (c < 0) continue;
			LWB_SERIAL_PutChar(c);
	}*/
/*
	LWB_LED_Init();
	LWB_PULSECOUNTER_Init();
	LWB_SCHEDULER_Init(&WB_SAMPLING_Service, &WB_SAMPLING_Reset);
	LWB_SCHEDULER_Start();

	// acknowledge startup

	WB_UI_AcknowledgeStartup();

	// wait for capacitor to be charged

	WB_POWER_WaitForCapacitor();

	// send startup message



	// start gps



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

	// wait for capacitor to be charged

	// send location message
*/
}


void TD_USER_Loop(void) {
	LWB_SERIAL_Debugln("loop %d %d %d",  TD_RTC_Now(), LWB_SCHEDULER_Millis(), LWB_UTILS_StackUsage());
	/*
	if (WB_SAMPLING_CompletionAlarm()) {
		LWB_SERIAL_Debugln("sampling complete!");
		if (LWB_ACCELERO_IsVertical()) {
			message = WB_MSG_PrepareReport(WB_SAMPLING_GetData());
		} else {
			LWB_SERIAL_Debugln("NOT verticall");
			LWB_SIGFOX_Send(WB_MSG_NOT_VERTICAL, 12);
		}
		WB_MSG_AddToQueue(message, 60000, false);
		WB_SAMPLING_Reset();
	}

	if (LWB_SCHEDULER_Millis() % 24 * 3600 == 0) {
		WB_GPS_Locate();
	}

	if (LWB_ACCELERO_MovementAlarm()) {
		WB_MSG_AddToQueue(WB_MSG_MOVEMENT_ALERT, 60000, WB_MSG_REQUEST_CALLBACK);
	}

	if (WB_MSG_DownlinkAlarm()) {
		LWB_SETTINGS_Set(SETTING_RUNMODE, RUNMODE_TRACKING);
	}

	WB_GPS_Process();
	WB_MSG_ProcessQueue();
	WB_MSG_ProcessQueue();
	*/
}
