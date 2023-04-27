#include "wb_config.h"
#include "wb_config_module.h"

#include "libwindbird/core/lwb_scheduler.h"
#include "libwindbird/core/lwb_utils.h"
#include "libwindbird/core/lwb_serial.h"
#include "libwindbird/core/lwb_settings.h"
#include "libwindbird/core/lwb_console.h"

#include "libwindbird/drivers/lwb_led.h"
#include "libwindbird/drivers/lwb_pulsecounter.h"

#include "libwindbird/metrics/lwb_wind.h"

#include <td_rtc.h>
#include <td_sigfox.h>

#include <td_flash.h>

uint8_t message[12];

void SamplingService(uint32_t millis) {
	/*
	 * /!\ this code runs inside an IRQ.
	 * This is necessary to keep measuring even
	 * during SIGFOX message transmission.
	 */
	uint16_t wind = 0xFFFF;
	uint16_t windExtreme = 0xFFFF;


	LWB_PULSECOUNTER_Sample();

	windExtreme = LWB_PULSECOUNTER_GetLastSamples(3 * SAMPLES_PER_SECOND);
	LWB_WIND_PushExtreme(windExtreme);

	switch (millis % 1000) {
		case 0:
			LWB_LED_Set();
			wind = LWB_PULSECOUNTER_GetLastSamples(1 * SAMPLES_PER_SECOND);
			LWB_WIND_PushSample(wind);
			//WB_COMPASS_Trigger();
			break;
		case 250:
			LWB_LED_Clear();
			//WB_COMPASS_Read();
			break;
		case 500:
			// nop
			LWB_LED_Set();
			break;
		case 750:
			LWB_LED_Clear();
			//WB_PROPELLER_Sample();
			break;
	}
	LWB_SERIAL_Debugln("millis: %d	wind: %d	windExtreme: %d", millis, wind, windExtreme);
}

void SamplingStart(uint32_t millis) {
	LWB_PULSECOUNTER_Reset();
	LWB_WIND_Reset();
}

void TD_USER_Setup(void) {
	LWB_SERIAL_Init(9600);



	LWB_CONSOLE_BootMenu();

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
	LWB_SCHEDULER_Init(&SamplingService, &SamplingStart);
	LWB_SCHEDULER_Start();*/
}

void TD_USER_Loop(void) {
	//LWB_SERIAL_Debugln("loop %d %d %d",  TD_RTC_Now(), LWB_SCHEDULER_Millis(), LWB_UTILS_StackUsage());
	//if (LWB_SCHEDULER_Millis() == 10000) TD_SIGFOX_SendV1(MODE_FRAME, 0, message, 12, 2, 0, 0);
}
