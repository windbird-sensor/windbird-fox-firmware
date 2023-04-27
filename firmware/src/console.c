#include <stdlib.h>

#include "wb_config.h"

#include "libwindbird/core/lwb_console.h"
#include "libwindbird/core/lwb_serial.h"
#include "libwindbird/core/lwb_settings.h"
#include "libwindbird/core/lwb_i2c.h"

#include "libwindbird/drivers/lwb_led.h"
#include "libwindbird/drivers/lwb_gps.h"
#include "libwindbird/drivers/lwb_pulsecounter.h"
#include "libwindbird/drivers/lwb_button.h"
#include "libwindbird/drivers/lwb_adc.h"
#include "libwindbird/drivers/lwb_compass.h"

#include <td_sigfox.h>
#include <td_rtc.h>
#include <td_measure.h>

#include <math.h>

static int Todo(int argc, const char * const * argv) {
	LWB_SERIAL_Println("todo");
	return 1;
}

static int Adc(int argc, const char * const * argv) {
	LWB_CONSOLE_CHECKARGC(argc, 1);
	int val = 0;
	if (strcmp(argv[1], "vbat") == 0) {
		val = LWB_ADC_SampleMillivolts(VBAT_ADC_CHANNEL, VBAT_TOP_RESISTOR, VBAT_BOTTOM_RESISTOR);
	} else if (strcmp(argv[1], "vbatload") == 0) {
		LWB_LED_Set();
		TD_RTC_Delay(T500MS);
		val = LWB_ADC_SampleMillivolts(VBAT_ADC_CHANNEL, VBAT_TOP_RESISTOR, VBAT_BOTTOM_RESISTOR);
		LWB_LED_Clear();
	} else if (strcmp(argv[1], "vcap") == 0) {
		val = LWB_ADC_SampleMillivolts(VCAP_ADC_CHANNEL, VCAP_TOP_RESISTOR, VCAP_BOTTOM_RESISTOR);
	} else if (strcmp(argv[1], "vcpu") == 0) {
		val = TD_MEASURE_VoltageTemperatureExtended(false);
	} else if (strcmp(argv[1], "tcpu") == 0) {
		val = TD_MEASURE_VoltageTemperatureExtended(true);
	} else if (strcmp(argv[1], "tex") == 0) {
		val = 999;
	} else {
		LWB_SERIAL_Printfln("%s : Unknown option", argv[1]);
		return 1;
	}
	LWB_SERIAL_Printfln("%s : %d", argv[1], val);
	return 0;
}

static int Button(int argc, const char * const * argv) {
	LWB_SERIAL_Println("Waiting for button press.");
	LWB_SERIAL_Println("Enter [q] to stop");
	do {
		switch (LWB_BUTTON_Process()) {
			case LWB_BUTTON_NO_ACTION:
				break;
			case LWB_BUTTON_PRESSED_POWER_SWITCH:
				LWB_SERIAL_Println("Button pressed : power");
				break;
			case LWB_BUTTON_PRESSED_CALIBRATION:
				LWB_SERIAL_Println("Button pressed : calibration");
				break;
		}
	} while (LWB_CONSOLE_LoopUntilQuit(true));
	return 0;
}

static int Gps(int argc, const char * const * argv) {
	LWB_SERIAL_Println("Keep [q] pressed to stop");
	LWB_GPS_PowerOn(0);
	int c;
	int nmeaBufferIndex = 0;
	char nmeaBuffer[GPS_BUFFER_SIZE] = {0};
	bool gotAMessage = false;
	while (1) {
		c = LWB_SERIAL_GetChar();
		if (c < 0) {
			// continue
		} else if (c == 'q') {
			break;
		} else if (c == '$') {
			nmeaBufferIndex = 0;
			memset(nmeaBuffer, 0, GPS_BUFFER_SIZE);
		} else if (c == '\n') {
			if (strncmp(nmeaBuffer, "GPGGA,", 6) == 0) {
				gotAMessage = true;
				LWB_SERIAL_Println(nmeaBuffer);
			}
		} else if (nmeaBufferIndex < GPS_BUFFER_SIZE) {
			nmeaBuffer[nmeaBufferIndex] = c;
			nmeaBufferIndex++;
		} else {
			LWB_SERIAL_Debug("NMEA buffer overflow\n");
		}
		TD_RTC_Sleep();
	}
	LWB_GPS_PowerOff();
	return gotAMessage ? 0 : 1;
}

static int Imu(int argc, const char * const * argv) {
	LWB_CONSOLE_CHECKARGC(argc, 1);
	bool success = true;
	if (strcmp(argv[1], "read") == 0) {
		LWB_SERIAL_Println("Reading IMU, 10Hz sampling rate.");
		LWB_SERIAL_Println("Enter [q] to stop");
		LWB_SERIAL_Println("COMPASS");
		LWB_SERIAL_Println("X\tY\tZ\tdir");
		int cX, cY, cZ;
		float heading;
		do {
			TD_RTC_Delay(T100MS);
			if(LWB_COMPASS_GetRaw(&cX, &cY, &cZ)) {
				heading = LWB_COMPASS_GetHeadingFromRaw(cX, cY, cZ);
				heading = heading * 180. / M_PI;
				if (heading < 0.) heading += 360.;
				LWB_SERIAL_Printf("%d\t%d\t%d\t%d\t", cX, cY, cZ, (int)heading);
			} else {
				LWB_SERIAL_Println("ERROR with compass reading");
				LWB_SERIAL_Print("ERR\tERR\tERR\tERR");
				success = false;
			}
			LWB_SERIAL_Println("");
		} while (LWB_CONSOLE_LoopUntilQuit(false));

	} else if (strcmp(argv[1], "showcal") == 0) {
		LWB_SERIAL_Println("todo");
	} else if (strcmp(argv[1], "setcal") == 0) {
		LWB_SERIAL_Println("todo");
	} else if (strcmp(argv[1], "calibrate") == 0) {
		LWB_SERIAL_Println("Starting calibration in 5 secondes.");
		LWB_SERIAL_Println("Rotate when the led is on.");
		TD_RTC_Delay(5000);
		LWB_LED_Set();
		success = LWB_COMPASS_Calibrate();
		LWB_LED_Clear();
	} else if (strcmp(argv[1], "selftest") == 0) {
		LWB_SERIAL_Println("Running compass selftest");
		if (!LWB_COMPASS_Test()) {
			LWB_SERIAL_Println("Compass selftest FAILED");
			success = false;
		} else {
			LWB_SERIAL_Println("Compass selftest OK");
		}
	}
	return success ? 0 : 1;
}

static int Led(int argc, const char * const * argv) {
	LWB_CONSOLE_CHECKARGC(argc, 1);
	if (strcmp(argv[1], "on") == 0) {
		LWB_LED_Set();
	} else if (strcmp(argv[1], "off") == 0) {
		LWB_LED_Clear();
	} else {
		LWB_SERIAL_Printfln("%s : Unknown option", argv[1]);
		return 1;
	}
	return 0;
}

static int SetMfgData(int argc, const char * const * argv) {
	LWB_CONSOLE_CHECKARGC(argc, 4);

	bool success = true;

	// -- name --
	//char productName[SETTING_PRODUCTNAME_LEN * 4] = {0};
	//char* productName = argv[2];
	//SETTING_PRODUCTNAME

	// -- version --
	//char hwVersion[4] = {0};
	//SETTING_HWVERSION

	// -- device --
	success &= LWB_SETTINGS_Set(SETTING_DEVICEID, strtoul(argv[3], NULL, 10));

	// -- pass --
	success &= LWB_SETTINGS_Set(SETTING_MFGPASS, strtoul(argv[4], NULL, 16));

	success &= LWB_SETTINGS_Save();
	return success ? 0 : 1;
}

static bool waitForDownlink = false;
static bool downlinkReceived = false;
static int downlink_callback(uint8_t *rx_frame, uint8_t length) {
	if (rx_frame == 0) {
		// Finished receiving
		LWB_SERIAL_Println("RX END");
		waitForDownlink = false;
		// Done
		return 1;
	} else {
		if (length == 0) {

			// Start receiving
			LWB_SERIAL_Println("RX BEGIN");

			// Done
			return 1;
		}

		// Received one good frame
		LWB_SERIAL_Dump("Got downlink: ", rx_frame, length);
		downlinkReceived = true;
		// Done
		return 1;
	}
}

static int Sigfox(int argc, const char * const * argv) {
	int success = true;
	char message[12] = "0123456789AB";
	if (strcmp(argv[1], "id") == 0) {
		LWB_SERIAL_Printfln("SIGFOX ID: %X", TD_SIGFOX_GetId());
	} else if (strcmp(argv[1], "uplink") == 0) {
		LWB_SERIAL_Println("Sending message...");
		success = TD_SIGFOX_SendV1(MODE_FRAME, false, (uint8_t *)message, 12, 2, false, false);
	} else if (strcmp(argv[1], "downlink") == 0) {
		LWB_SERIAL_Println("Sending message...");
		TD_SIGFOX_DOWNLINK_SetUserCallback(downlink_callback);
		waitForDownlink = true;
		downlinkReceived = false;
		TD_SIGFOX_SendV1(MODE_FRAME, false, (uint8_t *)message, 12, 2, true, false);
		LWB_SERIAL_Println("Waiting for downlink...");
		LWB_SERIAL_Println("Enter [q] to abort");
		do {
			TD_SIGFOX_DOWNLINK_Process();
		} while (waitForDownlink && LWB_CONSOLE_LoopUntilQuit(true));
		waitForDownlink = false;
		TD_SIGFOX_DOWNLINK_SetUserCallback(0);
		if (downlinkReceived) {
			LWB_SERIAL_Printfln("downlink RSSI : %d", TD_SIGFOX_DOWNLINK_GetRSSI());
		} else {
			LWB_SERIAL_Println("Timeout. No downlink received");
			success = false;
		}
	} else if (strcmp(argv[1], "showpwr") == 0) {
		LWB_SERIAL_Println("todo");
	} else if (strcmp(argv[1], "setpwr") == 0) {
		LWB_SERIAL_Println("todo");
	} else if (strcmp(argv[1], "cw") == 0) {
		LWB_SERIAL_Println("Starting CW test at 868.13MHz");
		LWB_SERIAL_Println("Enter [q] to stop");
		//TD_SIGFOX_RfPower
		success &= TD_SIGFOX_SendTestPA(0, 1, 0, 868130000);
		while (LWB_CONSOLE_LoopUntilQuit(true)) {};
		success &= TD_SIGFOX_SendTestPA(0, 0, 0, 868130000);
		LWB_SERIAL_Println("stopped");
	} else {
		LWB_SERIAL_Printfln("%s : Unknown option", argv[1]);
		success = false;
	}
	return success ? 0 : 1;
}

static int Speed(int argc, const char * const * argv) {
	LWB_CONSOLE_CHECKARGC(argc, 1);
	bool success = true;
	LWB_SERIAL_Println("Reading speed, 1Hz sampling rate.");
	LWB_SERIAL_Println("Enter [q] to stop");
	LWB_SERIAL_Println("Hz\tkm/h");
	if (strcmp(argv[1], "read") == 0) {
		LWB_PULSECOUNTER_Reset();
		int pulses;
		do {
			TD_RTC_Delay(T1S);
			pulses = LWB_PULSECOUNTER_SampleRaw();
			LWB_SERIAL_Printfln("%d\t%d", pulses, (int)(float)(pulses * SPEED_HZ_TO_KMH));

		} while (LWB_CONSOLE_LoopUntilQuit(false));
	} else if (strcmp(argv[1], "showcal") == 0) {
		LWB_SERIAL_Println("todo");
	} else if (strcmp(argv[1], "setcal") == 0) {
		LWB_SERIAL_Println("todo");
	} else {
		LWB_SERIAL_Printfln("%s : Unknown option", argv[1]);
		success = false;
	}
	return success ? 0 : 1;
}

LWB_CONSOLE_command_t LWB_CONSOLE_commands[] = {
		{"adc", &Adc, "[ vbat | vbatload | vcap | vcpu | tcpu | tex ]"},
		{"button", &Button, ""},
		{"exit", &LWB_CONSOLE_ExitCmd, ""},
		{"gps", &Gps, ""},
		{"help", &LWB_CONSOLE_HelpCmd, ""},
		{"imu", &Imu, "[ read | showcal | setcal | calibrate | selftest ]"},
		{"led", &Led, "[ on | off ]"},
		{"mfg", &SetMfgData, "**reserved**"},
		{"mode", &Todo, "[ windbird | wmo ]"},
		{"sigfox", &Sigfox, "[ id | uplink | downlink | showpwr | setpwr | cw ]"},
		{"speed", &Speed, "[ read | showcal | setcal ]"}
};

int LWB_CONSOLE_commandsCount = sizeof( LWB_CONSOLE_commands ) / sizeof( LWB_CONSOLE_command_t );

void LWB_CONSOLE_InitDevices() {
	LWB_LED_Init();
	LWB_GPS_Init();
	LWB_BUTTON_Init();
	LWB_PULSECOUNTER_Init();
	LWB_I2C_Init();
	LWB_COMPASS_Init();
}
