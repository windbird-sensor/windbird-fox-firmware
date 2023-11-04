/**************************************************************************
 * @file WB_sigfox.c
 * @brief Sigfox API for WINDBIRD's firmware
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

#include <math.h>

#include <td_sigfox.h>

#include "wb_network.h"
#include "wb_power.h"
#include "wb_debug.h"
#include "wb_led.h"
#include "wb_version.h"
#include "wb_ext_lora.h"

//#define DEBUG_NO_NETWORK

#define NETWORK_SIGFOX_RETRIES 2
// 2 retries for compliance with sigfox protocol

#define NETWORK_RETRIES_DEFAULT -1

#define NETWORK_MAX_TX_POWER 11
// limiting RF power to 11dBm so we get 14dBm max radiation
// for compliance with ETSI EN 300-220

#define NETWORK_STARTUP_MESSAGE 0x10
#define NETWORK_SHUTDOWN_MESSAGE 0x20
#define NETWORK_SHORT_REPORT_MESSAGE 0x50
#define NETWORK_LOCATION_FAILURE_MESSAGE 0xF0
#define NETWORK_LOCATION_MESSAGE 0xFF

#define NETWORK_DIAG_VERSION 0x01
#define NETWORK_DIAG_FIELDS_COUNT 15
#define NETWORK_DIAG_ERROR_VAL 0xFF


#define NETWORK_DIAG_VERSION_FIELD 0
#define NETWORK_DIAG_VCAP_BEFORE_FIELD 1
#define NETWORK_DIAG_VCAP_AFTER_FIELD 2
#define NETWORK_DIAG_VBAT_BEFORE_LED_FIELD 3
#define NETWORK_DIAG_VBAT_BEFORE_NOLED_FIELD 4
#define NETWORK_DIAG_VBAT_AFTER_LED_FIELD 5
#define NETWORK_DIAG_VBAT_AFTER_NOLED_FIELD 6
#define NETWORK_DIAG_ACC_X_FIELD 7
#define NETWORK_DIAG_ACC_Y_FIELD 8
#define NETWORK_DIAG_ACC_Z_FIELD 9
#define NETWORK_DIAG_HEAD_ERR_AVG_FIELD 10
#define NETWORK_DIAG_HEAD_ERR_MAX_FIELD 11
#define NETWORK_DIAG_TEMP_AVG_FIELD 12
#define NETWORK_DIAG_TEMP_MIN_FIELD 13
#define NETWORK_DIAG_TEMP_MAX_FIELD 14


int vcapBefore = 0;
int vcapAfter = 0;
int vbatBeforeLed = 0;
int vcapBeforeNoLed = 0;
int vbatAfterLed = 0;
int vbatAfterNoLed = 0;
int initVoltage = true;

static uint8_t message[12];

static uint8_t internalSequenceNumber = 0;

static uint8_t EncodeWindSpeed (float speed) {
  uint8_t intSpeed;

  if (speed < 10.) {
    // 0 to 9.75 kmh : 0.25 km/h resolution
    intSpeed = (int)(float)(speed * 4. + 0.5);
  } else if (speed < 80.) {
    // 10 to 79.5 kmh  : 0.5 km/h resolution
    intSpeed = (int)(float)(speed * 2. + 0.5) + 20;
  } else if (speed < 120.) {
    // 80 to 119 kmh  : 1 km/h resolution
    intSpeed = (int)(float)(speed + 0.5) + 100;
  } else if (speed < 190.) {
    // 120 to 188 kmh  : 2 km/h resolution
    intSpeed = (int)(float)(speed / 2. + 0.5) + 160;
  } else {
    // 190 or + : out of range
    intSpeed = 0xFF;
  }
  return intSpeed;
}

static uint8_t EncodeWindHeadingLowRes (float radians) {
  // 16x 22.5°
  float degrees = radians / M_PI * 180. + 360.;
  int degreesInt = (int)(float)round(degrees / 22.5);
  degreesInt %= 16;
  return degreesInt;
}

static uint8_t EncodeWindHeadingHighRes (float radians) {
  // 180x 2°
  float degrees = radians / M_PI * 180 + 360.;
  int degreesInt = (int)(float)round(degrees / 2.);
  degreesInt %= 180;
  return degreesInt;
}

static uint16_t EncodePressure (float pressure) {
  // 4096 (2^12) steps from 500 to 1100 hPa
        int16_t intPressure = (int)(float)((pressure - 500.) / 600. * 4096. + 0.5);
        if (intPressure < 0) {
          return 0;
        } else if (intPressure > 4095) {
          return 4095;
        }
        return intPressure;
}


static uint8_t EncodeTemperature(float temperature) {
	// -50 to 75degC, 0.5degC resolution
	temperature += 50.;
	temperature *= 2.;
	int tempInt = round(temperature);
	if (tempInt < 0) return 0;
	if (temperature > 254) return 254;
	return tempInt;
}

static uint8_t EncodeVoltage(float milliVolts) {
	// 1.75 to 4.3 V.
	milliVolts /= 10.;
	milliVolts -= 175.;
	int millivoltsInt = round(milliVolts);
	if (millivoltsInt < 0) return 0;
	if (millivoltsInt > 254) return 254;
	return millivoltsInt;
}

static uint8_t EncodeAccelero(float acc) {
	// already -128 to 127
	int accInt = round(acc) + 128.;
	if (accInt < 0) return 0;
	if (accInt > 254) return 254;
	return accInt;
}

static uint8_t EncodeHeadingError(float error) {
	// radius error, always positive
	// 0.010 = 1%
	// 0.001 to 255%
	error *= 1000.;
	int errorInt = round(error);
	if (errorInt < 0) return 0; // should never happen
	if (errorInt > 254) return 254;
	return errorInt;
}


static void EncodeFloat(uint8_t *target, float *value) {
  memcpy(target, value, sizeof(float));
}

static int DiagBatteryMillivoltsLedOn() {
	WB_LED_Set();
	TD_RTC_Delay(TMS(100));
	int mv = WB_POWER_GetBatteryMillivolts();
	WB_LED_Clear();
	return mv;
}

static uint8_t EncodeDiag(WB_REPORTS_DiagReport_t *r, int sequenceNumber) {
	WB_DEBUG("*** diagReport ***\n");
	WB_DEBUG("accelXAvg: %d\n", (int)r->accelXAvg);
	WB_DEBUG("accelYAvg: %d\n", (int)r->accelYAvg);
	WB_DEBUG("accelZAvg: %d\n", (int)r->accelZAvg);
	WB_DEBUG("accelXSamplesCount: %d\n", (int)r->accelXSamplesCount);
	WB_DEBUG("accelYSamplesCount: %d\n", (int)r->accelYSamplesCount);
	WB_DEBUG("accelZSamplesCount: %d\n", (int)r->accelZSamplesCount);
	WB_DEBUG("headingErrorAvg: %d\n", (int)(float)(r->headingErrorAvg*1000.));
	WB_DEBUG("headingErrorSamplesCount: %d\n", (int)r->headingErrorSamplesCount);
	WB_DEBUG("headingErrorMax: %d\n", (int)(float)(r->headingErrorMax*1000.));
	WB_DEBUG("tempAvg: %d\n", (int)r->tempAvg);
	WB_DEBUG("tempSamplesCount: %d\n", (int)r->tempSamplesCount);
	WB_DEBUG("tempMin: %d\n", (int)r->tempMin);
	WB_DEBUG("tempMax: %d\n", (int)r->tempMax);

	uint8_t out;
	switch (sequenceNumber % NETWORK_DIAG_FIELDS_COUNT) {
		case NETWORK_DIAG_VERSION_FIELD:
			out = NETWORK_DIAG_VERSION; // version
			break;
		case NETWORK_DIAG_ACC_X_FIELD:
			out = EncodeAccelero(r->accelXAvg / r->accelXSamplesCount);
			r->accelXSamplesCount = 0;
			r->accelXAvg = 0;
			break;
		case NETWORK_DIAG_ACC_Y_FIELD:
			out = EncodeAccelero(r->accelYAvg / r->accelYSamplesCount);
			r->accelYSamplesCount = 0;
			r->accelYAvg = 0;
			break;
		case NETWORK_DIAG_ACC_Z_FIELD:
			out = EncodeAccelero(r->accelZAvg / r->accelZSamplesCount);
			r->accelZSamplesCount = 0;
			r->accelZAvg = 0;
			break;
		case NETWORK_DIAG_HEAD_ERR_AVG_FIELD:
			out = EncodeHeadingError(r->headingErrorAvg / r->headingErrorSamplesCount);
			r->headingErrorAvg = 0;
			r->headingErrorSamplesCount = 0;
			break;
		case NETWORK_DIAG_HEAD_ERR_MAX_FIELD:
			out = EncodeHeadingError(r->headingErrorMax);
			r->headingErrorMax = -9999;
			break;
		case NETWORK_DIAG_TEMP_AVG_FIELD:
			out = EncodeTemperature(r->tempAvg / r->tempSamplesCount);
			r->tempAvg = 0;
			r->tempSamplesCount = 0;
			break;
		case NETWORK_DIAG_TEMP_MIN_FIELD:
			out = EncodeTemperature(r->tempMin);
			r->tempMin = 9999;
			break;
		case NETWORK_DIAG_TEMP_MAX_FIELD:
			out = EncodeTemperature(r->tempMax);
			r->tempMax = -9999;
			break;
		case NETWORK_DIAG_VCAP_BEFORE_FIELD:
			out = EncodeVoltage(vcapBefore);
			break;
		case NETWORK_DIAG_VCAP_AFTER_FIELD:
			out = EncodeVoltage(vcapAfter);
			break;
		case NETWORK_DIAG_VBAT_BEFORE_LED_FIELD:
			out = EncodeVoltage(vbatBeforeLed);
			break;
		case NETWORK_DIAG_VBAT_BEFORE_NOLED_FIELD:
			out = EncodeVoltage(vcapBeforeNoLed);
			break;
		case NETWORK_DIAG_VBAT_AFTER_LED_FIELD:
			out = EncodeVoltage(vbatAfterLed);
			break;
		case NETWORK_DIAG_VBAT_AFTER_NOLED_FIELD:
			out = EncodeVoltage(vbatAfterNoLed);
			break;
		default:
			WB_DEBUG("unexpected seq number modulo\n");
			out = NETWORK_DIAG_ERROR_VAL;
	}
	WB_DEBUG("OUT %d : %02x\n", sequenceNumber % NETWORK_DIAG_FIELDS_COUNT, out);
	return out;
}

static void Send(uint8_t* message, int length, int retries) {
	if (WB_EXT_LORA_IsActive()) {
			message[length] = internalSequenceNumber;
			internalSequenceNumber++;
			length++;
		WB_DEBUG_DUMP("lora ", message, length);
		#ifndef DEBUG_NO_NETWORK
			// retries arg is ignored for LoRa
			WB_EXT_LORA_Send(message, length);
		#endif
	} else {
		WB_DEBUG_DUMP("sigfox ", message, length);
		#ifndef DEBUG_NO_NETWORK
			if (retries == -1) retries = NETWORK_SIGFOX_RETRIES;
			TD_SIGFOX_SendV1(MODE_FRAME, 0, message, length, retries, 0, 0);
		#endif
	}
}

static int GetSequenceNumber() {
	if (WB_EXT_LORA_IsActive()) {
		return internalSequenceNumber;
	} else {
		return TD_SIGFOX_GetNextSequenceWithoutInc(); // undocumented function
	}
}

void WB_NETWORK_Init () {

  TD_SIGFOX_RfPower(NETWORK_MAX_TX_POWER);

}

void WB_NETWORK_StartupMessage (float windSpeed, float windHeading, uint32_t vbatNoLed, uint32_t vbatLed, uint32_t vcapEarlyBoot, uint32_t vcapNow) {

  //message[0]=NETWORK_STARTUP_MESSAGE
  message[0]=EncodeWindHeadingHighRes(windHeading);
  message[1]=EncodeWindSpeed(windSpeed);
  message[2]=EncodeVoltage(vbatNoLed);
  message[3]=EncodeVoltage(vbatLed);
  message[4]=EncodeVoltage(vcapNow);
  message[5]=EncodeVoltage(vcapEarlyBoot);
  message[6]=WB_FIRMWARE_VERSION & 0xFF;
  message[7]=(WB_FIRMWARE_VERSION >> 8) & 0xFF;
  message[8]=(WB_FIRMWARE_VERSION >> 16) & 0xFF;
  message[9]=(WB_FIRMWARE_VERSION >> 24) & 0xFF;
  Send(message, 10, NETWORK_RETRIES_DEFAULT);
}

void WB_NETWORK_ShutdownMessage (uint32_t vbatNoLed, uint32_t vbatLed, uint32_t vcapNow) {
  message[0]= NETWORK_SHUTDOWN_MESSAGE;
  message[1]=EncodeVoltage(vbatNoLed);
  message[2]=EncodeVoltage(vbatLed);
  message[3]=EncodeVoltage(vcapNow);
  Send(message, 4, NETWORK_RETRIES_DEFAULT);
}

void WB_NETWORK_LocationMessage (WB_GPS_Fix_t fix) {
    //message[0]=NETWORK_LOCATION_MESSAGE;
    message[0]=fix.latitude & 0xFF;
    message[1]=(fix.latitude >> 8) & 0xFF;
    message[2]=(fix.latitude >> 16) & 0xFF;
    message[3]=(fix.latitude >> 24) & 0xFF;
    message[4]=fix.longitude & 0xFF;
    message[5]=(fix.longitude >> 8) & 0xFF;
    message[6]=(fix.longitude >> 16) & 0xFF;
    message[7]=(fix.longitude >> 24) & 0xFF;
    message[8]=(fix.altitude) & 0xFF;
    message[9]=(fix.altitude >> 8) & 0xFF;
    message[10]=fix.hdop;
    Send(message, 11, NETWORK_RETRIES_DEFAULT);
}

void WB_NETWORK_LocationFailureMessage () {
  message[0]=NETWORK_LOCATION_FAILURE_MESSAGE;
  Send(message, 1, NETWORK_RETRIES_DEFAULT);
}

void WB_NETWORK_ReportMessage(WB_REPORTS_Report_t *report, uint8_t reportCount, WB_REPORTS_DiagReport_t *diagReport) {

	int sequenceNumber = GetSequenceNumber();
	WB_DEBUG("next sequence: %d\n", sequenceNumber);

	uint8_t headingAvg[2];
	uint8_t speedMax[2];
	uint8_t speedMin[2];
	uint8_t speedAvg[2];

	if (sequenceNumber % NETWORK_DIAG_FIELDS_COUNT == 0 || initVoltage) {
		vcapBefore = WB_POWER_GetCapacitorMillivolts();
		vcapBeforeNoLed = WB_POWER_GetBatteryMillivolts();
		vbatBeforeLed = DiagBatteryMillivoltsLedOn();
	}

	if (reportCount == 2) {
		int i;
		for (i=0; i<reportCount; i++) {
			headingAvg[i] = EncodeWindHeadingLowRes(report[i].headingAvg);
			speedMax[i] = EncodeWindSpeed(report[i].speedMax);
			speedMin[i] = EncodeWindSpeed(report[i].speedMin);
			speedAvg[i] = EncodeWindSpeed(report[i].speedAvg);
		}

		message[0] = speedMin[0];
		message[1] = speedMin[1];
		message[2] = speedAvg[0];
		message[3] = speedAvg[1];
		message[4] = speedMax[0];
		message[5] = speedMax[1];
		message[6] = (headingAvg[0] << 4) | headingAvg[1];
		message[7] = EncodeDiag(diagReport, sequenceNumber);
		Send(message, 8, NETWORK_RETRIES_DEFAULT);
	} else {
		headingAvg[0] = EncodeWindHeadingHighRes(report[0].headingAvg);
		speedMax[0] = EncodeWindSpeed(report[0].speedMax);
		speedMin[0] = EncodeWindSpeed(report[0].speedMin);
		speedAvg[0] = EncodeWindSpeed(report[0].speedAvg);

		message[0] = NETWORK_SHORT_REPORT_MESSAGE;
		message[1] = speedMin[0];
		message[2] = speedAvg[0];
		message[3] = speedMax[0];
		message[4] = headingAvg[0];
		message[5] = EncodeDiag(diagReport, sequenceNumber);
		Send(message, 6, 0);
	}
	if (sequenceNumber % NETWORK_DIAG_FIELDS_COUNT == 0 || initVoltage) {
		vcapAfter = WB_POWER_GetCapacitorMillivolts();
		vbatAfterNoLed = WB_POWER_GetBatteryMillivolts();
		vbatAfterLed = DiagBatteryMillivoltsLedOn();
		initVoltage = false;
	}
}
