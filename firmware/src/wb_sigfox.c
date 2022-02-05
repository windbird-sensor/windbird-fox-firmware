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

#include <td_sigfox.h>
#include <td_measure.h>
#include "wb_sigfox.h"
#include "wb_debug.h"

//#define DEBUG_NO_SIGFOX

#define SIGFOX_RETRIES 2
// 2 retries for compliance with sigfox protocol

#define SIGFOX_MAX_TX_POWER 11
// limiting RF power to 11dBm so we get 14dBm max radiation
// for compliance with ETSI EN 300-220

#ifdef DEBUG_NO_SIGFOX
#define SIGFOX_SEND(...) WB_DEBUG_DUMP("sigfox", __VA_ARGS__)
#else
#define SIGFOX_SEND(...) \
  WB_DEBUG_DUMP("sigfox= ", __VA_ARGS__); \
  TD_SIGFOX_SendV1(MODE_FRAME, 0, __VA_ARGS__, SIGFOX_RETRIES, 0, 0)
#endif

#define SIGFOX_STARTUP_MESSAGE 0x10
#define SIGFOX_SHUTDOWN_MESSAGE 0x20
#define SIGFOX_MONITORING_MESSAGE 0x30
#define SIGFOX_CALIBRATION_MESSAGE_A 0x4A
#define SIGFOX_CALIBRATION_MESSAGE_B 0x4B
#define SIGFOX_LOCATION_FAILURE_MESSAGE 0xF0
#define SIGFOX_LOCATION_MESSAGE 0xFF

static uint8_t message[12];


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

static uint8_t EncodeWindHeading (float heading) {
  return (int)(float)(heading / 0.392699082 + 16.5) % 16;
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
        // should never be < -50 or > 100
  return (uint8_t)(float)(temperature + 50.5);
}

static uint8_t EncodeVoltage(float milliVolts) {
  return (uint8_t)(float)((milliVolts / 10. + 0.5) - 200.);
}

static void EncodeFloat(uint8_t *target, float *value) {
  memcpy(target, value, sizeof(float));
}

void WB_SIGFOX_Init () {

  TD_SIGFOX_RfPower(SIGFOX_MAX_TX_POWER);

}

void WB_SIGFOX_StartupMessage (float windSpeed, float windHeading) {

  message[0]= SIGFOX_STARTUP_MESSAGE | EncodeWindHeading(windHeading);
  message[1]=EncodeWindSpeed(windSpeed);
  message[2]=TD_MEASURE_VoltageTemperature(false); //voltage

  // embed compilation date in startup message
  //MMM DD YYYY
  //01234567890
  message[3]=__DATE__[0];
  message[4]=__DATE__[1];
  message[5]=__DATE__[2];
  message[6]=__DATE__[4];
  message[7]=__DATE__[5];
  message[8]=__DATE__[9];
  message[9]=__DATE__[10];

  SIGFOX_SEND(message, 10);
}

void WB_SIGFOX_ShutdownMessage () {
  message[0]= SIGFOX_SHUTDOWN_MESSAGE;
  message[1]=TD_MEASURE_VoltageTemperature(false); //voltage
  SIGFOX_SEND(message, 2);
}

void WB_SIGFOX_LocationMessage (WB_GPS_Fix_t fix) {
    //message[0]=SIGFOX_LOCATION_MESSAGE;
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
  SIGFOX_SEND(message, 11);
}

void WB_SIGFOX_LocationFailureMessage () {
  message[0]=SIGFOX_LOCATION_FAILURE_MESSAGE;
  SIGFOX_SEND(message, 1);
}

void WB_SIGFOX_ReportMessage(WB_REPORTS_Report_t *report, uint8_t reportCount) {

  uint8_t headingAvg[reportCount];
  uint8_t speedMax[reportCount];
  uint8_t speedMin[reportCount];
  uint8_t speedAvg[reportCount];
  float tempAvg = 0;

  int i;
  for (i=0; i<reportCount; i++) {
    headingAvg[i] = EncodeWindHeading(report[i].headingAvg);
    speedMax[i] = EncodeWindSpeed(report[i].speedMax);
    speedMin[i] = EncodeWindSpeed(report[i].speedMin);
    speedAvg[i] = EncodeWindSpeed(report[i].speedAvg);
    tempAvg += report[i].tempAvg;
  }
  tempAvg /= 3;

  message[0] = speedMin[0];
  message[1] = speedMin[1];
  message[2] = speedMin[2];
  message[3] = speedAvg[0];
  message[4] = speedAvg[1];
  message[5] = speedAvg[2];
  message[6] = speedMax[0];
  message[7] = speedMax[1];
  message[8] = speedMax[2];
  message[9] = (headingAvg[0] << 4) | headingAvg[1];
  message[10] = (headingAvg[2] << 4);
  message[11] = EncodeTemperature(tempAvg);

  SIGFOX_SEND(message, 12);
}

void WB_SIGFOX_MonitoringMessage(float tempMin, float tempAvg, float tempMax, float voltageMin, float voltageAvg, float voltageMax) {

  message[0]=SIGFOX_MONITORING_MESSAGE;
  message[1]=EncodeTemperature(tempMin);
  message[2]=EncodeTemperature(tempAvg);
  message[3]=EncodeTemperature(tempMax);
  message[4]=EncodeVoltage(voltageMin);
  message[5]=EncodeVoltage(voltageAvg);
  message[6]=EncodeVoltage(voltageMax);

  SIGFOX_SEND(message, 7);
}

void WB_SIGFOX_CalibrationMessage(float yOffset, float yScale, float zOffset, float zScale) {

  message[0]=SIGFOX_CALIBRATION_MESSAGE_A;
  EncodeFloat(&message[1], &yOffset);
  EncodeFloat(&message[5], &yScale);
  SIGFOX_SEND(message, 9);

  message[0]=SIGFOX_CALIBRATION_MESSAGE_B;
  EncodeFloat(&message[1], &zOffset);
  EncodeFloat(&message[5], &zScale);
  SIGFOX_SEND(message, 9);

}
