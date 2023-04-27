/**************************************************************************
 * @file WB_gps.h
 * @brief GPS API for WINDBIRD's firmware
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

#ifndef LWB_GPS_H_
#define LWB_GPS_H_

#include <stdint.h>
#include <stdbool.h>

#define GPS_BUFFER_SIZE 255

typedef struct {
	int32_t latitude;
	int32_t longitude;
	int16_t altitude;
	uint8_t hdop;
} LWB_GPS_Fix_t;

void LWB_GPS_Init();
bool LWB_GPS_Locate();
void LWB_GPS_PowerOn(uint32_t timeoutAt);
void LWB_GPS_PowerOff();

#endif /* LWB_GPS_H_ */
