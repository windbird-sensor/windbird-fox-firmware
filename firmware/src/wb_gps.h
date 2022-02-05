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
 
#ifndef WB_GPS_H_
#define WB_GPS_H_

typedef struct {
	int32_t latitude;
	int32_t longitude;
	int16_t altitude;
	uint8_t hdop;
} WB_GPS_Fix_t;

void WB_GPS_Init();
bool WB_GPS_Locate();
void WB_GPS_PowerOn(uint16_t timeout);
void WB_GPS_PowerOff();

#endif /* WB_GPS_H_ */
