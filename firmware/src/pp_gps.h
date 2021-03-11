/**************************************************************************
 * @file pp_gps.h
 * @brief GPS API for PIOUPIOU's firmware
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
 
#ifndef PP_GPS_H_
#define PP_GPS_H_

typedef struct {
	int32_t latitude;
	int32_t longitude;
	int16_t altitude;
	uint8_t hdop;
} PP_GPS_Fix_t;

void PP_GPS_Init();
bool PP_GPS_Locate();
void PP_GPS_PowerOn(uint16_t timeout);
void PP_GPS_PowerOff();

#endif /* PP_GPS_H_ */
