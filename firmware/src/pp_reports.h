/**************************************************************************
 * @file pp_reports.h
 * @brief Reports API for PIOUPIOU's firmware
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

#ifndef PP_REPORTS_H_
#define PP_REPORTS_H_

#include <stdint.h>

typedef struct {
	float speedMax;
	float speedMin;
	float speedAvg;
	float headingX;
	float headingY;
	float headingAvg;
	float tempAvg;
	uint8_t samplesCount;
} PP_REPORTS_Report_t;

void PP_REPORTS_Init();
void PP_REPORTS_Start();
void PP_REPORTS_Stop();
void PP_REPORTS_Pause();
void PP_REPORTS_Resume();

#endif /* PP_REPORTS_H_ */
