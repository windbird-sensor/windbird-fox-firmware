/**************************************************************************
 * @file WB_reports.h
 * @brief Reports API for WINDBIRD's firmware
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

#ifndef WB_REPORTS_H_
#define WB_REPORTS_H_

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
} WB_REPORTS_Report_t;

void WB_REPORTS_Init();
void WB_REPORTS_Start();
void WB_REPORTS_Stop();
void WB_REPORTS_Pause();
void WB_REPORTS_Resume();

#endif /* WB_REPORTS_H_ */
