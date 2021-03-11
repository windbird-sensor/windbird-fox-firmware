/**************************************************************************
 * @file pp_sigfox.h
 * @brief Sigfox API for PIOUPIOU's firmware
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

#ifndef PP_SIGFOX_H_
#define PP_SIGFOX_H_
#include "pp_gps.h"
#include "pp_reports.h"

void PP_SIGFOX_Init();
void PP_SIGFOX_StartupMessage(float windSpeed, float windHeading);
void PP_SIGFOX_ShutdownMessage();
void PP_SIGFOX_ReportMessage(PP_REPORTS_Report_t *report, uint8_t reportCount);
void PP_SIGFOX_LocationMessage (PP_GPS_Fix_t fix);
void PP_SIGFOX_LocationFailureMessage ();
void PP_SIGFOX_MonitoringMessage(float tempMin, float tempAvg, float tempMax, float voltageMin, float voltageAvg, float voltageMax);
void PP_SIGFOX_CalibrationMessage(float yOffset, float yScale, float zOffset, float zScale);
#endif /* PP_SIGFOX_H_ */
