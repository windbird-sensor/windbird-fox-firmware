/**************************************************************************
 * @file WB_sigfox.h
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

#ifndef WB_SIGFOX_H_
#define WB_SIGFOX_H_
#include "wb_gps.h"
#include "wb_reports.h"

void WB_SIGFOX_Init();
void WB_SIGFOX_StartupMessage(float windSpeed, float windHeading, uint32_t vbatNoLed, uint32_t vbatLed, uint32_t vcapEarlyBoot, uint32_t vcapNow);
void WB_SIGFOX_ShutdownMessage (uint32_t vbatNoLed, uint32_t vbatLed, uint32_t vcapNow);
void WB_SIGFOX_ReportMessage(WB_REPORTS_Report_t *report, uint8_t reportCount, WB_REPORTS_DiagReport_t *diagReport);
void WB_SIGFOX_LocationMessage (WB_GPS_Fix_t fix);
void WB_SIGFOX_LocationFailureMessage ();
#endif /* WB_SIGFOX_H_ */
