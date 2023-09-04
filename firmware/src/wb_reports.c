/**************************************************************************
 * @file WB_reports.c
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

#include <math.h>
#include <td_scheduler.h>
#include <td_measure.h>
#include <td_rtc.h>
#include "wb_debug.h"
#include "wb_reports.h"
#include "wb_compass.h"
#include "wb_propeller.h"
#include "wb_sigfox.h"
#include "wb_runmode.h"

#define SAMPLE_PERIOD 3
#define SAMPLE_PER_REPORT 100
#define REPORT_COUNT 2

static bool isPaused;

static WB_REPORTS_Report_t report[REPORT_COUNT];
static uint8_t samplingTimer;
static uint8_t reportIndex;

static void ResetCurrent() {
	report[reportIndex].speedMax = -999;
	report[reportIndex].speedMin = 999;
	report[reportIndex].speedAvg = 0;
	report[reportIndex].headingX = 0;
	report[reportIndex].headingY = 0;
	report[reportIndex].headingAvg = 0;
	report[reportIndex].tempAvg = 0;
	report[reportIndex].samplesCount = 0;
}

static void SamplingTimer(uint32_t argument, uint8_t repetition) {

	if (isPaused) return;

	float windSpeed = WB_PROPELLER_GetSpeed();
	if (windSpeed < 0) {
		return; // we got a report with dt = 0, so ignore it
	} else if (fabs(windSpeed) < 0.0001) {
		windSpeed = 0.0001; // so we can compute a mean direction
	}

	if (windSpeed > report[reportIndex].speedMax) {
		report[reportIndex].speedMax = windSpeed;
	} else if (windSpeed < report[reportIndex].speedMin) {
		report[reportIndex].speedMin = windSpeed;
	}
	report[reportIndex].speedAvg += windSpeed;

	float windHeading = WB_COMPASS_GetHeading();
	report[reportIndex].headingX += windSpeed * cos(windHeading);
	report[reportIndex].headingY += windSpeed * sin(windHeading);


	float temperature = (TD_MEASURE_VoltageTemperatureExtended(true) / 10.);

	report[reportIndex].tempAvg += temperature;

	WB_DEBUG("sample\t%d\t%d\t%d\t%d\t%d\n",
			(int)(windSpeed*10.),
			(int)(float)(windHeading/M_PI*180.),
			(int)temperature,
			report[reportIndex].samplesCount,
			reportIndex);

	report[reportIndex].samplesCount++;

	if (report[reportIndex].samplesCount == SAMPLE_PER_REPORT) {

		report[reportIndex].headingAvg = atan2(report[reportIndex].headingY, report[reportIndex].headingX);
		report[reportIndex].speedAvg /= SAMPLE_PER_REPORT;
		report[reportIndex].tempAvg /= SAMPLE_PER_REPORT;

		WB_DEBUG("REPORT\t%d\t%d\t%d\t%d\t%d\t%d\n",
				(int)(report[reportIndex].speedAvg*10.),
				(int)(report[reportIndex].speedMax*10.),
				(int)(report[reportIndex].speedMin*10.),
				(int)(float)(report[reportIndex].headingAvg/M_PI*180.),
				(int)report[reportIndex].tempAvg,
				reportIndex);

		switch (WB_RUNMODE_Get()) {
			case MODE_SIGFOX_5M:
				WB_SIGFOX_ReportMessage(report, 1);
				break;
			case MODE_SIGFOX_10M:
				if (reportIndex == REPORT_COUNT-1) {
					WB_SIGFOX_ReportMessage(report, REPORT_COUNT);
					reportIndex=0;
				} else {
					reportIndex++;
				}
				break;
			case MODE_OGN:
				// todo
				break;
		}


		ResetCurrent();
	}
}

void WB_REPORTS_Start() {
	isPaused = false;
	reportIndex = 0;
	ResetCurrent();
	WB_PROPELLER_Reset();
	if (samplingTimer != 0xFF) {
		TD_SCHEDULER_Remove(samplingTimer);
	}
	samplingTimer = TD_SCHEDULER_Append(SAMPLE_PERIOD, 0, 0, TD_SCHEDULER_INFINITE, SamplingTimer, 0);
	if (samplingTimer == 0xFF) WB_DEBUG("ERROR initializing REPORT samplingTimer\n");
}

void WB_REPORTS_Stop() {
	if (samplingTimer != 0xFF) {
		TD_SCHEDULER_Remove(samplingTimer);
		samplingTimer = 0xFF;
	}
}

void WB_REPORTS_Pause() {
	isPaused = true;
}

void WB_REPORTS_Resume() {
	WB_PROPELLER_GetSpeed();
	// clean the pulse count (we may have missed some RTC overflows)

	isPaused = false;
}

void WB_REPORTS_Init() {
	samplingTimer = 0xFF;
	isPaused = true;
}
