/**************************************************************************
 * @file pp_reports.c
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

#include <math.h>
#include <td_scheduler.h>
#include <td_measure.h>
#include <td_rtc.h>
#include "pp_debug.h"
#include "pp_reports.h"
#include "pp_compass.h"
#include "pp_propeller.h"
#include "pp_sigfox.h"

#define SAMPLE_PERIOD 2
#define SAMPLE_PER_REPORT 120
#define REPORT_COUNT 3

static bool isPaused;

static PP_REPORTS_Report_t report[REPORT_COUNT];
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

	float windSpeed = PP_PROPELLER_GetSpeed();
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

	float windHeading = PP_COMPASS_GetHeading();
	report[reportIndex].headingX += windSpeed * cos(windHeading);
	report[reportIndex].headingY += windSpeed * sin(windHeading);


	float temperature = (TD_MEASURE_VoltageTemperatureExtended(true) / 10.);

	report[reportIndex].tempAvg += temperature;

	PP_DEBUG("sample\t%d\t%d\t%d\t%d\t%d\n",
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

		PP_DEBUG("REPORT\t%d\t%d\t%d\t%d\t%d\t%d\n",
				(int)(report[reportIndex].speedAvg*10.),
				(int)(report[reportIndex].speedMax*10.),
				(int)(report[reportIndex].speedMin*10.),
				(int)(float)(report[reportIndex].headingAvg/M_PI*180.),
				(int)report[reportIndex].tempAvg,
				reportIndex);

		if (reportIndex == REPORT_COUNT-1) {
			PP_SIGFOX_ReportMessage(report, REPORT_COUNT);
			reportIndex=0;
		} else {
			reportIndex++;
		}

		ResetCurrent();
	}
}

void PP_REPORTS_Start() {
	isPaused = false;
	reportIndex = 0;
	ResetCurrent();
	PP_PROPELLER_Reset();
	if (samplingTimer != 0xFF) {
		TD_SCHEDULER_Remove(samplingTimer);
	}
	samplingTimer = TD_SCHEDULER_Append(SAMPLE_PERIOD, 0, 0, TD_SCHEDULER_INFINITE, SamplingTimer, 0);
	if (samplingTimer == 0xFF) PP_DEBUG("ERROR initializing REPORT samplingTimer\n");
}

void PP_REPORTS_Stop() {
	if (samplingTimer != 0xFF) {
		TD_SCHEDULER_Remove(samplingTimer);
		samplingTimer = 0xFF;
	}
}

void PP_REPORTS_Pause() {
	isPaused = true;
}

void PP_REPORTS_Resume() {
	PP_PROPELLER_GetSpeed();
	// clean the pulse count (we may have missed some RTC overflows)

	isPaused = false;
}

void PP_REPORTS_Init() {
	samplingTimer = 0xFF;
	isPaused = true;
}
