/*
 * lwb_wind.c
 *
 *  Created on: 10 Apr 2023
 *      Author: windbird-dev
 */
#include <math.h>
#include "../libwindbird.h"
#include "../../wb_config.h"

#include "lwb_wind.h"

// We store the pulse count, before conversion to speed.
// We save memory using uint16 vs float32.
// Conversion is done when requesting the data

static volatile uint16_t samples[METRICS_SAMPLES_COUNT] = {0};
static volatile uint16_t extremeMax[METRICS_NPERIODS] = {0};
static volatile uint16_t extremeMin[METRICS_NPERIODS] = {0xFFFF};

static volatile int cursor = 0;

void LWB_WIND_Reset() {
	// can't use memset because of "volatile" keyword
	int i;
	for (i=0; i<METRICS_SAMPLES_COUNT; i++) {
		samples[i] = 0;
	}
	for (i=0; i<METRICS_NPERIODS; i++) {
		extremeMax[i] = 0;
		extremeMin[i] = 0xFFFF;
	}
	cursor = 0;
}

void LWB_WIND_PushSample(uint16_t sample) {
	samples[cursor++] = sample;
}

void LWB_WIND_PushExtreme(uint16_t sample) {
	int period = cursor / METRICS_PER_PERIOD;
	if (sample > extremeMax[period]) extremeMax[period] = sample;
	else if (sample < extremeMin[period]) extremeMin[period] = sample;
}

float LWB_WIND_GetExtremeMax(int period) {
	return extremeMax[period];
}

float LWB_WIND_GetExtremeMin(int period) {
	return extremeMin[period];
}

float LWB_WIND_GetAverage(int offset, int length) {
	if (offset + length > METRICS_SAMPLES_COUNT) return -1.;
	float average = 0.0;
	int i;
	int max = length + offset;
	for (i=offset; i<max; i++) average += samples[i];
	average /= (float) length;
	return average / (float) METRICS_SAMPLES_PER_SECOND * (float) SPEED_HZ_TO_KMH;
}

float LWB_WIND_GetStandardDeviation(int offset, int length, float average) {
	if (offset + length > METRICS_SAMPLES_COUNT) return -1.;
	if (average < 0.) {
		average = LWB_WIND_GetAverage(offset, length);
	} else {
		// convert back to pulses
		average *= (float) METRICS_SAMPLES_PER_SECOND;
		average /= (float) SPEED_HZ_TO_KMH;
	}

	float variance = 0.0;
	int i;
	int max = length + offset;

	for (i=offset; i<max; i++) {
		variance += pow(samples[i] - average, 2);
	}
	variance /= (float) length;
	return sqrt(variance) / (float) METRICS_SAMPLES_PER_SECOND * (float) SPEED_HZ_TO_KMH;
}
