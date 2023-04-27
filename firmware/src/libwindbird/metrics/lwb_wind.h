/*
 * lwb_wind.h
 *
 *  Created on: 10 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_WIND_H_
#define LWB_WIND_H_

#include <stdint.h>
#include <stdbool.h>

void LWB_WIND_Reset();
void LWB_WIND_PushSample(uint16_t sample);
void LWB_WIND_PushExtreme(uint16_t sample);
float LWB_WIND_GetExtremeMax(int period);
float LWB_WIND_GetExtremeMin(int period);
float LWB_WIND_GetAverage(int offset, int length);
float LWB_WIND_GetStandardDeviation(int offset, int length, float average);

#endif /* LWB_WIND_H_ */
