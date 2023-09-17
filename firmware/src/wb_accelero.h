/*
 * wb_accelero.h
 *
 *  Created on: 26 Aug 2023
 *      Author: windbird-dev
 */

#ifndef WB_ACCELERO_H_
#define WB_ACCELERO_H_

#include <stdint.h>
#include <stdbool.h>

void WB_ACCELERO_Init();
bool WB_ACCELERO_Test();
void WB_ACCELERO_TestCalibration();
bool WB_ACCELERO_GetRaw(float *x, float *y, float *z);

#endif /* WB_ACCELERO_H_ */
