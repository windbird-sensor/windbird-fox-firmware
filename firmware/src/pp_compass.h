/**************************************************************************
 * @file pp_compass.h
 * @brief Compass Sensor API for PIOUPIOU's firmware
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
 
#ifndef PP_COMPASS_H_
#define PP_COMPASS_H_

#include <stdint.h>
#include <stdbool.h>

void PP_COMPASS_Init();
bool PP_COMPASS_Test();
void PP_COMPASS_TestCalibration();
bool PP_COMPASS_GetRaw(int16_t *x, int16_t *y, int16_t *z);
float PP_COMPASS_GetHeading();
void PP_COMPASS_Calibrate();
void PP_COMPASS_SaveCalibration();
void PP_COMPASS_ClearCalibration();

#endif /* PP_COMPASS_H_ */
