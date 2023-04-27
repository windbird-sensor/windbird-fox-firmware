/**************************************************************************
 * @file WB_compass.h
 * @brief Compass Sensor API for WINDBIRD's firmware
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

#ifndef LWB_COMPASS_H_
#define LWB_COMPASS_H_

#include <stdint.h>
#include <stdbool.h>

void LWB_COMPASS_Init();
bool LWB_COMPASS_Calibrate();
bool LWB_COMPASS_Test();
bool LWB_COMPASS_GetRaw(int *x, int *y, int *z);
float LWB_COMPASS_GetHeading();
float LWB_COMPASS_GetHeadingFromRaw(int x, int y, int z);

#endif /* WB_COMPASS_H_ */
