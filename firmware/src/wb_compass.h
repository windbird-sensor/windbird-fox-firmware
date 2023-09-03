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
 
#ifndef WB_COMPASS_H_
#define WB_COMPASS_H_

#include <stdint.h>
#include <stdbool.h>

void WB_COMPASS_Init();
bool WB_COMPASS_Test();
bool WB_COMPASS_GetRaw(float *x, float *y, float *z);
float WB_COMPASS_GetHeading();

#endif /* WB_COMPASS_H_ */
