/**************************************************************************
 * @file WB_pressure.h
 * @brief Pressure Sensor API for WINDBIRD's firmware
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

#ifndef WB_PRESSURE_H_
#define WB_PRESSURE_H_

void WB_PRESSURE_Init();
void WB_PRESSURE_Shutdown();

bool WB_PRESSURE_Test(float* pressure);
float WB_PRESSURE_Get();

#endif /* WB_PRESSURE_H_ */
