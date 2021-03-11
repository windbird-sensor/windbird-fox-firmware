/**************************************************************************
 * @file pp_pressure.h
 * @brief Pressure Sensor API for PIOUPIOU's firmware
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

#ifndef PP_PRESSURE_H_
#define PP_PRESSURE_H_

void PP_PRESSURE_Init();
void PP_PRESSURE_Shutdown();

bool PP_PRESSURE_Test(float* pressure);
float PP_PRESSURE_Get();

#endif /* PP_PRESSURE_H_ */
