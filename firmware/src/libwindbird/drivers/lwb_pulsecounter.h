/**************************************************************************
 * @file WB_propeller.h
 * @brief Propeller Sensor API for WINDBIRD's firmware
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
#ifndef LWB_PULSECOUNTER_H_
#define LWB_PULSECOUNTER_H_

void LWB_PULSECOUNTER_Init();
uint8_t LWB_PULSECOUNTER_SampleRaw();
void LWB_PULSECOUNTER_Reset();
void LWB_PULSECOUNTER_Sample();
uint16_t LWB_PULSECOUNTER_GetLastSamples(int count);

#endif /* WB_PROPELLER_H_ */
