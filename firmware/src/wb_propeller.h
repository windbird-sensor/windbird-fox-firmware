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
#ifndef WB_PROPELLER_H_
#define WB_PROPELLER_H_

void WB_PROPELLER_Init();
float WB_PROPELLER_GetSpeed();
void WB_PROPELLER_Reset();

#endif /* WB_PROPELLER_H_ */
