/**************************************************************************
 * @file pp_propeller.h
 * @brief Propeller Sensor API for PIOUPIOU's firmware
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
#ifndef PP_PROPELLER_H_
#define PP_PROPELLER_H_

void PP_PROPELLER_Init();
float PP_PROPELLER_GetSpeed();
void PP_PROPELLER_Reset();

#endif /* PP_PROPELLER_H_ */
