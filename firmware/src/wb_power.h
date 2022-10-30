/**************************************************************************
 * @file wb_power.h
 * @brief POWER API for WINDBIRD's firmware
 * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2022 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of WINDBIRD WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/windbird-sensor/windbird-firmware/blob/main/README.md
 *
 ******************************************************************************/

#ifndef WB_POWER_H_
#define WB_POWER_H_

#define VAUX_PORT gpioPortC
#define VAUX_BIT 14

void WB_POWER_Init();
uint32_t WB_POWER_GetBatteryMillivolts();
uint32_t WB_POWER_GetCapacitorMillivolts();
void WB_POWER_EnableVAUX();
void WB_POWER_DisableVAUX();

#endif /* WB_POWER_H_ */
