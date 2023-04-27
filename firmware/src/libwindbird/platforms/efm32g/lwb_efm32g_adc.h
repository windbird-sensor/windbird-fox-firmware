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

#ifndef LWB_EFM32G_ADC_H_
#define LWB_EFM32G_ADC_H_

#include "../../libwindbird.h"

#ifdef LWB_PLATFORM_EFM32G

#include <em_adc.h>

uint32_t LWB_ADC_SampleMillivolts(ADC_SingleInput_TypeDef input, uint32_t rTop, uint32_t rBottom);
uint32_t LWB_ADC_SampleRaw(ADC_SingleInput_TypeDef input);

#endif /* LWB_PLATFORM_EFM32G */

#endif /* LWB_EFM32G_ADC_H_ */
