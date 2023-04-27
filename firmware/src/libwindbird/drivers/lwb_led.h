/**************************************************************************
 * @file WB_led.h
 * @brief LED API for WINDBIRD's firmware
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
#ifndef LWB_LED_H_
#define LWB_LED_H_

#include <stdint.h>

typedef enum {
	LWB_LED_FADE_IN,
	LWB_LED_FADE_OUT,
	} LWB_LED_FadeDirection_t;

void LWB_LED_Init();
void LWB_LED_Clear();
void LWB_LED_Set();
void LWB_LED_Fade(LWB_LED_FadeDirection_t direction, uint32_t duration);

#endif /* LWB_LED_H_ */
