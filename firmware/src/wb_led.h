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
#ifndef WB_LED_H_
#define WB_LED_H_

#define LED_PORT gpioPortC
#define LED_BIT 15

typedef enum {
	WB_LED_FADE_IN,
	WB_LED_FADE_OUT,
	} WB_LED_FadeDirection_t;

void WB_LED_Init();
void WB_LED_Test();
void WB_LED_Clear();
void WB_LED_Set();
void WB_LED_StartBlink(uint8_t seconds, uint16_t ticks);
void WB_LED_StopBlink();
void WB_LED_Fade(WB_LED_FadeDirection_t direction, uint32_t duration);

#endif /* WB_LED_H_ */
