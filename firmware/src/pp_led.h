/**************************************************************************
 * @file pp_led.h
 * @brief LED API for PIOUPIOU's firmware
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
#ifndef PP_LED_H_
#define PP_LED_H_

void PP_LED_Init();
void PP_LED_Test();
void PP_LED_Clear();
void PP_LED_Set();
void PP_LED_StartBlink(uint8_t seconds, uint16_t ticks);
void PP_LED_StopBlink();

#endif /* PP_LED_H_ */
