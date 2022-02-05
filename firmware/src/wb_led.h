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

void WB_LED_Init();
void WB_LED_Test();
void WB_LED_Clear();
void WB_LED_Set();
void WB_LED_StartBlink(uint8_t seconds, uint16_t ticks);
void WB_LED_StopBlink();

#endif /* WB_LED_H_ */
