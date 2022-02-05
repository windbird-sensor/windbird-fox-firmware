/**************************************************************************
 * @file WB_button.h
 * @brief Button API for WINDBIRD's firmware
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
 
#ifndef WB_BUTTON_H_
#define WB_BUTTON_H_

typedef enum {
	WB_BUTTON_NO_ACTION,
	WB_BUTTON_PRESSED_POWER_SWITCH,
	WB_BUTTON_PRESSED_CALIBRATION,
	} WB_BUTTON_State_t;

void WB_BUTTON_Init();

WB_BUTTON_State_t WB_BUTTON_Loop();

#endif /* WB_BUTTON_H_ */
