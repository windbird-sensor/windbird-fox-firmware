/**************************************************************************
 * @file pp_button.h
 * @brief Button API for PIOUPIOU's firmware
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
 
#ifndef PP_BUTTON_H_
#define PP_BUTTON_H_

typedef enum {
	PP_BUTTON_NO_ACTION,
	PP_BUTTON_PRESSED_POWER_SWITCH,
	PP_BUTTON_PRESSED_CALIBRATION,
	} PP_BUTTON_State_t;

void PP_BUTTON_Init();

PP_BUTTON_State_t PP_BUTTON_Loop();

#endif /* PP_BUTTON_H_ */
