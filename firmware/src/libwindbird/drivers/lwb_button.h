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

#ifndef LWB_BUTTON_H_
#define LWB_BUTTON_H_

typedef enum {
	LWB_BUTTON_NO_ACTION,
	LWB_BUTTON_PRESSED_POWER_SWITCH,
	LWB_BUTTON_PRESSED_CALIBRATION,
	} LWB_BUTTON_State_t;

void LWB_BUTTON_Init();

LWB_BUTTON_State_t LWB_BUTTON_Process();

#ifdef LWB_PLATFORM_TD120X
#else
	#error Not implemented for this platform
#endif

#endif /* WB_BUTTON_H_ */
