/**************************************************************************
 * @file debug.h
 * @brief Debug functions for WINDBIRD's firmware
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
#ifndef LWB_SERIAL_H_
#define LWB_SERIAL_H_

#include "../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X
	#include "../platforms/td120x/lwb_td120x_serial.h"
#else
	#error Not implemented for this platform
#endif



#endif /* LWB_SERIAL_H_ */
