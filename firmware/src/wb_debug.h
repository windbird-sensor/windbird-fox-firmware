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
#ifndef WB_DEBUG_H_
#define WB_DEBUG_H_

#include <td_printf.h>

#define DEBUG_WINDBIRD

#ifdef DEBUG_WINDBIRD
#define WB_DEBUG(...) tfp_printf(__VA_ARGS__)
#define WB_DEBUG_DUMP(...) tfp_dump(__VA_ARGS__)
#else
#define WB_DEBUG(...)
#define WB_DEBUG_DUMP(...)
#endif

#endif /* WB_DEBUG_H_ */
