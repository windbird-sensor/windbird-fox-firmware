/**************************************************************************
 * @file debug.h
 * @brief Debug functions for PIOUPIOU's firmware
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
#ifndef PP_DEBUG_H_
#define PP_DEBUG_H_

#ifdef DEBUG_PIOUPIOU
#include <td_printf.h>
#define PP_DEBUG(...) tfp_printf(__VA_ARGS__)
#define PP_DEBUG_DUMP(...) tfp_dump(__VA_ARGS__)
#else
#define PP_DEBUG(...)
#define PP_DEBUG_DUMP(...)
#endif

#endif /* PP_DEBUG_H_ */
