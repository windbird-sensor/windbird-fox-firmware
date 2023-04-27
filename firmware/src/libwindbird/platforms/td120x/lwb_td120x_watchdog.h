/*
 * lwb_td120x_watchdog.h
 *
 *  Created on: 16 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_TD120X_WATCHDOG_H_
#define LWB_TD120X_WATCHDOG_H_

#include "../../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X

#include <td_watchdog.h>

#define WATCHDOG_TIMEOUT 128

#define LWB_WATCHDOG_Init() TD_WATCHDOG_Init(WATCHDOG_TIMEOUT); TD_WATCHDOG_Enable(true, false)
#define LWB_WATCHDOG_Feed() TD_WATCHDOG_Feed()

#endif /* LWB_PLATFORM_TD120X */

#endif /* LWB_TD120X_WATCHDOG_H_ */
