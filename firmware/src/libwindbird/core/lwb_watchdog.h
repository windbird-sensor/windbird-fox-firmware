/*
 * lwb_watchdog.h
 *
 *  Created on: 16 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_WATCHDOG_H_
#define LWB_WATCHDOG_H_

#include "../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X
	#include "../platforms/td120x/lwb_td120x_watchdog.h"
#else
	#error Not implemented for this platform
#endif

#endif /* LWB_WATCHDOG_H_ */
