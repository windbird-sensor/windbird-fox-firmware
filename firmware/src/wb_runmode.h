/*
 * wb_runmode.h
 *
 *  Created on: 1 Sep 2023
 *      Author: windbird-dev
 */

#ifndef WB_RUNMODE_H_
#define WB_RUNMODE_H_

typedef enum {
	// --- switchable runmodes on button press ---
	MODE_NETWORK_10M,
	MODE_NETWORK_5M
	// --- non-switchable runmodes ---
	// ex: calibration, test...
} WB_RUNMODE_mode_t;

#define WB_RUNMODE_MAX_SWITCHABLE 2

void WB_RUNMODE_Init();
void WB_RUNMODE_Set(WB_RUNMODE_mode_t newMode);
WB_RUNMODE_mode_t WB_RUNMODE_Get();

#endif /* WB_RUNMODE_H_ */
