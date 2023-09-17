/*
 * wb_runmode.c
 *
 *  Created on: 1 Sep 2023
 *      Author: windbird-dev
 */

#include <td_flash.h>

#include "wb_runmode.h"
#include "wb_debug.h"

#define MODE_DEFAULT MODE_SIGFOX_10M

static WB_RUNMODE_mode_t currentMode;

void WB_RUNMODE_Init() {
	if (!TD_FLASH_DeclareVariable((WB_RUNMODE_mode_t *) &currentMode, sizeof (currentMode), 0)) {
		WB_DEBUG("currentMode not found in Flash, set MODE_DEFAULT\r\n");
		currentMode = MODE_DEFAULT;
	}
	WB_DEBUG("current runmode = %d\n", currentMode);
}

void WB_RUNMODE_Set(WB_RUNMODE_mode_t newMode) {
	currentMode = newMode;
	TD_FLASH_WriteVariables();
}

WB_RUNMODE_mode_t WB_RUNMODE_Get() {
	return currentMode;
}
