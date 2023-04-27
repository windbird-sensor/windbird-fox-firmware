/*
 * lwb_flash.h
 *
 *  Created on: 16 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_SETTINGS_H_
#define LWB_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>


bool LWB_SETTINGS_Set(uint8_t index, uint32_t val);
bool LWB_SETTINGS_SetFloat(uint8_t index, float val);

bool LWB_SETTINGS_IsNAN(float val);

bool LWB_SETTINGS_Clear();

bool LWB_SETTINGS_Save();

uint32_t LWB_SETTINGS_Read(uint8_t index);
float LWB_SETTINGS_ReadFloat(uint8_t index);

#ifdef LWB_PLATFORM_TD120X
#else
	#error Not implemented for this platform
#endif

#endif /* LWB_SETTINGS_H_ */
