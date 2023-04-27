/**************************************************************************
 * @file WB_compass.c
 * @brief Compass Sensor API for WINDBIRD's firmware
 * @author Nicolas BALDECK
 * @author ludobaill
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

#include <math.h>

#include "../../wb_config.h"

#include "../core/lwb_serial.h"
#include "../core/lwb_settings.h"
#include "lwb_compass.h"

static float xOffset, yOffset, zOffset;
static float xScale, yScale, zScale;

bool LWB_COMPASS_Calibrate() {

	int x, y, z;
	int xMin, yMin, zMin;
	int xMax, yMax, zMax;

	xMin=yMin=zMin=999999;
	xMax=yMax=zMax=-999999;

	uint32_t timeout = 1000;
	while (timeout) {
		timeout--;

		if(!LWB_COMPASS_GetRaw(&x, &y, &z)) {
			LWB_SERIAL_Println("Failed to read from compass");
			return false;
		}

		if (x < xMin) xMin = x;
		if (x > xMax) xMax = x;

		if (y < yMin) yMin = y;
		if (y > yMax) yMax = y;

		if (z < zMin) zMin = z;
		if (z > zMax) zMax = z;
		LWB_SERIAL_Printfln("%d\t%d\t%d\t\t%d\t%d\t%d\t\t%d\t%d\t%d\t\t %d",
				xMin,
				x,
				xMax,
				yMin,
				y,
				yMax,
				zMin,
				z,
				zMax,
				timeout);

	}

	xOffset = (xMin+xMax)/2.;
	yOffset = (yMin+yMax)/2.;
	zOffset = (zMin+zMax)/2.;

	xScale = xMax - xOffset;
	yScale = yMax - yOffset;
	zScale = zMax - zOffset;

	LWB_SERIAL_Printfln("Calibration done : %d\t%d\t%d\t%d\t%d\t%d",
			(int)(float)(xOffset),
			(int)(float)(xScale),
			(int)(float)(yOffset),
			(int)(float)(yScale),
			(int)(float)(zOffset),
			(int)(float)(zScale));

	LWB_SETTINGS_SetFloat(SETTING_COMPASS_XSCALE, xScale);
	LWB_SETTINGS_SetFloat(SETTING_COMPASS_YSCALE, yScale);
	LWB_SETTINGS_SetFloat(SETTING_COMPASS_ZSCALE, zScale);
	LWB_SETTINGS_SetFloat(SETTING_COMPASS_XOFFSET, xOffset);
	LWB_SETTINGS_SetFloat(SETTING_COMPASS_YOFFSET, yOffset);
	LWB_SETTINGS_SetFloat(SETTING_COMPASS_ZOFFSET, zOffset);

	if (!LWB_SETTINGS_Save()) {
		LWB_SERIAL_Println("Failed to save calibration");
	}

	return true;
}

#define LOAD_SETTING(var, index, def) 	var = LWB_SETTINGS_ReadFloat(index); \
	if (LWB_SETTINGS_IsNAN(var)) { \
		LWB_SERIAL_Printfln("No " #var " in Flash. Using default=%d (x100)", (int)(float)(def*100)); \
		var = def; \
	} else { \
		/* LWB_SERIAL_Printfln("Using " #var " from Flash : %d (x100)", (int)(float)(var*100)); */ \
	}

void LWB_COMPASS_Init() {

	// LWB_SERIAL_Debugln("Compass init");

	/*if (!LWB_COMPASS_Test()) {
		LWB_SERIAL_Println("FAIL - Compass test error");
	}*/

	LOAD_SETTING(xScale, SETTING_COMPASS_XSCALE, COMPASS_XSCALE_DEFAULT);
	LOAD_SETTING(yScale, SETTING_COMPASS_YSCALE, COMPASS_YSCALE_DEFAULT);
	LOAD_SETTING(zScale, SETTING_COMPASS_ZSCALE, COMPASS_ZSCALE_DEFAULT);

	LOAD_SETTING(xOffset, SETTING_COMPASS_XOFFSET, COMPASS_XOFFSET_DEFAULT);
	LOAD_SETTING(yOffset, SETTING_COMPASS_YOFFSET, COMPASS_YOFFSET_DEFAULT);
	LOAD_SETTING(zOffset, SETTING_COMPASS_ZOFFSET, COMPASS_ZOFFSET_DEFAULT);
}

float LWB_COMPASS_GetHeading() {
	int rawX, rawY, rawZ;
	if(!LWB_COMPASS_GetRaw(&rawX, &rawY, &rawZ)) {
		LWB_SERIAL_Println("ERROR with compass reading");
		return 0;
	}
	return LWB_COMPASS_GetHeadingFromRaw(rawX, rawY, rawZ);
}

float LWB_COMPASS_GetHeadingFromRaw(int rawX, int rawY, int rawZ) {

	/*    [Right View]        *     [Front view]
	 *                        *
	 *                  ┬     *         |      ┬
	 *    ┌      /¯|    │     *         ┌      │
	 *    ├═╦═══┤  |   [Y]    *         ©     [Y]
	 *    ┘ ║    \_|    │     *         ┘      │
	 *      ║           │     *         ║      │
	 *                  V     *                V
	 *    <───[X]───┤         *     ├──[Z]──>
	 */

	float x = (rawX - xOffset) / xScale;
	// float y = (rawY - yOffset) / yScale;
	float z = (rawZ - zOffset) / zScale;

	float heading = atan2(z, x);
	if(heading < 0) heading += 2. * M_PI;

	return heading;
}
