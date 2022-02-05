/**************************************************************************
 * @file WB_compass.c
 * @brief Compass Sensor API for WINDBIRD's firmware
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

#include <math.h>
#include <stdlib.h>
#include <td_rtc.h>
#include <td_flash.h>
#include "wb_debug.h"
#include "wb_i2c.h"
#include "wb_compass.h"
#include "wb_sigfox.h"

#define HMC5883L_ADDRESS 0x1E

#define HMC5883L_CRA 0x00
#define HMC5883L_CRA_AVERAGE_1_SAMPLE 0b00000000
#define HMC5883L_CRA_MODE_NORMAL 0b00000000
#define HMC5883L_CRA_MODE_POSITIVE_BIAS 0b00000001
#define HMC5883L_CRA_MODE_NEGATIVE_BIAS 0b00000010
#define HMC5883L_CRA_RATE_0P75HZ 0b00000000

#define HMC5883L_CRB 0x01
#define HMC5883L_CRB_GAIN_DEFAULT 5

#define HMC5883L_MR 0x02
#define HMC5883L_MR_ONESHOT 0b00000001

#define HMC5883L_ID_REG_A 0x0A

#define HMC5883L_DATA_OUT_REG 0x03

#define HMC5883L_SELFTEST_LOW_LIMIT 243
#define HMC5883L_SELFTEST_HIGH_LIMIT 575

static uint8_t i2cBuffer[6];
static float yOffset, zOffset, yScale, zScale;

static bool Config(uint8_t mode, uint8_t gain) {

	i2cBuffer[0] = gain << 5;
	if(!WB_I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_CRB, i2cBuffer[0], WB_I2C_DEFAULT_TIMEOUT)) return false;
	int16_t dummy;
	if(!WB_COMPASS_GetRaw(&dummy, &dummy, &dummy)) return false; //set the new gain

	i2cBuffer[0] = HMC5883L_CRA_AVERAGE_1_SAMPLE | mode | HMC5883L_CRA_RATE_0P75HZ;
	if(!WB_I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_CRA, i2cBuffer[0], WB_I2C_DEFAULT_TIMEOUT)) return false;


	return true;
}

static bool ConnectionTest() {
	if (WB_I2C_ReadBytes(HMC5883L_ADDRESS, HMC5883L_ID_REG_A, 3, i2cBuffer, WB_I2C_DEFAULT_TIMEOUT) == 3) {
		return (i2cBuffer[0] == 'H' && i2cBuffer[1] == '4' && i2cBuffer[2] == '3');
	} else {
		WB_DEBUG("!!! I2C ERROR !!! WB_COMPASS_ConnectionTest\n");
		return false;
	}
}

static bool SelfTest() {

	int16_t x, y, z;

	bool result = true;

	// POSITIVE BIAS

	if (!Config(HMC5883L_CRA_MODE_POSITIVE_BIAS, HMC5883L_CRB_GAIN_DEFAULT)) {
		WB_DEBUG("FAIL - Config error\n");
		result = false;
	}

	TD_RTC_Delay(TMS(250));


	if (!WB_COMPASS_GetRaw(&x, &y, &z)) {
		WB_DEBUG("FAIL - acquire error\n");
	}

	if (x < HMC5883L_SELFTEST_LOW_LIMIT || x > HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - X axis is out of bounds (%d)\n", x);
		result = false;
	}
	if (y < HMC5883L_SELFTEST_LOW_LIMIT || y > HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - Y axis is out of bounds (%d)\n", y);
		result = false;
	}
	if (z < HMC5883L_SELFTEST_LOW_LIMIT || z > HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - Z axis is out of bounds (%d)\n", z);
		result = false;
	}


	// NEGATIVE BIAS

	if (!Config(HMC5883L_CRA_MODE_NEGATIVE_BIAS, HMC5883L_CRB_GAIN_DEFAULT)) {
		WB_DEBUG("FAIL - Config error\n");
		result = false;
	}

	TD_RTC_Delay(TMS(250));


	if (!WB_COMPASS_GetRaw(&x, &y, &z)) {
		WB_DEBUG("FAIL - acquire error\n");
	}

	if (x > -HMC5883L_SELFTEST_LOW_LIMIT || x < -HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - X neg axis is out of bounds (%d)\n", x);
		result = false;
	}
	if (y > -HMC5883L_SELFTEST_LOW_LIMIT || y < -HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - Y neg axis is out of bounds (%d)\n", y);
		result = false;
	}
	if (z > -HMC5883L_SELFTEST_LOW_LIMIT || z < -HMC5883L_SELFTEST_HIGH_LIMIT) {
		WB_DEBUG("FAIL - Z neg axis is out of bounds (%d)\n", z);
		result = false;
	}

	// RESTORE CONFIG

	if (!Config(HMC5883L_CRA_MODE_NORMAL, HMC5883L_CRB_GAIN_DEFAULT)) {
		WB_DEBUG("FAIL - Config error\n");
		result = false;
	}

	return result;
}


bool WB_COMPASS_GetRaw(int16_t *x, int16_t *y, int16_t *z) {
	if(!WB_I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_MR, HMC5883L_MR_ONESHOT, WB_I2C_DEFAULT_TIMEOUT)) return false;
	TD_RTC_Delay(TMS(6));
	if (WB_I2C_ReadBytes(HMC5883L_ADDRESS, HMC5883L_DATA_OUT_REG, 6, i2cBuffer, WB_I2C_DEFAULT_TIMEOUT) != 6) return false;

	*x = (i2cBuffer[0] << 8) + i2cBuffer[1];
	*y = (i2cBuffer[2] << 8) + i2cBuffer[3];
	*z = (i2cBuffer[4] << 8) + i2cBuffer[5];

	return true;
}


bool WB_COMPASS_Test () {

	bool result = true;

	if (!ConnectionTest()) {
		WB_DEBUG("!!! FAIL !!! Connection Test\n");
		result = false;
	}

	if (!SelfTest()) {
		WB_DEBUG("!!! FAIL !!! Self-Test\n");
		result = false;
	}

	return result;

}

void WB_COMPASS_Calibrate() {

	int16_t x, y, z, yMin, zMin, yMax, zMax;

	yMin=zMin=9999;
	yMax=zMax=-9999;

	uint32_t timeout = 1000;
	while (timeout) {
		timeout--;

		if(!WB_COMPASS_GetRaw(&x, &y, &z)) continue;

		if (y < yMin) yMin = y;
		if (y > yMax) yMax = y;

		if (z < zMin) zMin = z;
		if (z > zMax) zMax = z;

		WB_DEBUG("calibration point\t%d\t%d\t%d\t%d\t\t%d\t%d\t%d\t%d\n",
				x,
				yMin,
				y,
				yMax,
				zMin,
				z,
				zMax,
				timeout);

	}

	yOffset = (yMin+yMax)/2;
	zOffset = (zMin+zMax)/2;

	yScale = 1/(yMax - yOffset);
	zScale = 1/(zMax - zOffset);

	WB_DEBUG("Calibration done : %d\t%d\t%d\t%d\n",
			(int)(float)(yOffset),
			(int)(float)(yScale*1e6),
			(int)(float)(zOffset),
			(int)(float)(zScale*1e6));

}

void WB_COMPASS_SaveCalibration() {
	TD_FLASH_WriteVariables();
	//WB_SIGFOX_CalibrationMessage(yOffset, yScale, zOffset, zScale);
}

void WB_COMPASS_ClearCalibration() {
	TD_FLASH_DeleteVariables();
}

void WB_COMPASS_Init() {

	bool needSave = false;

	if (!Config(HMC5883L_CRA_MODE_NORMAL, HMC5883L_CRB_GAIN_DEFAULT)) {
		WB_DEBUG("FAIL - Config error\n");
	}

	//TD_FLASH_DeleteVariables();

	if (!TD_FLASH_DeclareVariable((uint8_t *) &yOffset, sizeof (float), 0)) {
		WB_DEBUG("No yOffset in Flash − Using default\n");
		yOffset = 10;
		needSave = true;
	} else {
		WB_DEBUG("Using yOffset from Flash : %d\n", (int)(float)(yOffset));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &zOffset, sizeof (float), 0)) {
		WB_DEBUG("No zOffset in Flash − Using default\n");
		zOffset = -32;
		needSave = true;
	} else {
		WB_DEBUG("Using zOffset from Flash : %d\n", (int)(float)(zOffset));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &yScale, sizeof (float), 0)) {
		WB_DEBUG("No yScale in Flash − Using default\n");
		yScale = 0.0092;
		needSave = true;
	} else {
		WB_DEBUG("Using yScale from Flash : %d\n", (int)(float)(yScale*1e6));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &zScale, sizeof (float), 0)) {
		WB_DEBUG("No zScale in Flash − Using default\n");
		zScale = 0.0085;
		needSave = true;
	} else {
		WB_DEBUG("Using zScale from Flash : %d\n", (int)(float)(zScale*1e6));
	}

	if (needSave) TD_FLASH_WriteVariables();

}

float WB_COMPASS_GetHeading() {

	int16_t rawX, rawY, rawZ;
	if(!WB_COMPASS_GetRaw(&rawX, &rawY, &rawZ)) {
		WB_DEBUG("ERROR with compass reading\n");
		return 0;
	}

	float y = (rawY - yOffset) * yScale;
	float z = (rawZ - zOffset) * zScale;

	float heading = atan2(y, -z);
	if(heading < 0) heading += 2. * M_PI;

	return heading;

}

void WB_COMPASS_TestCalibration() {
	while (true) {
		WB_DEBUG("%d\n", (int)(float)(WB_COMPASS_GetHeading()/M_PI*180.));
	}
}
