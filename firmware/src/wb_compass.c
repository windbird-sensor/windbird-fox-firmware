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
#include <stdlib.h>
#include <td_rtc.h>
#include <td_flash.h>
#include "wb_debug.h"
#include "wb_i2c.h"
#include "wb_compass.h"
#include "wb_sigfox.h"

// I2C chip address
#define MC6470_ADDRESS 0x0C

// Define all register name and addresses
#define MC6470_SELF_TEST_REG			0x0C
#define MC6470_MORE_INFO_VERSION_REG		0x0D
#define MC6470_MORE_INFO_REG			0x0E
#define MC6470_WHO_I_AM_REG			0x0F
#define MC6470_OUTPUT_X_LSB_REG			0x10
#define MC6470_OUTPUT_X_MSB_REG			0x11
#define MC6470_OUTPUT_Y_LSB_REG			0x12
#define MC6470_OUTPUT_Y_MSB_REG			0x13
#define MC6470_OUTPUT_Z_LSB_REG			0x14
#define MC6470_OUTPUT_Z_MSB_REG			0x15
#define MC6470_STATUS_REG			0x18
#define MC6470_CONTROL_1_REG			0x1B
#define MC6470_CONTROL_2_REG			0x1C
#define MC6470_CONTROL_3_REG			0x1D
#define MC6470_CONTROL_4_REG			0x1E
#define MC6470_OFFSET_X_LSB_REG			0x20
#define MC6470_OFFSET_X_MSB_REG			0x21
#define MC6470_OFFSET_Y_LSB_REG			0x22
#define MC6470_OFFSET_Y_MSB_REG			0x23
#define MC6470_OFFSET_Z_LSB_REG			0x24
#define MC6470_OFFSET_Z_MSB_REG			0x25
#define MC6470_ITHR_L_REG			0x26
#define MC6470_ITHR_H_REG			0x27
#define MC6470_TEMPERATURE_REG			0x31

// Define Register bit mask
#define MC6470_STATUS_DRDY_MASK			0x40
#define MC6470_STATUS_DOR_MASK			0x20
#define MC6470_STATUS_FFU_MASK			0x40
#define MC6470_STATUS_TRDY_MASK			0x02
#define MC6470_STATUS_ORDY_MASK			0x01

#define MC6470_CONTROL_1_PC_MASK		0x80
#define MC6470_CONTROL_1_ODR_MASK		0x18
#define MC6470_CONTROL_1_FS_MASK		0x02

#define MC6470_CONTROL_2_AVG_MASK		0x80
#define MC6470_CONTROL_2_FCO_MASK		0x40
#define MC6470_CONTROL_2_AOR_MASK		0x20
#define MC6470_CONTROL_2_FF_MASK		0x10
#define MC6470_CONTROL_2_DEN_MASK		0x08
#define MC6470_CONTROL_2_DRP_MASK		0x04
#define MC6470_CONTROL_2_DTS_MASK		0x02
#define MC6470_CONTROL_2_DOS_MASK		0x01

#define MC6470_CONTROL_3_SRST_MASK		0x80
#define MC6470_CONTROL_3_FORCE_MASK		0x40
#define MC6470_CONTROL_3_STC_MASK		0x10
#define MC6470_CONTROL_3_TCS_MASK		0x02
#define MC6470_CONTROL_3_OCL_MASK		0x01

#define MC6470_CONTROL_4_MMD_MASK		0xC0
#define MC6470_CONTROL_4_RS_MASK		0x10
#define MC6470_CONTROL_4_AS_MASK		0x08


#define MC6470_SELF_TEST_RESET	 		0x55
#define MC6470_SELF_TEST_ON_GOING		0xAA
#define MC6470_SELF_TEST_PASS			0x55

#define MC6470_MORE_INFO_VERSION_VALUE		0x11
#define MC6470_MORE_INFO_VALUE			0x15
#define MC6470_WHO_I_AM_VALUE			0x49

#define READBYTES(...) WB_I2C_ReadBytes(MC6470_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)
#define READBYTE(...) WB_I2C_ReadByte(MC6470_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)
#define WRITEBYTE(...) WB_I2C_WriteByte(MC6470_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)

static float xOffset, yOffset, zOffset;
static float xScale, yScale, zScale;

static bool SetPower(bool powerOn) {
	uint8_t ctrl1;
	if(!READBYTE(MC6470_CONTROL_1_REG, &ctrl1)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't read ctrl1\n");
		return false;
	}
	ctrl1 |= powerOn ? MC6470_CONTROL_1_PC_MASK : 0;
	// Write it back
	if(!WRITEBYTE(MC6470_CONTROL_1_REG, ctrl1)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't write to ctrl1\n");
		return false;
	}
	return true;
}

static bool Config() {
	uint8_t control1Reg;

	// Configure force state CNTL1:FS = 1
	if(!READBYTE(MC6470_CONTROL_1_REG, &control1Reg)) {
		WB_DEBUG("!!! I2C ERROR !!! can't read MC6470_CONTROL_1_REG\n");
		return false;
	}

	// set FS bit
	control1Reg |= MC6470_CONTROL_1_FS_MASK;

	if (!WRITEBYTE(MC6470_CONTROL_1_REG, control1Reg)) {
		WB_DEBUG("!!! I2C ERROR !!! can't write MC6470_CONTROL_1_REG\n");
		return false;
	}

	return true;
}

static bool ConnectionTest(void) {
	uint8_t buffer[3];
	bool result = true;

	// Read 3 consecutive registers [ More Info Version; More Info; Who I Am]
	if (READBYTES(MC6470_MORE_INFO_VERSION_REG, 3, buffer) != 3) {
		WB_DEBUG("!!! I2C ERROR !!! can't read MC6470_MORE_INFO_VERSION_REG registers\n");
		return false;
	}

	if (buffer[0] != MC6470_MORE_INFO_VERSION_VALUE) {
		WB_DEBUG("!!! I2C ERROR !!! MC6470_MORE_INFO_VERSION_VALUE does not match\n");
		result = false;
	}
	if (buffer[1] != MC6470_MORE_INFO_VALUE) {
		WB_DEBUG("!!! I2C ERROR !!! MC6470_MORE_INFO_VALUE does not match\n");
		result = false;
	}
	if (buffer[2] != MC6470_WHO_I_AM_VALUE) {
		WB_DEBUG("!!! I2C ERROR !!! MC6470_WHO_I_AM_VALUE does not match\n");
		result = false;
	}

	if (!result) WB_DEBUG("!!! I2C ERROR !!! WB_COMPASS_ConnectionTest\n");

	return result;
}

static bool SelfTest() {
	if (!SetPower(true)) return false;

	bool result = false;
	do { // hack to mimic try/catch. Throw is replaced by break.

		// Read CTRL3 and set Bit STC to 1
		uint8_t ctrl3;
		if(!READBYTE(MC6470_CONTROL_3_REG, &ctrl3)) {
			WB_DEBUG("!!! I2C ERROR !!! Can't read ctrl3\n");
			break;
		}

		ctrl3 |= MC6470_CONTROL_3_STC_MASK; // CTRL3.STC = 1

		if(!WRITEBYTE(MC6470_CONTROL_3_REG, ctrl3)) {
			WB_DEBUG("!!! I2C ERROR !!! Can't write to ctrl3\n");
			break;
		}

		// Read once to get the on going self test value
		uint8_t onGoingSelfTestValue;

		if(!READBYTE(MC6470_SELF_TEST_REG, &onGoingSelfTestValue)) {
			WB_DEBUG("!!! I2C ERROR !!! Can't read MC6470_SELF_TEST_REG\n");
			break;
		}

		if(onGoingSelfTestValue != MC6470_SELF_TEST_ON_GOING) {
			WB_DEBUG("ERROR: onGoingSelfTestValue != MC6470_SELF_TEST_ON_GOING [%x]\n", onGoingSelfTestValue);
			break;
		}

		// Read again to get the PASS value
		uint8_t finalSelfTestValue;
		if(!READBYTE(MC6470_SELF_TEST_REG, &finalSelfTestValue)) {
			WB_DEBUG("!!! I2C ERROR !!! Can't read MC6470_SELF_TEST_REG\n");
			break;
		}

		if(finalSelfTestValue != MC6470_SELF_TEST_PASS) {
			WB_DEBUG("ERROR: finalSelfTestValue != MC6470_SELF_TEST_PASS [%x]\n", finalSelfTestValue);
			break;
		}

		result = true;

	} while (0); // will run once.

	SetPower(false);
	return result;
}

bool WB_COMPASS_GetRaw(int *x, int *y, int *z) {
	if (!SetPower(true)) return false;

	bool result = false;
	do {
		// trigger a single measurement
		// CTRL3.Force =1
		uint8_t ctrl3Reg;

		if (!READBYTE(MC6470_CONTROL_3_REG, &ctrl3Reg)) {
			WB_DEBUG("can't read MC6470_CONTROL_3_REG\n");
			break;
		}

		ctrl3Reg |= MC6470_CONTROL_3_FORCE_MASK;

		if (!WRITEBYTE(MC6470_CONTROL_3_REG, ctrl3Reg)) {
			WB_DEBUG("can't write MC6470_CONTROL_3_REG\n");
			break;
		}

		uint8_t statusReg;
		int retry;
		for (retry=5; retry<0; retry--) {
			TD_RTC_Delay(TMS(5));
			if (!READBYTE(MC6470_STATUS_REG, &statusReg)) {
				WB_DEBUG("can't read MC6470_STATUS_REG\n");
				continue; // retry;
			}
			 // measurement is successful when REG3.FORCE is back to 0
			if ((statusReg & MC6470_STATUS_DRDY_MASK) != 0) break;

			WB_DEBUG("compass data is not ready yet\n");
		}
		if (retry == 0) {
			WB_DEBUG("compass measurement timeout\n");
			break;
		}

		// Read measurements

		int16_t measurementsBuffer[6];

		if (READBYTES(MC6470_OUTPUT_X_LSB_REG, 6, measurementsBuffer) != 6) {
			WB_DEBUG("Can't read results");
			break;
		}

		*x = (int)(int16_t)measurementsBuffer[0];
		*y = (int)(int16_t)measurementsBuffer[1];
		*z = (int)(int16_t)measurementsBuffer[2];
		// casting two uint8 (LSB and MSB) into one int16
		// Results are always little endian

		result = true;
	} while(0); // will run once.

	SetPower(false);
	return result;
}


bool WB_COMPASS_Test () {

	// One shot measure is triggered using the force state.
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

	int x, y, z;
	int xMin, yMin, zMin;
	int xMax, yMax, zMax;

	xMin=yMin=zMin=9999;
	xMax=yMax=zMax=-9999;

	uint32_t timeout = 1000;
	while (timeout) {
		timeout--;

		if(!WB_COMPASS_GetRaw(&x, &y, &z)) continue;

		if (x < xMin) xMin = x;
		if (x > xMax) xMax = x;

		if (y < yMin) yMin = y;
		if (y > yMax) yMax = y;

		if (z < zMin) zMin = z;
		if (z > zMax) zMax = z;
		WB_DEBUG("calibration point\t %d\t%d\t%d\t \t %d\t%d\t%d\t \t %d\t%d\t%d\t \t %d\n",
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

	xOffset = (xMin+xMax)/2;
	yOffset = (yMin+yMax)/2;
	zOffset = (zMin+zMax)/2;

	xScale = xMax - xOffset;
	yScale = yMax - yOffset;
	zScale = zMax - zOffset;

	WB_DEBUG("Calibration done : %d\t%d\t%d\t%d\t%d\t%d\n",
			(int)(float)(xOffset),
			(int)(float)(xScale),
			(int)(float)(yOffset),
			(int)(float)(yScale),
			(int)(float)(zOffset),
			(int)(float)(zScale));

}

void WB_COMPASS_SaveCalibration() {
	TD_FLASH_WriteVariables();
	//WB_SIGFOX_CalibrationMessage(yOffset, yScale, zOffset, zScale);
}

void WB_COMPASS_ClearCalibration() {
	TD_FLASH_DeleteVariables();
}

void WB_COMPASS_Init() {

	WB_DEBUG("Compass init\n");

	bool needSave = false;

	/*if (!Config()) {
		WB_DEBUG("FAIL - Compass Config error\n");
	}

	if (!ConnectionTest()) {
		WB_DEBUG("FAIL - Compass ConnectionTest error\n");
	}*/

	if (!SelfTest()) {
		WB_DEBUG("FAIL - Compass SelfTest error\n");
	}

	// TD_FLASH_DeleteVariables();

	if (!TD_FLASH_DeclareVariable((uint8_t *) &xOffset, sizeof (float), 0)) {
		WB_DEBUG("No xOffset in Flash. Using default\n");
		xOffset = 0;
		needSave = true;
	} else {
		WB_DEBUG("Using xOffset from Flash : %d\n", (int)(float)(xOffset));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &yOffset, sizeof (float), 0)) {
		WB_DEBUG("No yOffset in Flash. Using default\n");
		yOffset = 0;
		needSave = true;
	} else {
		WB_DEBUG("Using yOffset from Flash : %d\n", (int)(float)(yOffset));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &zOffset, sizeof (float), 0)) {
		WB_DEBUG("No zOffset in Flash. Using default\n");
		zOffset = 0;
		needSave = true;
	} else {
		WB_DEBUG("Using zOffset from Flash : %d\n", (int)(float)(zOffset));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &xScale, sizeof (float), 0)) {
		WB_DEBUG("No xScale in Flash. Using default\n");
		xScale = 1;
		needSave = true;
	} else {
		WB_DEBUG("Using xScale from Flash : %d\n", (int)(float)(xScale));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &yScale, sizeof (float), 0)) {
		WB_DEBUG("No yScale in Flash. Using default\n");
		yScale = 1;
		needSave = true;
	} else {
		WB_DEBUG("Using yScale from Flash : %d\n", (int)(float)(yScale));
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &zScale, sizeof (float), 0)) {
		WB_DEBUG("No zScale in Flash. Using default\n");
		zScale = 1;
		needSave = true;
	} else {
		WB_DEBUG("Using zScale from Flash : %d\n", (int)(float)(zScale));
	}

	if (needSave) TD_FLASH_WriteVariables();

}

float WB_COMPASS_GetHeading() {

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

	int rawX, rawY, rawZ;
	if(!WB_COMPASS_GetRaw(&rawX, &rawY, &rawZ)) {
		WB_DEBUG("ERROR with compass reading\n");
		return 0;
	}

	float x = (rawX - xOffset) / yScale;
	// float y = (rawY - yOffset) / yScale;
	float z = (rawZ - zOffset) / zScale;

	float heading = atan2(z, x);
	if(heading < 0) heading += 2. * M_PI;

	return heading;

}

void WB_COMPASS_TestCalibration() {
	while (true) {
		WB_DEBUG("%d\n", (int)(float)(WB_COMPASS_GetHeading()/M_PI*180.));
	}
}
