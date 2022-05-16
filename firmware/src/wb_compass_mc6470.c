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

// I2C chip address
#define MC6470_ADDRESS 0x4C

// Define all register name and addresses
#define MC6470_SELF_TEST_REG			0x0C
#define MC6470_MORE_INFO_VERSION_REG 	0x0D
#define MC6470_MORE_INFO_REG			0x0E
#define MC6470_WHO_I_AM_REG				0x0F
#define MC6470_OUTPUT_X_LSB_REG			0x10
#define MC6470_OUTPUT_X_MSB_REG			0x11
#define MC6470_OUTPUT_Y_LSB_REG			0x12
#define MC6470_OUTPUT_Y_MSB_REG			0x13
#define MC6470_OUTPUT_Z_LSB_REG			0x14
#define MC6470_OUTPUT_Z_MSB_REG			0x15
#define MC6470_STATUS_REG				0x18
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
#define MC6470_ITHR_L_REG				0x26
#define MC6470_ITHR_H_REG				0x27
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

#define MC6470_MORE_INFO_VERSION_VALUE 	0x11
#define MC6470_MORE_INFO_VALUE			0x15
#define MC6470_WHO_I_AM_VALUE			0x49


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

//static uint8_t i2cBuffer[6];
static float yOffset, zOffset, yScale, zScale;

static bool Config(uint8_t mode, uint8_t gain) {
	uint8_t control1Reg;
	// Configure force state CNTL1:FS = 1
	if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_CONTROL_1_REG, 1, &control1Reg, WB_I2C_DEFAULT_TIMEOUT) != 1)
	{
		return false; // can't read MC6470_CONTROL_1_REG
	}

	// set FS bit
	control1Reg |= MC6470_CONTROL_1_FS_MASK;

	if (WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_CONTROL_1_REG, control1Reg, WB_I2C_DEFAULT_TIMEOUT) == false)
	{
		return false; // can't write MC6470_CONTROL_1_REG
	}

	return true;
}

static bool ConnectionTest(void)
{
	uint8_t buffer[3];
	bool connectionStatus;

	// Read 3 consecutives registers [ More Info Version; More Info; Who I Am]
	if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_WHO_I_AM_REG, 3, buffer, WB_I2C_DEFAULT_TIMEOUT) == 3)
	{
		if( buffer[0]== MC6470_MORE_INFO_VERSION_VALUE && buffer[1]== MC6470_MORE_INFO_VALUE && buffer[2]== MC6470_WHO_I_AM_VALUE )
		{
			return true;
		}
	}
	else
	{
		WB_DEBUG("!!! I2C ERROR !!! WB_COMPASS_ConnectionTest\n");
		return false;
	}
}

static bool SelfTest() {
	// Init the STD register to its default VALUE
	bool writeStbStatus = WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_SELF_TEST_REG, MC6470_SELF_TEST_RESET, WB_I2C_DEFAULT_TIMEOUT);
	if( writeStbStatus == false)
	{
		return false;
	}

	// Trigger the Self Test

	// Read CTRL3 and set Bit STC to 1
	uint8_t ctrl3;
	bool ctrlStatus = WB_I2C_ReadByte(MC6470_ADDRESS, MC6470_CONTROL_3_REG, &ctrl3, WB_I2C_DEFAULT_TIMEOUT);
	if( ctrl3 == false )
	{
		return false;
	}

	ctrl3 |= MC6470_CONTROL_3_STC_MASK; // CTRL3.STC = 1

	// Write it back
	ctrlStatus = WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_CONTROL_3_REG, crtl3, WB_I2C_DEFAULT_TIMEOUT);
	if( writeStbStatus == false)
	{
		return false;
	}

	// Read once to get the on going self test value
	uint8_t onGoingSelfTestValue;

	bool onGoingStbStatus = WB_I2C_ReadByte(MC6470_ADDRESS, MC6470_SELF_TEST_REG, &onGoingSelfTestValue, WB_I2C_DEFAULT_TIMEOUT);
	if( writeStbStatus == false || onGoingSelfTestValue != MC6470_SELF_TEST_ON_GOING)
	{
		return false;
	}

	// Read again to get the PASS value
	uint8_t finalSelfTestValue;
	bool resultStbStatus = WB_I2C_ReadByte(MC6470_ADDRESS, MC6470_SELF_TEST_REG, &finalSelfTestValue, WB_I2C_DEFAULT_TIMEOUT);
	if( writeStbStatus == false || finalSelfTestValue != MC6470_SELF_TEST_PASS)
	{
		return false;
	}

	return true;
}


bool WB_COMPASS_GetRaw(int16_t *x, int16_t *y, int16_t *z) {



	//CTRL1.PC = 1 Power control ON
	uint8_t ctrl1Reg;

	if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_CONTROL_1_REG, 1, &ctrl1Reg, WB_I2C_DEFAULT_TIMEOUT) != 1)
	{
		return false; // can't read MC6470_CONTROL_1_REG
	}

	ctrl1Reg |= MC6470_CONTROL_1_PC_MASK;

	if (WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_CONTROL_1_REG, ctrl1Reg, WB_I2C_DEFAULT_TIMEOUT) == false)
	{
		return false; // can't write MC6470_CONTROL_1_REG
	}


	// CTRL3.Force =1
	uint8_t ctrl3Reg;

	if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_CONTROL_3_REG, 1, &ctrl3Reg, WB_I2C_DEFAULT_TIMEOUT) != 1)
	{
		return false; // can't read MC6470_CONTROL_3_REG
	}

	ctrl3Reg |= MC6470_CONTROL_3_FORCE_MASK;

	if (WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_CONTROL_3_REG, control1Reg, WB_I2C_DEFAULT_TIMEOUT) == false)
	{
		return false; // can't write MC6470_CONTROL_3_REG
	}

	// Wait 5Ms
	TD_RTC_Delay(TMS(5));

	// Force bit returns to 0 when measure is done
	do
	{
		if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_CONTROL_3_REG, 1, &ctrl3Reg, WB_I2C_DEFAULT_TIMEOUT) != 1)
		{
			return false; // can't read MC6470_CONTROL_3_REG
		}
	}while((ctrl3Reg & MC6470_CONTROL_3_FORCE_MASK) != 0)

	//Read results
	int16_t resultsBuffer[3];

	if (WB_I2C_ReadBytes(MC6470_ADDRESS, MC6470_OUTPUT_X_LSB_REG, 6, resultsBuffer, WB_I2C_DEFAULT_TIMEOUT) != 6)
	{
		return false; // Can't read results
	}

	// return in stand by mode
	ctrl1Reg &= ~MC6470_CONTROL_1_PC_MASK;

	if (WB_I2C_WriteByte(MC6470_ADDRESS, MC6470_CONTROL_1_REG, control1Reg, WB_I2C_DEFAULT_TIMEOUT) == false)
	{
		return false; // can't write MC6470_CONTROL_1_REG
	}

	*x = resultsBuffer[0]; // Results are always little endian
	*y = resultsBuffer[1]; // Results are always little endian
	*z = resultsBuffer[2]; // Results are always little endian

	return true;
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
