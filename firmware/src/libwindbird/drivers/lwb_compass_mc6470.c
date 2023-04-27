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

#include "lwb_compass.h"

#include "../core/lwb_serial.h"
#include "../core/lwb_i2c.h"

#include <td_rtc.h>

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

#define READBYTES(...) LWB_I2C_ReadBytes(MC6470_ADDRESS, __VA_ARGS__, LWB_I2C_DEFAULT_TIMEOUT)
#define READBYTE(...) LWB_I2C_ReadByte(MC6470_ADDRESS, __VA_ARGS__, LWB_I2C_DEFAULT_TIMEOUT)
#define WRITEBYTE(...) LWB_I2C_WriteByte(MC6470_ADDRESS, __VA_ARGS__, LWB_I2C_DEFAULT_TIMEOUT)

static bool SetPower(bool powerOn) {
	uint8_t ctrl1;
	if(!READBYTE(MC6470_CONTROL_1_REG, &ctrl1)) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! Can't read ctrl1");
		return false;
	}
	ctrl1 |= powerOn ? MC6470_CONTROL_1_PC_MASK : 0;
	// Write it back
	if(!WRITEBYTE(MC6470_CONTROL_1_REG, ctrl1)) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! Can't write to ctrl1");
		return false;
	}
	return true;
}

static bool Config() {
	uint8_t control1Reg;

	// Configure force state CNTL1:FS = 1
	if(!READBYTE(MC6470_CONTROL_1_REG, &control1Reg)) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! can't read MC6470_CONTROL_1_REG");
		return false;
	}

	// set FS bit
	control1Reg |= MC6470_CONTROL_1_FS_MASK;

	if (!WRITEBYTE(MC6470_CONTROL_1_REG, control1Reg)) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! can't write MC6470_CONTROL_1_REG");
		return false;
	}

	return true;
}

static bool ConnectionTest(void) {
	uint8_t buffer[3];
	bool result = true;

	// Read 3 consecutive registers [ More Info Version; More Info; Who I Am]
	if (READBYTES(MC6470_MORE_INFO_VERSION_REG, 3, buffer) != 3) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! can't read MC6470_MORE_INFO_VERSION_REG registers");
		return false;
	}

	if (buffer[0] != MC6470_MORE_INFO_VERSION_VALUE) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! MC6470_MORE_INFO_VERSION_VALUE does not match");
		result = false;
	}
	if (buffer[1] != MC6470_MORE_INFO_VALUE) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! MC6470_MORE_INFO_VALUE does not match");
		result = false;
	}
	if (buffer[2] != MC6470_WHO_I_AM_VALUE) {
		LWB_SERIAL_Println("!!! I2C ERROR !!! MC6470_WHO_I_AM_VALUE does not match");
		result = false;
	}

	if (!result) LWB_SERIAL_Println("!!! I2C ERROR !!! WB_COMPASS_ConnectionTest");

	return result;
}

static bool SelfTest() {
	if (!SetPower(true)) return false;

	bool result = false;
	do { // hack to mimic try/catch. Throw is replaced by break.

		// Read CTRL3 and set Bit STC to 1
		uint8_t ctrl3;
		if(!READBYTE(MC6470_CONTROL_3_REG, &ctrl3)) {
			LWB_SERIAL_Println("!!! I2C ERROR !!! Can't read ctrl3");
			break;
		}

		ctrl3 |= MC6470_CONTROL_3_STC_MASK; // CTRL3.STC = 1

		if(!WRITEBYTE(MC6470_CONTROL_3_REG, ctrl3)) {
			LWB_SERIAL_Println("!!! I2C ERROR !!! Can't write to ctrl3");
			break;
		}

		// Read once to get the on going self test value
		uint8_t onGoingSelfTestValue;

		if(!READBYTE(MC6470_SELF_TEST_REG, &onGoingSelfTestValue)) {
			LWB_SERIAL_Println("!!! I2C ERROR !!! Can't read MC6470_SELF_TEST_REG");
			break;
		}

		if(onGoingSelfTestValue != MC6470_SELF_TEST_ON_GOING) {
			LWB_SERIAL_Printfln("ERROR: onGoingSelfTestValue != MC6470_SELF_TEST_ON_GOING [%x]", onGoingSelfTestValue);
			break;
		}

		// Read again to get the PASS value
		uint8_t finalSelfTestValue;
		if(!READBYTE(MC6470_SELF_TEST_REG, &finalSelfTestValue)) {
			LWB_SERIAL_Println("!!! I2C ERROR !!! Can't read MC6470_SELF_TEST_REG");
			break;
		}

		if(finalSelfTestValue != MC6470_SELF_TEST_PASS) {
			LWB_SERIAL_Printfln("ERROR: finalSelfTestValue != MC6470_SELF_TEST_PASS [%x]", finalSelfTestValue);
			break;
		}

		result = true;

	} while (0); // will run once.

	SetPower(false);
	return result;
}

bool LWB_COMPASS_GetRaw(int *x, int *y, int *z) {
	*x = 0;
	*y = 0;
	*z = 0;

	if (!SetPower(true)) return false;

	bool result = false;
	do {
		// trigger a single measurement
		// CTRL3.Force =1
		uint8_t ctrl3Reg;

		if (!READBYTE(MC6470_CONTROL_3_REG, &ctrl3Reg)) {
			LWB_SERIAL_Println("can't read MC6470_CONTROL_3_REG");
			break;
		}

		ctrl3Reg |= MC6470_CONTROL_3_FORCE_MASK;

		if (!WRITEBYTE(MC6470_CONTROL_3_REG, ctrl3Reg)) {
			LWB_SERIAL_Println("can't write MC6470_CONTROL_3_REG");
			break;
		}

		uint8_t statusReg;
		int retry;
		for (retry=5; retry<0; retry--) {
			TD_RTC_Delay(TMS(5));
			if (!READBYTE(MC6470_STATUS_REG, &statusReg)) {
				LWB_SERIAL_Println("can't read MC6470_STATUS_REG");
				continue; // retry;
			}
			 // measurement is successful when REG3.FORCE is back to 0
			if ((statusReg & MC6470_STATUS_DRDY_MASK) != 0) break;

			LWB_SERIAL_Println("compass data is not ready yet");
		}
		if (retry == 0) {
			LWB_SERIAL_Println("compass measurement timeout");
			break;
		}

		// Read measurements

		int16_t measurementsBuffer[6];

		if (READBYTES(MC6470_OUTPUT_X_LSB_REG, 6, measurementsBuffer) != 6) {
			LWB_SERIAL_Println("Can't read results");
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


bool LWB_COMPASS_Test () {

	// One shot measure is triggered using the force state.
	bool result = true;

	if (!ConnectionTest()) {
		LWB_SERIAL_Println("!!! FAIL !!! Connection Test");
		result = false;
	}

	if (!SelfTest()) {
		LWB_SERIAL_Println("!!! FAIL !!! Self-Test");
		result = false;
	}

	return result;

}
