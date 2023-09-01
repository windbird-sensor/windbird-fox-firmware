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

#include <td_rtc.h>
#include "wb_debug.h"
#include "wb_i2c.h"
#include "wb_accelero.h"

// I2C chip address
#define MC6470_ACC_ADDRESS 0x4C
// OU 0x06

// Define all register name and addresses
#define MC6470_ACC_MODE_REG 0x07
#define MC6470_ACC_MODE_STANDBY_MASK 0x00
#define MC6470_ACC_MODE_WAKE_MASK 0x01

#define MC6470_ACC_OUT_REG 0x0D
// 0x0D to 0x012 for X[LSB], X[MSB], Y, Z...

#define MC6470_ACC_STATUS_REG 0x03
#define MC6470_ACC_STATUS_SAMPLE_READY_MASK 0x80

#define MC6470_ACC_OPSTAT_REG 0x07
#define MC6470_ACC_OPSTAT_STANDBY_MASK 0x00
#define MC6470_ACC_OPSTAT_WAKE_MASK 0x01

#define MC6470_ACC_SRTFR_REG 0x08
#define MC6470_ACC_SRTFR_RATE_1HZ_MASK 0x05
#define MC6470_ACC_SRTFR_RATE_05HZ_MASK 0x06
#define MC6470_ACC_SRTFR_RATE_025HZ_MASK 0x07

#define READBYTES(...) WB_I2C_ReadBytes(MC6470_ACC_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)
#define READBYTE(...) WB_I2C_ReadByte(MC6470_ACC_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)
#define WRITEBYTE(...) WB_I2C_WriteByte(MC6470_ACC_ADDRESS, __VA_ARGS__, WB_I2C_DEFAULT_TIMEOUT)

static uint8_t GetStatus() {
	uint8_t status;
	if(!READBYTE(MC6470_ACC_STATUS_REG, &status)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't read MC6470_ACC_STATUS_REG\n");
		return 0xFF;
	}
	return status;
}

static bool SampleReady() {
	uint8_t status = GetStatus();
	return status & MC6470_ACC_STATUS_SAMPLE_READY_MASK;
}

static bool SetAwake(bool wake) {
	uint8_t mode;
	if (wake) {
		mode = MC6470_ACC_MODE_WAKE_MASK;
	} else {
		mode = MC6470_ACC_MODE_STANDBY_MASK;
	}
	if(!WRITEBYTE(MC6470_ACC_MODE_REG, mode)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't write to MC6470_ACC_MODE_REG\n");
		return false;
	}
	return true;
}

static bool SetSampleRate() {
	// 0.25Hz
	// todo : handle other sample rates and other SRTFR flags
	if(!WRITEBYTE(MC6470_ACC_SRTFR_REG, 0 /*MC6470_ACC_SRTFR_RATE_025HZ_MASK*/)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't write to MC6470_ACC_SRTFR_REG\n");
		return false;
	}
	return true;
}

void WB_ACCELERO_Init() {
	WB_DEBUG("Accelero init\n");
	if (WB_ACCELERO_Test()) {
		WB_DEBUG("Accelero test OK\n");
	} else {
		WB_DEBUG("Accelero test FAIL !!!!\n");
	}
	SetAwake(0);
	SetSampleRate();


	if(!WRITEBYTE(0x20, 5)) {
		WB_DEBUG("!!! I2C ERROR !!! Can't write to MC6470_ACC_SRTFR_REG\n");
	}


	//SetAwake(1);

}

bool WB_ACCELERO_Test() {
	return GetStatus() != 0xFF;
}

bool WB_ACCELERO_GetRaw(int *x, int *y, int *z) {
	//SampleReady(); // clear sample ready flag if already set

	//if (!SetAwake(true)) return false;
	SetAwake(1);
	TD_RTC_Delay(TMS(35));
	bool result = false;
	do {
		/*int retry;
		for (retry=5; retry>0; retry--) {
			TD_RTC_Delay(TMS(5));
			if (SampleReady()) break;
			WB_DEBUG("accelero data is not ready yet\n");
		}
		if (retry == 0) {
			WB_DEBUG("accelero measurement timeout\n");
			break;
		}*/

		// Read measurements

		int16_t measurementsBuffer[3];
		if (READBYTES(MC6470_ACC_OUT_REG, 6, measurementsBuffer) != 6) {
			WB_DEBUG("Can't read results");
			break;
		}

		*x = (int)(int16_t)measurementsBuffer[0];
		*y = (int)(int16_t)measurementsBuffer[1];
		*z = (int)(int16_t)measurementsBuffer[2];
		// casting two uint8 (LSB and MSB) into one int16
		// Results are always little endian

		result = true;
	} while (0); // will run once
	SetAwake(0);
	//SetAwake(false);
	return result;
}
