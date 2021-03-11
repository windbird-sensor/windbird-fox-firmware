/**************************************************************************
 * @file pp_pressure.c
 * @brief Pressure Sensor API for PIOUPIOU's firmware
  * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2015 Bac Plus Zéro S.A.S.
 * (C) Copyright 2016 Altostratus SA
 * (C) Copyright 2021 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of PIOUPIOU WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/pioupiou-archive/pioupiou-v1-firmware/blob/master/README.md
 *
 ******************************************************************************/
#include <td_rtc.h>

#include "pp_debug.h"
#include "pp_i2c.h"

#define LPS25_ADDRESS 0x5D
#define LPS25_WHO_AM_I_REG 0x0F
#define LPS25_WHO_AM_I_EXPECTED 0xBD

#define LPS25_CTRL_REG1 0x20
#define LPS25_CTRL_REG1_OUTP_DTA_RATEBIT_ONESHOT 0b00000000
#define LPS25_CTRL_REG1_ACTIV_MOD 0b10000000
#define LPS25_CTRL_REG1_PWR_DOWN 0b00000000

#define LPS25_CTRL_REG2 0x21
#define LPS25_CTRL_REG2_ONE_SHOT 0b00000001

#define LPS25_STATUS_REG 0x27
#define LPS25_PD_AVAILABLE 0b00000011

#define LPS25_PRESS_OUT_XL 0x28
#define LPS25_PRESS_OUT_L 0x29
#define LPS25_PRESS_OUT_H 0x2A
#define LPS25_TEMP_OUT_L 0x2B
#define LPS25_TEMP_OUT_H 0x2C

static uint8_t buffer[1];

static bool IsDataReady() {
	PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_STATUS_REG, buffer, PP_I2C_DEFAULT_TIMEOUT);
	return (buffer[0] & LPS25_PD_AVAILABLE) == LPS25_PD_AVAILABLE;
}

static bool ConnectionTest () {
	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_WHO_AM_I_REG, buffer, PP_I2C_DEFAULT_TIMEOUT) == 1) {
		return buffer[0] == LPS25_WHO_AM_I_EXPECTED;
	} else {
		PP_DEBUG("!!! I2C ERROR !!! PP_PRESS_ConnectionTest\n");
		return false;
	}
}

void PP_PRESSURE_Shutdown() {
	PP_I2C_WriteByte(LPS25_ADDRESS, LPS25_CTRL_REG1, LPS25_CTRL_REG1_PWR_DOWN, PP_I2C_DEFAULT_TIMEOUT);
}

void PP_PRESSURE_Startup() {
	buffer[0]=LPS25_CTRL_REG1_OUTP_DTA_RATEBIT_ONESHOT | LPS25_CTRL_REG1_ACTIV_MOD;
	PP_I2C_WriteByte(LPS25_ADDRESS, LPS25_CTRL_REG1, buffer[0], PP_I2C_DEFAULT_TIMEOUT);
}

void PP_PRESSURE_Init() {
	PP_PRESSURE_Shutdown();
	PP_PRESSURE_Startup();
}

float PP_PRESSURE_Get() {

	if (!PP_I2C_WriteByte(LPS25_ADDRESS, LPS25_CTRL_REG2, LPS25_CTRL_REG2_ONE_SHOT, PP_I2C_DEFAULT_TIMEOUT)) return false;

	TD_RTC_Delay(TMS(50));

	if (!IsDataReady()) {
		PP_DEBUG(" - PD_AVAILABLE extending delay\n");
		TD_RTC_Delay(TMS(20));
		if (!IsDataReady()) {
			PP_DEBUG(" - PD_AVAILABLE !!! TIMEOUT !!!\n");
			return -1;
		}
	}

	uint8_t pressXL, pressL, pressH;

	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_PRESS_OUT_XL, buffer, PP_I2C_DEFAULT_TIMEOUT) != 1) return false;
	pressXL=buffer[0];
	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_PRESS_OUT_L, buffer, PP_I2C_DEFAULT_TIMEOUT) != 1) return false;
	pressL=buffer[0];
	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_PRESS_OUT_H, buffer, PP_I2C_DEFAULT_TIMEOUT) != 1) return false;
	pressH=buffer[0];

	uint32_t pressureRaw = (pressH << 16) + (pressL << 8) + pressXL;
	return pressureRaw / 4096.;

	/*uint8_t tempL, tempH;

	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_TEMP_OUT_L, buffer, PP_I2C_DEFAULT_TIMEOUT) != 1) return false;
	tempL=buffer[0];
	if (PP_I2C_ReadByte(LPS25_ADDRESS, LPS25_TEMP_OUT_H, buffer, PP_I2C_DEFAULT_TIMEOUT) != 1) return false;
	tempH=buffer[0];

	int16_t temperatureRaw = (tempH << 8) | tempL;
	*temperature = 42.5 + temperatureRaw / 480.;*/

}

bool PP_PRESSURE_Test(float *pressure) {

	bool result = true;

	if (!ConnectionTest()) {
		PP_DEBUG("!!! FAIL !!! Connection test (result : %d, expected : %d)\n", buffer[0], LPS25_WHO_AM_I_EXPECTED);
		result = false;
	}

	*pressure = PP_PRESSURE_Get();
	if (*pressure < 0) {
		PP_DEBUG("!!! FAIL !!! Reading test\n");
		result = false;
	}

	return result;

}

