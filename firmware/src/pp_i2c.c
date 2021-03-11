/**************************************************************************
 * @file pp_i2c.c
 * @brief I2C API for PIOUPIOU's firmware
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


#include <em_cmu.h>
#include <em_gpio.h>
#include <em_i2c.h>

#define I2C_SDA_PORT gpioPortA
#define I2C_SDA_PIN 0
#define I2C_SDA_MODE gpioModeWiredAnd
#define I2C_SDA_DOUT 1

#define I2C_SCL_PORT gpioPortA
#define I2C_SCL_PIN 1
#define I2C_SCL_MODE gpioModeWiredAnd
#define I2C_SCL_DOUT 1

/** Initialize I2C0
 */
void PP_I2C_Init() {
	CMU_ClockEnable(cmuClock_I2C0, true);

	GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_MODE, I2C_SDA_DOUT);
	GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_MODE, I2C_SCL_DOUT);

	I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;

	const I2C_Init_TypeDef init = I2C_INIT_DEFAULT;
	I2C_Init(I2C0, &init);

	I2C_Enable(I2C0, true);
}


/** Enable or disable I2C
 * @param isEnabled true = enable, false = disable
 */
void PP_I2C_Enable(bool isEnabled) {

 I2C_Enable(I2C0, isEnabled);
 CMU_ClockEnable(cmuClock_I2C0, isEnabled);

}


I2C_TransferReturn_TypeDef Transfer(I2C_TransferSeq_TypeDef *seq, uint16_t timeout) {

	I2C_TransferReturn_TypeDef ret;
	/* Do a polled transfer */
	ret = I2C_TransferInit(I2C0, seq);

	while (ret == i2cTransferInProgress && timeout--) {
		ret = I2C_Transfer(I2C0);
	}

	return(ret);

}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t PP_I2C_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {

	I2C_TransferSeq_TypeDef seq;

	uint8_t regid[1];

	seq.addr = devAddr << 1;
	seq.flags = I2C_FLAG_WRITE_READ;

	/* Select register to be read */
	regid[0] = regAddr;
	seq.buf[0].data = regid;
	seq.buf[0].len = 1;

	/* 1 bytes reg */
	seq.buf[1].data = data;
	seq.buf[1].len = length;

	if (Transfer(&seq, timeout) == i2cTransferDone) {
		return seq.buf[1].len;
	} else {
		return false;
	}

}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t PP_I2C_ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
	return PP_I2C_ReadBytes(devAddr, regAddr, 1, data, timeout);
}


/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool PP_I2C_WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, uint16_t timeout) {

  I2C_TransferSeq_TypeDef seq;

  uint8_t writeData[3];

  seq.addr = devAddr << 1;
  seq.flags = I2C_FLAG_WRITE;

  /* Select register to be written */
  writeData[0] = regAddr;
  seq.buf[0].data = writeData;

  /* Only 1 byte reg */
  writeData[1] = data;
  seq.buf[0].len = 2;

  if (Transfer(&seq, timeout) == i2cTransferDone) {
  	  return true;
    } else {
  	  return false;
    }

}

