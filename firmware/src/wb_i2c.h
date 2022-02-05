/**************************************************************************
 * @file WB_i2c.h
 * @brief I2C API for WINDBIRD's firmware
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

#ifndef WB_I2C_H_
#define WB_I2C_H_

#include <stdint.h>
#include <stdbool.h>

#define WB_I2C_DEFAULT_TIMEOUT 10000

void WB_I2C_Init();
void WB_I2C_Enable(bool isEnabled);
int8_t WB_I2C_ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
int8_t WB_I2C_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
bool WB_I2C_WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, uint16_t timeout);

#endif /* WB_I2C_H_ */
