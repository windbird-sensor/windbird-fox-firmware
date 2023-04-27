/*
 * wb_config_module.h
 *
 *  Created on: 9 Apr 2023
 *      Author: windbird-dev
 */

#ifndef WB_CONFIG_MODULE_H_
#define WB_CONFIG_MODULE_H_

#include "wb_config.h"

#define MODULE_REVISION REVISION_TD1208
#define PRODUCT_LED_PORT LED_PORT
#define PRODUCT_LED_BIT LED_BIT
#define PRODUCT_LED_BLINK 1
// PRODUCT_LED_DRIVE will be overwritten by VAUX's gpioDriveModeHigh

// enable VAUX in bootloader, to allow external UART communication
#define PRODUCT_INIT_DATA {PIP(VAUX_PORT, VAUX_BIT, PI_OUTPUT, 1),PIS(VAUX_PORT, gpioDriveModeHigh),PIP(GPS_POWER_PORT, GPS_POWER_BIT, PI_OUTPUT, 0)}

#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0


#include <td_config.h>

#endif /* WB_CONFIG_MODULE_H_ */
