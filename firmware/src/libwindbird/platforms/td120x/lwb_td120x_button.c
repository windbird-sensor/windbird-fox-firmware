/**************************************************************************
 * @file WB_button.c
 * @brief Button API for WINDBIRD's firmware
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
#include "../../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X

#include "../../../wb_config.h"

#include <em_gpio.h>
#include <td_core.h>


#include "../../drivers/lwb_button.h"
#include "../../core/lwb_serial.h"


#define DELAY_DEBOUNCING 10000
#define DELAY_TOO_LONG 2000000
#define DELAY_CALIBRATION 9000000

static volatile bool buttonPressedEvent;

static void InterruptCallback(uint32_t mask) {
	buttonPressedEvent=true;
}

void LWB_BUTTON_Init() {

	int type;
	IRQn_Type irq_parity;

	GPIO_PinModeSet(BUTTON_PORT, BUTTON_BIT, gpioModeInputPullFilter, 1);

	// Set up a user hook on button pin interrupt
	type = (BUTTON_MASK & TD_GPIO_ODD_MASK) ?
			TD_GPIO_USER_ODD : TD_GPIO_USER_EVEN;
	TD_GPIO_SetCallback(type, InterruptCallback, BUTTON_MASK);

	// Enable falling edge interrupts on button pin
	GPIO_IntConfig(BUTTON_PORT, BUTTON_BIT, false, true, true);

	// Clear and enable the corresponding interrupt in the CPU's Nested Vector
	// Interrupt Controller
	irq_parity =
			(BUTTON_MASK & TD_GPIO_ODD_MASK) ? GPIO_ODD_IRQn : GPIO_EVEN_IRQn;
	NVIC_ClearPendingIRQ(irq_parity);
	NVIC_EnableIRQ(irq_parity);
}

LWB_BUTTON_State_t LWB_BUTTON_Process() {

	LWB_BUTTON_State_t state = LWB_BUTTON_NO_ACTION;

	uint32_t duration = 0;

	while ((GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT) == 0) && (duration < DELAY_CALIBRATION)) {
		duration++;
	}

#ifdef DEBUG
	duration *= 5;
#endif

	if (duration > 0) {

		buttonPressedEvent = 0;

		if (duration == DELAY_CALIBRATION) {
			state = LWB_BUTTON_PRESSED_CALIBRATION;
		} else if (duration > DELAY_TOO_LONG) {
			// nop
		} else if (duration >= DELAY_DEBOUNCING) {
			state = LWB_BUTTON_PRESSED_POWER_SWITCH;
		}

		LWB_SERIAL_Debugln("button event duration : %d", duration);
	}

	return state;
}

#endif /* LWB_PLATFORM_TD120X */
