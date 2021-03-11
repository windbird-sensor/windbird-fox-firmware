/**************************************************************************
 * @file pp_button.c
 * @brief Button API for PIOUPIOU's firmware
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
 
#include <em_gpio.h>
#include <td_core.h>


#include "pp_debug.h"
#include "pp_button.h"

#define BUTTON_PORT gpioPortB
#define BUTTON_BIT 11
#define BUTTON_MASK (1 << BUTTON_BIT)

#define DELAY_DEBOUNCING 10000
#define DELAY_TOO_LONG 2000000
#define DELAY_CALIBRATION 9000000

static volatile bool buttonPressedEvent;

static void InterruptCallback(uint32_t mask) {
	buttonPressedEvent=true;
}

void PP_BUTTON_Init() {

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

PP_BUTTON_State_t PP_BUTTON_Loop() {

	PP_BUTTON_State_t state = PP_BUTTON_NO_ACTION;

	uint32_t duration = 0;

	while ((GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT) == 0) && (duration < DELAY_CALIBRATION)) {
		duration++;
	}

	if (duration > 0) {

		buttonPressedEvent = 0;

		if (duration == DELAY_CALIBRATION) {
			state = PP_BUTTON_PRESSED_CALIBRATION;
		} else if (duration > DELAY_TOO_LONG) {
			// nop
		} else if (duration >= DELAY_DEBOUNCING) {
			state = PP_BUTTON_PRESSED_POWER_SWITCH;
		}

		PP_DEBUG("button event duration : %d\n", duration);
	}

	return state;
}
