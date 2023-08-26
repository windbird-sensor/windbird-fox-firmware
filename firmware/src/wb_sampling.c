#include "wb_config.h"

#include "libwindbird/drivers/lwb_pulsecounter.h"
#include "libwindbird/metrics/lwb_speed.h"

#include "wb_sampling.h"

static bool isComplete = false;

void WB_SAMPLING_Service(uint32_t millis) {
	/*
	 * /!\ this code runs inside an IRQ.
	 * This is necessary to keep measuring even
	 * during SIGFOX message transmission.
	 */
	uint16_t pulses = 0xFFFF;
	uint16_t pulsesExtreme = 0xFFFF;


	LWB_PULSECOUNTER_Sample();

	pulsesExtreme = LWB_PULSECOUNTER_GetLastSamples(3 * SAMPLES_PER_SECOND);
	LWB_WIND_PushExtreme(pulsesExtreme);

	switch (millis % 1000) {
		case 0:
			LWB_LED_Set();
			pulses = LWB_PULSECOUNTER_GetLastSamples(1 * SAMPLES_PER_SECOND);
			LWB_SPEED_PushSample(pulses);
			//WB_COMPASS_Trigger();
			break;
		case 250:
			LWB_LED_Clear();
			//WB_COMPASS_Read();
			break;
		case 500:
			// nop
			LWB_LED_Set();
			break;
		case 750:
			LWB_LED_Clear();
			//WB_PROPELLER_Sample();
			break;
	}
	LWB_SERIAL_Debugln("millis: %d	pulses: %d	pulsesExtreme: %d", millis, pulses, pulsesExtreme);
}

void WB_SAMPLING_Reset(uint32_t millis) {
	isComplete = false;
	LWB_PULSECOUNTER_Reset();
	LWB_SPEED_Reset();
}
