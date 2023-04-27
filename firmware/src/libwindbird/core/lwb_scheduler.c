#include "../libwindbird.h"
#include "lwb_scheduler.h"

#include "../../wb_config.h"

#ifdef LWB_PLATFORM_TD120X
	#include <td_rtc.h>
#else
	#error "Not implemented for this platform"
#endif

bool running = false;
static volatile uint32_t millis = 0;

static LWB_SCHEDULER_handler_t samplingService = 0;
static LWB_SCHEDULER_handler_t samplingStart = 0;

static void tick (void) {
	if (!running) return;

	#ifdef LWB_PLATFORM_EFM32G
		// schedule next tick
		uint32_t rtcNow = RTC_CounterGet();
		RTC_CompareSet(TD_RTC_USER, ((rtcNow / SCHEDULER_SAMPLING_TICKS + 1) * SCHEDULER_SAMPLING_TICKS) % 0xFFFFFF );
	#endif

	// run sampling service
	samplingService(millis);

	// update clock
	millis = (millis + SCHEDULER_SAMPLING_MS) % SCHEDULER_CALENDAR_LENGTH_MS;
}


void LWB_SCHEDULER_Init(LWB_SCHEDULER_handler_t samplingServiceFunction, LWB_SCHEDULER_handler_t samplingStartFunction) {
	samplingService = samplingServiceFunction;
	samplingStart = samplingStartFunction;

	#ifdef LWB_PLATFORM_TD120X
		uint32_t msk;
		// Disable IRQs
		msk = __get_PRIMASK();
		__set_PRIMASK(1);

		TD_RTC_SetUserHandler(&tick);
		TD_RTC_EnableUserInterrupts(true);
		TD_RTC_ClearUserInterrupts();
		__set_PRIMASK(msk);
	#endif
}

void LWB_SCHEDULER_Start() {
	millis = 0;
	running = true;
	tick();
}

void LWB_SCHEDULER_Stop() {
	running = false;
}

uint32_t LWB_SCHEDULER_Millis() {
	return millis;
}

