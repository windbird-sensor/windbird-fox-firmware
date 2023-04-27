#ifndef LWB_SCHEDULER_H_
#define LWB_SCHEDULER_H_

#include <stdint.h>

typedef void (*LWB_SCHEDULER_handler_t)(uint32_t);

void LWB_SCHEDULER_Init(LWB_SCHEDULER_handler_t samplingServiceFunction, LWB_SCHEDULER_handler_t samplingStartFunction);
void LWB_SCHEDULER_Start();
void LWB_SCHEDULER_Stop();
uint32_t LWB_SCHEDULER_Millis();

#endif /* LWB_SCHEDULER_H_ */
