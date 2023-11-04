#ifndef WB_EXT_LORA_H_
#define WB_EXT_LORA_H_

#include <stdint.h>
#include <stdbool.h>

void WB_EXT_LORA_SetLowPower();
void WB_EXT_LORA_Init();
bool WB_EXT_LORA_IsActive();
bool WB_EXT_LORA_Send(uint8_t* message, int length);

#endif // WB_EXT_LORA_H_
