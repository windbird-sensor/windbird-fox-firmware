#include <td_uart.h>
#include <td_rtc.h>
#include <td_scheduler.h>

#include <string.h>

#include "wb_debug.h"

#include "wb_ext_lora.h"

#define BUFFER_SIZE 30
#define AUTO_REJOIGN_PERIOD 144

static int answerBufferIndex = 0;
static char answerBuffer[BUFFER_SIZE];
// we might share this buffer with gps buffer

static bool extensionIsActive = false;
static int joinCounter = -1;

static uint8_t timeoutAlarm;
static volatile bool isTimeout = true;

static void ClearTimeout() {
	if (timeoutAlarm != 0xFF) {
		TD_SCHEDULER_Remove(timeoutAlarm);
		timeoutAlarm = 0xFF;
	}
}

static void TimeoutAlarm (uint32_t argument, uint8_t repetition) {
	isTimeout = true;
	ClearTimeout();
}

static bool SetTimeout(uint32_t seconds) {
	isTimeout = false;
	timeoutAlarm = TD_SCHEDULER_AppendIrq(seconds, 0, 0, 1, TimeoutAlarm, 0);
	if (timeoutAlarm == 0xFF) {
		WB_DEBUG("ERROR initializing LORA timeoutTimer\n");
		return false;
	}
	return true;
}

static bool ExpectAnswer(char* stringToCompare, int lengthToCompare, uint32_t timeoutSeconds) {
	// timeoutSeconds is supposed to be much less than watchdog period
	if (!SetTimeout(timeoutSeconds)) return false;
	int c;
	while (!isTimeout) {
		while ((c = TD_UART_GetChar()) >= 0) {
			//if (c >= 0) WB_DEBUG(".%s.", c);
			if (c < 0) {
				// no data. Ignore
			} else if (c == '+') {
				answerBufferIndex = 0;
				memset(answerBuffer, 0, BUFFER_SIZE);
			} else if (c == '\n') {
				//if (strncmp(answerBuffer, "AT: ERROR", 9) == 0) continue;
				//WB_DEBUG("%s\n", answerBuffer);
				if (strncmp(answerBuffer, stringToCompare, lengthToCompare) == 0) {
					ClearTimeout();
					return true;
				}
				// be cautious at the size of lengthToCompare. Must be < BUFFER_SIZE
			} else if (answerBufferIndex < BUFFER_SIZE) {
				answerBuffer[answerBufferIndex] = c;
				answerBufferIndex++;
			} else {
				// WB_DEBUG("AT buffer overflow\n");
				// ignore garbage
			}
		}
		TD_RTC_Sleep();
	}
	ClearTimeout();
	return false;
}

static void Wake() {
	TD_UART_PutChar(0xFF);
	TD_UART_PutChar(0xFF);
	TD_UART_PutChar(0xFF);
	TD_UART_PutChar(0xFF);
}

static bool Join() {
	if (joinCounter < 0) {
		TD_UART_Flush();
		Wake();
		tfp_printf("AT+JOIN=FORCE\r\n");
		if (ExpectAnswer("JOIN: Network joined", 20, 10)) {
			ExpectAnswer("JOIN: Done", 10, 2);
			joinCounter = 0;
			tfp_printf("Joined network\n");
		} else {
			tfp_printf("Failed to join\n");
			return false;
		}
	} else if (joinCounter == AUTO_REJOIGN_PERIOD) {
		joinCounter = -1;
		// rejoin next time
	} else {
		joinCounter++;
	}
	return true;
}

void WB_EXT_LORA_SetLowPower() {
	Wake();
	tfp_printf("\r\nAT+LOWPOWER=AUTOON\r\n");
}

void WB_EXT_LORA_Init() {
	TD_UART_Flush();
	Wake();
	tfp_printf("AT+ID=DevEui\r\n");
	if (ExpectAnswer("ID: DevEui", 10, 1)) {
		tfp_printf("LoRa extension board: detected\n");
		TD_UART_Send(answerBuffer, answerBufferIndex);
		TD_UART_PutChar('\n');
		extensionIsActive = true;
		Join();
	} else {
		tfp_printf("LoRa extension board: not present\n");
		extensionIsActive = false;
	}
}

bool WB_EXT_LORA_IsActive() {
	return extensionIsActive;
}

bool WB_EXT_LORA_Send(uint8_t* message, int length) {
	if (!Join()) return false;

	int i;
	TD_UART_Flush();
	Wake();
	tfp_printf("AT+MSGHEX=");
	for (i=0; i<length; i++) tfp_printf("%02x", message[i]);
	tfp_printf("\r\n");

	if (ExpectAnswer("MSGHEX: Done", 12, 10)) {
		tfp_printf("msg sent\n");
		return true;
	} else {
		tfp_printf("msg not sent\n");
		return false;
	}
}
