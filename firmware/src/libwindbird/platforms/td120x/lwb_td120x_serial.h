/**************************************************************************
 * @file debug.h
 * @brief Debug functions for WINDBIRD's firmware
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
#ifndef LWB_TD120X_SERIAL_H_
#define LWB_TD120X_SERIAL_H_

#include "../../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X
	#include <td_uart.h>
	#include <td_printf.h>

	#define LWB_SERIAL_Init(rate) init_printf(TD_UART_Init(rate, true, false), TD_UART_Putc, TD_UART_Start, TD_UART_Stop);

	#define LWB_SERIAL_Available() TD_UART_AvailableChars()
	#define LWB_SERIAL_GetChar() TD_UART_GetChar()

	#define LWB_SERIAL_Print(str) TD_UART_SendString(str)
	#define LWB_SERIAL_Println(str) TD_UART_SendString(str); TD_UART_SendString("\r\n")
	#define LWB_SERIAL_Printf(...) tfp_printf(__VA_ARGS__)
	#define LWB_SERIAL_Printfln(...) tfp_printf(__VA_ARGS__); TD_UART_SendString("\r\n")

	#define LWB_SERIAL_PutChar(c) TD_UART_PutChar(c)
	#define LWB_SERIAL_Write(buffer, length) TD_UART_Send(buffer, length)

	#define LWB_SERIAL_Dump(message, buffer, len) tfp_dump(message, buffer, len)

	#define LWB_SERIAL_Flush() TD_UART_Flush()


	#ifdef DEBUG
		#define LWB_SERIAL_Assert(condition) \
			if (!(condition)) { \
				LWB_SERIAL_Println("Assertion failed: " #condition); \
				NVIC_SystemReset(); \
			}
	#else
		#define LWB_SERIAL_Assert(condition)
	#endif

	#ifdef DEBUG_WINDBIRD
		#define LWB_SERIAL_Debug(...) tfp_printf(__VA_ARGS__)
		#define LWB_SERIAL_Debugln(...) tfp_printf(__VA_ARGS__); TD_UART_SendString("\r\n")
		#define LWB_SERIAL_DebugDump(message, buffer, len) tfp_dump(message, buffer, len)
	#else
		#define LWB_SERIAL_Debug(...)
		#define LWB_SERIAL_Debugln(...)
		#define LWB_SERIAL_DebugDump(...)
	#endif
#endif

#endif /* LWB_TD120X_SERIAL_H_ */
