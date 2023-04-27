/*
 * lwb_console.h
 *
 *  Created on: 17 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_CONSOLE_H_
#define LWB_CONSOLE_H_

#include <stdbool.h>

typedef int (*LWB_CONSOLE_callback_t)(int argc, const char * const * argv);

typedef struct LWB_CONSOLE_command {
	const char *name;
	LWB_CONSOLE_callback_t callback;
	const char *description;
} LWB_CONSOLE_command_t;

void LWB_CONSOLE_BootMenu();

bool LWB_CONSOLE_LoopUntilQuit(bool sleep);

extern LWB_CONSOLE_command_t LWB_CONSOLE_commands[];
extern int LWB_CONSOLE_commandsCount;
extern void LWB_CONSOLE_InitDevices(void);

int LWB_CONSOLE_HelpCmd(int argc, const char * const * argv);
int LWB_CONSOLE_ExitCmd(int argc, const char * const * argv);

#define LWB_CONSOLE_CHECKARGC(argc, expected) if (argc -1 != expected) { \
		LWB_SERIAL_Printfln("Bad arguments count. Expected %d, got %d", expected, argc - 1); \
		return 1; \
	}

#endif /* LWB_CONSOLE_H_ */
