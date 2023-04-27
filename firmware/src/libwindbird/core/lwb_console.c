/*
 * lwb_console.c
 *
 *  Created on: 16 Apr 2023
 *      Author: windbird-dev
 */

#include "../../wb_config.h"

#include "lwb_console.h"
#include "lwb_serial.h"

#include "../3rd/microrl/src/microrl.h"

#include <string.h>

#include <em_rtc.h>
#include <td_rtc.h>

static microrl_t rl;
static microrl_t * prl = &rl;

#define CONSOLE_STARTUP_SECONDS 4

void PrintHeader() {
	LWB_SERIAL_Print("\033[2J");
	LWB_SERIAL_Println("\033[1m");
	LWB_SERIAL_Println("    \\\\   ");
	LWB_SERIAL_Println("    (o>  ");
	LWB_SERIAL_Println(" \\\\_//)  ");
	LWB_SERIAL_Println("  \\_/_)  ");
	LWB_SERIAL_Println("   _|_   ");
	LWB_SERIAL_Println("");
	LWB_SERIAL_Println("WINDBIRD");
	LWB_SERIAL_Println("\033[m");
	LWB_SERIAL_Printfln("ID: %s", "WB1234");
	LWB_SERIAL_Println("");
	LWB_SERIAL_Printfln("HW version: %s", "1234");
	LWB_SERIAL_Printfln("SW version: %s", "1234");
	LWB_SERIAL_Println("");
}

void print(const char * str) {
	LWB_SERIAL_Print((char*)str);
}

int notFound(int argc, const char * const * argv) {
	LWB_SERIAL_Printfln("%s : unknown command.", argv[0]);
	LWB_SERIAL_Println("Enter 'help' for a list of built-in commands.");
	/*LWB_SERIAL_Println("");
	LWB_CONSOLE_HelpCmd(0,0);*/
	return 1;
}

int execute(int argc, const char * const * argv) {
	LWB_CONSOLE_callback_t callback = &notFound;

	int i;
	for (i=0; i<LWB_CONSOLE_commandsCount; i++) {
		if (strcmp(argv[0], LWB_CONSOLE_commands[i].name) == 0) {
			callback = LWB_CONSOLE_commands[i].callback;
			break;
		}
	}

	int ret = callback(argc, argv);
	if (ret == 0) {
		LWB_SERIAL_Println("\033[32mok\033[m");
	} else {
		LWB_SERIAL_Println("\033[31mFAILED\033[m");
	}
	return ret;
}

void sigint(void) {
	print ("^C catched!\n\r");
}

void RunConsole() {
	PrintHeader();
	LWB_SERIAL_Println("***CONSOLE MODE***");
	LWB_SERIAL_Println("Enter 'help' for a list of built-in commands.");
	LWB_SERIAL_Println("Enter 'exit' to reboot.");
	LWB_SERIAL_Println("");

	LWB_CONSOLE_InitDevices();

	// call init with ptr to microrl instance and print callback
	microrl_init (prl, print);
	// set callback for execute
	microrl_set_execute_callback (prl, execute);

	microrl_set_sigint_callback (prl, sigint);

	while (1) {
	    int c = LWB_SERIAL_GetChar(); // Read e.g. serial input
	    if (c < 0) {
		    TD_RTC_Sleep();
		    continue;
	    }
	    microrl_insert_char (prl, c);
	}
}

int LWB_CONSOLE_HelpCmd(int argc, const char * const * argv) {
	LWB_SERIAL_Println("List of commands:");
	int i;
	for (i=0; i<LWB_CONSOLE_commandsCount; i++) {
		LWB_SERIAL_Printfln("  %-8s %s", LWB_CONSOLE_commands[i].name, LWB_CONSOLE_commands[i].description);
	}
	return 0;
}

int LWB_CONSOLE_ExitCmd(int argc, const char * const * argv) {
	LWB_SERIAL_Println("Rebooting...");
	NVIC_SystemReset();
	return 0;
}

bool LWB_CONSOLE_LoopUntilQuit(bool sleep) {
	while (LWB_SERIAL_Available() > 0) {
		if (LWB_SERIAL_GetChar() == 'q') {
			LWB_SERIAL_Println("**[q] pressed**");
			return false;
		}
	}
	if (sleep) TD_RTC_Sleep();
	return true;
}

void LWB_CONSOLE_BootMenu() {
	#ifdef DEBUG_FORCE_CONSOLE
		RunConsole();
	#endif

	PrintHeader();
	LWB_SERIAL_Println("Hold [c] for console");
	LWB_SERIAL_Printfln("or wait %d seconds for normal startup", CONSOLE_STARTUP_SECONDS);
	LWB_SERIAL_Println("");
	LWB_SERIAL_Flush();

	uint32_t stop;
	int c;
	int lastC = 0;
	int lastLastC = 0;
	int sec;
	for (sec=0; sec<CONSOLE_STARTUP_SECONDS; sec++) {
		LWB_SERIAL_Printf("%d...", sec);
		stop = RTC_CounterGet() + 32768;
		while (RTC_CounterGet() < stop) {
			c = LWB_SERIAL_GetChar();
			if (c < 0) continue;
			if (c == 'c' && lastC == 'c' && lastLastC == 'c') {
				RunConsole(); // enter infinite loop until reboot
			}
			lastLastC = lastC;
			lastC = c;
		}
	}
	LWB_SERIAL_Println("BOOT!");
}
