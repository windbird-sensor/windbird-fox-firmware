/*
 * lwb_utils.h
 *
 *  Created on: 10 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LWB_UTILS_H_
#define LWB_UTILS_H_

#include "../libwindbird.h"

#ifdef LWB_PLATFORM_TD120X
	#include <td_utils.h>
	#define LWB_UTILS_StackUsage() TD_STACK_Usage()
#endif

long long LWB_Utils_atolli(char *instr, char ignore);

#endif /* LWB_UTILS_H_ */
