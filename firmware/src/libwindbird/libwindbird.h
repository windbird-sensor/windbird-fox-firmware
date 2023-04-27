/*
 * libwindbird.h
 *
 *  Created on: 10 Apr 2023
 *      Author: windbird-dev
 */

#ifndef LIBWINDBIRD_H_
#define LIBWINDBIRD_H_

#ifdef EFM32G210F128
	#define LWB_PLATFORM_EFM32G
	#ifdef MODULE_REVISION
		#define LWB_PLATFORM_TD120X
	#endif
#else
	#error "Not implemented for this platform"
#endif

#endif /* LIBWINDBIRD_H_ */
