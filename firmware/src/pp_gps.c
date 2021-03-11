/**************************************************************************
 * @file pp_gps.c
 * @brief GPS API for PIOUPIOU's firmware
  * @author Nicolas BALDECK
 ******************************************************************************
 * @section License
 * (C) Copyright 2015 Bac Plus Zéro S.A.S.
 * (C) Copyright 2016 Altostratus SA
 * (C) Copyright 2021 OpenWindMap SCIC SA
 ******************************************************************************
 *
 * This file is a part of PIOUPIOU WIND SENSOR.
 * Any use of this source code is subject to the license detailed at
 * https://github.com/pioupiou-archive/pioupiou-v1-firmware/blob/master/README.md
 *
 ******************************************************************************/
 
#include <efm32.h>
#include <em_gpio.h>
#include <td_core.h>
#include <td_uart.h>
#include <td_utils.h>
#include <td_scheduler.h>
#include <td_watchdog.h>

#include "pp_gps.h"
#include "pp_debug.h"
#include "pp_sigfox.h"

#define GPS_POWER_PORT gpioPortB
#define GPS_POWER_BIT 13

#define GPS_BACKUP_PORT gpioPortC
#define GPS_BACKUP_BIT 15

#define BUFFER_SIZE 255

#define FIXES_SIZE 5

static char nmeaBuffer[BUFFER_SIZE];
static uint16_t nmeaBufferIndex;

static uint8_t timeoutAlarm;
static volatile bool hasTimedOutEvent;

static uint8_t fixesCount;
static PP_GPS_Fix_t fixes[FIXES_SIZE];

static bool gpsIsEnabled;

static void TimeoutTimer(uint32_t arg, uint8_t repetition) {
	hasTimedOutEvent = true;
	PP_DEBUG("GPS TIMEOUT IRQ...");
}

static int hexToInt(char s[]) {
  //strtol is not available (because using td_utils.h instead of stdlib.h)
    int hexdigit, i, inhex, n;
    i=0;
    if(s[i] == '0') {
        ++i;
        if(s[i] == 'x' || s[i] == 'X'){
            ++i;
        }
    }

    n = 0;
    inhex = true;
    for(; inhex == true; ++i) {
        if(s[i] >= '0' && s[i] <= '9') {
            hexdigit = s[i] - '0';
        } else if(s[i] >= 'a' && s[i] <= 'f') {
            hexdigit = s[i] - 'a' + 10;
        } else if(s[i] >= 'A' && s[i] <= 'F') {
            hexdigit = s[i] - 'A' + 10;
        } else {
            inhex = false;
        }

        if(inhex == true) {
            n = 16 * n + hexdigit;
        }
    }

    return n;
}

static uint8_t GetFixQuality() {
	uint8_t quality = 0xFF;
	if (fixesCount >= FIXES_SIZE) {
		quality = 0;
		uint8_t i;
		for (i=0; i<FIXES_SIZE; i++) {
			quality += fixes[i].hdop;
		}
		quality = (quality*10)/FIXES_SIZE/10;
	}
	PP_DEBUG("Fix quality %d\n", quality);
	return quality;
}

static void ParseNMEA_ExtractWord(char* nmea, uint16_t *nmeaPos, char* buffer) {
	uint16_t bufferPos = 0;
	while (*nmeaPos < BUFFER_SIZE) {
		char c = nmea[*nmeaPos];
		*nmeaPos += 1;
		if (c == ',' || c == '*') {
			buffer[bufferPos]=0;
			break;
		}
		buffer[bufferPos] = c;
		bufferPos++;
	}

	//PP_DEBUG("--%s--\n", buffer);
}

static int32_t ParseNMEA_ExtractLatitude(char* nmea, uint16_t *nmeaPos, char* buffer) {
	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer); // lat

	char tempBuffer[7];

	// extracting degrees
	tempBuffer[0]=buffer[0];
	tempBuffer[1]=buffer[1];
	tempBuffer[2]=0;
	int32_t latitude = atoi(tempBuffer)*1000000;

	//extracting minutes
	tempBuffer[0]=buffer[2];
	tempBuffer[1]=buffer[3];
	tempBuffer[2]=buffer[5];
	tempBuffer[3]=buffer[6];
	tempBuffer[4]=buffer[7];
	tempBuffer[5]=buffer[8];
	tempBuffer[6]=0;
	latitude += atoi(tempBuffer)*100/60; // converting to degrees

	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	// lat hemi
	if (buffer[0] == 'S') {
		latitude = -latitude;
	} else if (buffer[0] != 'N') {
		latitude = 0;
	}

	return latitude;
}

static int32_t ParseNMEA_ExtractLongitude(char* nmea, uint16_t *nmeaPos, char* buffer) {
	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer); //lon
	char tempBuffer[7];

	// extracting degrees
	tempBuffer[0]=buffer[0];
	tempBuffer[1]=buffer[1];
	tempBuffer[2]=buffer[2];
	tempBuffer[3]=0;
	int32_t longitude = atoi(tempBuffer)*1000000;

	//extracting minutes
	tempBuffer[0]=buffer[3];
	tempBuffer[1]=buffer[4];
	tempBuffer[2]=buffer[6];
	tempBuffer[3]=buffer[7];
	tempBuffer[4]=buffer[8];
	tempBuffer[5]=buffer[9];
	tempBuffer[6]=0;
	longitude += atoi(tempBuffer)*100/60; // converting to degrees

	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	// lon hemi
	if (buffer[0] == 'W') {
		longitude = -longitude;
	} else if (buffer[0] != 'E') {
		longitude = 0;
	}

	return longitude;

}



static bool ParseNMEA_ExtractChecksum(char* nmea, uint16_t *nmeaPos, char* buffer) {
	int16_t checksumLen = *nmeaPos-1;
	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer); // checksum
	uint8_t checksumExpected = hexToInt(buffer);
	uint8_t checksum = 0;
	int i;
	for (i=0; i<checksumLen; i++) {
		checksum ^= nmea[i];
	}
	//PP_DEBUG("checksum %d %d\n", checksumExpected, checksum);
	return checksum == checksumExpected;
}

static bool ParseNMEA_ExtractValidity(char* nmea, uint16_t *nmeaPos, char* buffer) {
	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	return (buffer[0] == '1' || buffer[0] == '2');
}

static uint16_t ParseNMEA_ExtractHDOP(char* nmea, uint16_t *nmeaPos, char* buffer) {
	// horizontal dilution of precision
	// < 1 Ideal
	// 1-2 Excellent
	// 2-5 Good
	// 5-10 Moderate
	// 10-20 Fair
	// >20 Poor

	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	return (atolli(buffer, '.')+50)/100;
}

static uint16_t ParseNMEA_ExtractAltitude(char* nmea, uint16_t *nmeaPos, char* buffer) {

	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	// MSL altitude
	uint16_t altitude = atoi(buffer);

	ParseNMEA_ExtractWord(nmea, &*nmeaPos, buffer);
	// altitude unit
	if (buffer[0] != 'M') altitude = 0;

	return altitude;

}

static bool ParseNMEA(char* nmea) {
	//char nmea[] = "GPGGA,141339.000,4518.1883,S,00554.4083,W,2,7,1.22,227.6,M,48.5,M,,*5D";

	if (strncmp(nmea, "GPGGA,", 6) != 0) return false;
	//only listen for GPGGA messages

	char buffer[BUFFER_SIZE];
	uint16_t nmeaPos=6;

	PP_GPS_Fix_t fix;

	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // time
	fix.latitude = ParseNMEA_ExtractLatitude(nmea, &nmeaPos, buffer); //latitude
	fix.longitude = ParseNMEA_ExtractLongitude(nmea, &nmeaPos, buffer); //longitude
	bool isFixValid = ParseNMEA_ExtractValidity(nmea, &nmeaPos, buffer); // fix
	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // sat count
	fix.hdop = ParseNMEA_ExtractHDOP(nmea, &nmeaPos, buffer); //hdop
	fix.altitude = ParseNMEA_ExtractAltitude(nmea, &nmeaPos, buffer); //altitude
	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // geoid separation
	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // geoid separation unit
	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // Age of Diff. Corr.
	ParseNMEA_ExtractWord(nmea, &nmeaPos, buffer); // Diff. Ref. Station ID
	bool isChecksumValid = ParseNMEA_ExtractChecksum(nmea, &nmeaPos, buffer);
	//isChecksumValid = true;

	PP_DEBUG("%s\n", nmea);
	PP_DEBUG("Lat: %d Lon: %d Alti: %d HDOP: %d\n", fix.latitude, fix.longitude, fix.altitude, fix.hdop);

	if (isFixValid && isChecksumValid) {
		PP_DEBUG("Valid Fix\n", isFixValid, isChecksumValid);
		fixesCount++;
		fixes[fixesCount % FIXES_SIZE]=fix;
		return true;
	} else {
		PP_DEBUG("Invalid Fix %d %d\n", isFixValid, isChecksumValid);
		return false;
	}


}

static bool ListenNMEA() {
	//PP_DEBUG("PP_GPS_ProcessNMEA : %d\n", TD_UART_AvailableChars());

	int c;
	while ((c = TD_UART_GetChar()) >= 0) {
		if (c == '$') {
			nmeaBufferIndex = 0;
			memset(nmeaBuffer, 0, BUFFER_SIZE);
		} else if (c == '\n') {
			//PP_DEBUG("%s\n", NMEABuffer);
			return ParseNMEA(nmeaBuffer);
		} else if (nmeaBufferIndex < BUFFER_SIZE) {
			nmeaBuffer[nmeaBufferIndex] = c;
			nmeaBufferIndex++;
		} else {
			PP_DEBUG("NMEA buffer overflow\n");
		}
	}

	return false;
}


void PP_GPS_Init() {

	// the serial port should already be initialized in the main program.

	// GPS is turned off at startup
	GPIO_PinModeSet(GPS_POWER_PORT, GPS_POWER_BIT, gpioModePushPull, 0);
	GPIO_PinModeSet(GPS_BACKUP_PORT, GPS_BACKUP_BIT, gpioModePushPull, 0);

	gpsIsEnabled = false;
	timeoutAlarm = 0xFF;

}

void PP_GPS_PowerOn(uint16_t timeout) {
	PP_DEBUG("POWER ON GPS...");

	GPIO_PinOutSet(GPS_POWER_PORT, GPS_POWER_BIT);
	GPIO_PinOutSet(GPS_BACKUP_PORT, GPS_BACKUP_BIT);
	gpsIsEnabled = true;
	fixesCount = 0;
	PP_DEBUG("OK\n");

	if (timeoutAlarm != 0xFF) {
		TD_SCHEDULER_Remove(timeoutAlarm);
	}
	timeoutAlarm = TD_SCHEDULER_AppendIrq(timeout, 0 , 0, 1, TimeoutTimer, 0);
	if (timeoutAlarm == 0xFF) PP_DEBUG("ERROR initializing GPS timeoutAlarm\n");
	hasTimedOutEvent = false;

	// clear UART buffer
	TD_UART_Flush();

}

void PP_GPS_PowerOff() {
	PP_DEBUG("POWER OFF GPS...");

	GPIO_PinOutClear(GPS_POWER_PORT, GPS_POWER_BIT);
	GPIO_PinOutClear(GPS_BACKUP_PORT, GPS_BACKUP_BIT);
	gpsIsEnabled = false;

	if (timeoutAlarm != 0xFF) {
		TD_SCHEDULER_Remove(timeoutAlarm);
		timeoutAlarm = 0xFF;
	}

	PP_DEBUG("OK\n");

	// clear UART buffer
	TD_UART_Flush();

}

bool PP_GPS_Locate() {
	if (!gpsIsEnabled) return true; // exit loop if gps is not enabled
	TD_WATCHDOG_Feed();
	if (hasTimedOutEvent || (ListenNMEA() && (GetFixQuality() < 3 || fixesCount > 10))) {
		if (fixesCount < FIXES_SIZE) {
			PP_DEBUG("GPS TIMED OUT %d %d\n", fixesCount, GetFixQuality());
			PP_SIGFOX_LocationFailureMessage ();
		} else {
			PP_DEBUG("GPS SUCCESS %d %d\n", fixesCount, GetFixQuality());
			PP_GPS_Fix_t fix;
			memset(&fix, 0, sizeof(fix));
			uint16_t hdopAvg = 0; // needs 16 bits
			uint32_t altitudeAvg = 0; // needs 32 bits
			int i;
			for (i=0; i< FIXES_SIZE; i++) {
				fix.latitude += fixes[i].latitude;
				fix.longitude += fixes[i].longitude;
				altitudeAvg += fixes[i].altitude;
				hdopAvg += fixes[i].hdop;
			}
			fix.latitude /= FIXES_SIZE;
			fix.longitude /= FIXES_SIZE;
			fix.altitude = (int32_t)(altitudeAvg / FIXES_SIZE);
			fix.hdop = (uint16_t)(hdopAvg / FIXES_SIZE);

			PP_DEBUG("%d\t%d\t%d\t%d\n", fix.latitude, fix.longitude, fix.altitude, fix.hdop);

			PP_SIGFOX_LocationMessage(fix);
		}
		return true;
	}
	return false;
}

