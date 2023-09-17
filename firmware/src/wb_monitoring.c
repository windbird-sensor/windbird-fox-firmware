/**************************************************************************
 * @file WB_monitoring.c
 * @brief Monitoring API for WINDBIRD's firmware
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
#include <em_emu.h>
#include <td_watchdog.h>
#include <td_scheduler.h>
#include <td_rtc.h>
#include "wb_reports.h"
#include "wb_debug.h"
#include "wb_sigfox.h"
#include "wb_gps.h"
#include "wb_button.h"
#include "wb_reports.h"
#include "wb_monitoring.h"
#include "wb_compass.h"

#define GPS_AUTOLOCATE_TIMEOUT 100

#define WATCH_PERIOD 20
#define DAILY_PERIOD 24*3600
#define FIRST_AUTOLOCATE_PERIOD 3600

#define WATCHDOG_TIMEOUT 64

#define AUTOLOCATE_PERIOD 2

static uint8_t watchTimer;
static uint8_t dailyTimer;
static uint8_t firstAutoLocateTimer;

static uint8_t dayCount;

static void WatchTimer (uint32_t argument, uint8_t repetition) {
  TD_WATCHDOG_Feed();
}

static void AutoLocate () {
  WB_GPS_PowerOn(GPS_AUTOLOCATE_TIMEOUT);
  while ((!WB_GPS_Locate()) && (WB_BUTTON_Loop() != WB_BUTTON_PRESSED_POWER_SWITCH)) {
    TD_RTC_Sleep();
  }
  WB_GPS_PowerOff();
}

static void DailyTimer (uint32_t argument, uint8_t repetition) {
  WB_REPORTS_Pause();

  if (dayCount % AUTOLOCATE_PERIOD == AUTOLOCATE_PERIOD - 1) {
    WB_DEBUG("autolocate\n");
    AutoLocate();
  }

  // reinit the compass in case it lost its configuration
  WB_COMPASS_Init();

  WB_REPORTS_Resume();

  dayCount++;
}

static void FirstAutoLocateTimer (uint32_t argument, uint8_t repetition) {
  WB_REPORTS_Pause();

  WB_DEBUG("autolocate\n");
  AutoLocate();

  WB_REPORTS_Resume();
}

void WB_MONITORING_Init () {
  if (!TD_WATCHDOG_Init(WATCHDOG_TIMEOUT)) WB_DEBUG("Watchdog failed\n");
  TD_WATCHDOG_Enable(true, false);

  watchTimer = TD_SCHEDULER_Append(WATCH_PERIOD, 0, 0, TD_SCHEDULER_INFINITE, WatchTimer, 0);
  if (watchTimer == 0xFF) WB_DEBUG("ERROR initializing MONITORING watchTimer\n");

  dailyTimer = TD_SCHEDULER_Append(DAILY_PERIOD, 0, 0, TD_SCHEDULER_INFINITE, DailyTimer, 0);
  if (dailyTimer == 0xFF) WB_DEBUG("ERROR initializing MONITORING dailyTimer\n");

  firstAutoLocateTimer = TD_SCHEDULER_Append(FIRST_AUTOLOCATE_PERIOD, 0, 0, 1, FirstAutoLocateTimer, 0);
  if (firstAutoLocateTimer == 0xFF) WB_DEBUG("ERROR initializing MONITORING dailyTimer\n");

  dayCount = 0;
}
