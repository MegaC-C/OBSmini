#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "error_handling.h"
#include <nrfx_wdt.h>
#include <zephyr/logging/log.h>

#define WDT_TIME_TO_RESET_MS 60000 // 1min = 60000ms

void watchdog_init(nrfx_wdt_event_handler_t wdt_event_handler);
void watchdog_feed(void);

#endif