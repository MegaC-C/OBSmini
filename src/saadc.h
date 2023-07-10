#ifndef SAADC_H
#define SAADC_H

#include "error_handling.h"
#include <nrfx_saadc.h>
#include <zephyr/logging/log.h>

void saadc_init(nrfx_saadc_event_handler_t saadc_event_handler);
int16_t get_battery_voltage_mV(nrfx_saadc_event_handler_t saadc_event_handler);
void saadc_buffer_set(void);

#endif