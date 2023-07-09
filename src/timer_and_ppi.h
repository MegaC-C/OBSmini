#ifndef TIMER_AND_PPI_H
#define TIMER_AND_PPI_H

#include "error_handling.h"
#include <nrfx_ppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <zephyr/logging/log.h>

void timer_and_ppi_init(nrfx_timer_event_handler_t timer_event_handler);
void timer_stop(void);
void timer_start(void);
uint32_t get_echo_time_us(void);

#endif