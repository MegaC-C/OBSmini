#ifndef PULSE_GENERATOR_H
#define PULSE_GENERATOR_H

#include "error_handling.h"
#include "pin_config.h"
#include <nrfx_pwm.h>
#include <zephyr/logging/log.h>

void pulse_generator_init();
void burst_ultrasonic_pulse_sequence();

#endif