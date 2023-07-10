#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

#include <hal/nrf_gpio.h>

#define BLUE_LED      NRF_GPIO_PIN_MAP(0, 17)
#define RED_LED       NRF_GPIO_PIN_MAP(0, 20)
#define TRAFO_L1      NRF_GPIO_PIN_MAP(0, 4)
#define TRAFO_L2      NRF_GPIO_PIN_MAP(0, 5)
#define TRAFO_R1      NRF_GPIO_PIN_MAP(0, 11)
#define TRAFO_R2      NRF_GPIO_PIN_MAP(1, 9)
#define OPAMPS_ON_OFF NRF_GPIO_PIN_MAP(0, 15)
#define LEFT_SENSOR   0
#define RIGHT_SENSOR  1

// this defines the delays after each pulse for changing detection limits
#define SHORT_DISTANCE_MODE_DELAY_US  2000
#define MEDIUM_DISTANCE_MODE_DELAY_US 8000
#define LONG_DISTANCE_MODE_DELAY_US   14000

// this defines the responsiveness towards the received echo signal (maximum = 4096)
// high values used for noisy short range, lower value for weaker long range signals.
// TO DO: optimize thresholds
#define SHORT_DISTANCE_LIMIT_HIGH  3500
#define MEDIUM_DISTANCE_LIMIT_HIGH 3000
#define LONG_DISTANCE_LIMIT_HIGH   2500

// this is used to dynamically find the time it takes for the transducer to decay after an pulse and be able to recieve incoming signals 
// TO DO: not yet implemented!
#define ULTRASONIC_DECAY_LIMIT_LOW 500

#define SAADC_SAMPLINGRATE_US 5    // sample every 5 µs to get the max possible 200 kHz SAADC
#define SAADC_BUF_SIZE        4000 // at 200kHz it takes 20ms to fill the 4000 buffer
#define MAX_MEASUREMENTS      101

#define TIME_TO_SYSTEM_OFF_S 30

#endif