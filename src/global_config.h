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

#define SAADC_BUF_SIZE        4000 // at 200kHz it takes 20ms to fill the 4000 buffer
#define SAADC_SAMPLINGRATE_US 5    // sample every 5 µs to get the max possible 200 kHz SAADC
#define MAX_MEASUREMENTS      101  // 2 measurements take 20ms

#endif