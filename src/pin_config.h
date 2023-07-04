#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

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

#endif