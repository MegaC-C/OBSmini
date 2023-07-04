#include "error_handling.h"

int nrfx_err = NRFX_SUCCESS;
int err      = 0;

// TODO: perform reboot or similar
void error_handling()
{
    for (int i = 0; i < 5; i++)
    {
        nrf_gpio_pin_toggle(RED_LED);
        k_msleep(200);
    }
    nrf_gpio_pin_set(RED_LED);
}