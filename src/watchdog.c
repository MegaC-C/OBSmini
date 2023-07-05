#include "watchdog.h"

LOG_MODULE_REGISTER(watchdog);

static const nrfx_wdt_t wdt_instance = NRFX_WDT_INSTANCE(0);

static nrfx_wdt_channel_id wdt_channel_0;

static const nrfx_wdt_config_t wdt_config_normal = {
    .behaviour          = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
    .reload_value       = WDT_TIME_TO_RESET_MS,
    .interrupt_priority = NRFX_WDT_DEFAULT_CONFIG_IRQ_PRIORITY};

void watchdog_init(nrfx_wdt_event_handler_t wdt_event_handler)
{
    IRQ_CONNECT(WDT_IRQn, IRQ_PRIO_LOWEST, nrfx_wdt_0_irq_handler, NULL, 0);
    
    nrfx_err = nrfx_wdt_init(&wdt_instance, &wdt_config_normal, wdt_event_handler);
    NRFX_ERR_CHECK(nrfx_err, "initializing watchdog failed");

    nrfx_err = nrfx_wdt_channel_alloc(&wdt_instance, &wdt_channel_0);
    NRFX_ERR_CHECK(nrfx_err, "allocationg watchdog channel failed");

    nrfx_wdt_enable(&wdt_instance);
}

void watchdog_feed(void)
{
    nrfx_wdt_feed(&wdt_instance);
}
