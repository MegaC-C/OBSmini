#include "timer_and_ppi.h"

LOG_MODULE_REGISTER(timer_and_ppi);

static const nrfx_timer_t timer_to_sample_saadc_via_ppi_instance    = NRFX_TIMER_INSTANCE(1); // never use TIMER0 as it is used by BLE SoftDevice
static const nrfx_timer_t timer_to_measure_ultrasonic_echo_instance = NRFX_TIMER_INSTANCE(2);

static const nrfx_timer_config_t timer_config = {
    .frequency          = NRF_TIMER_FREQ_1MHz,
    .mode               = NRF_TIMER_MODE_TIMER,
    .bit_width          = NRF_TIMER_BIT_WIDTH_16,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context          = NULL};

static nrf_ppi_channel_t ppi_channel_to_sample_saadc_via_timmer;

void timer_and_ppi_init(nrfx_timer_event_handler_t timer_event_handler)
{
    IRQ_CONNECT(TIMER2_IRQn, IRQ_PRIO_LOWEST, nrfx_timer_2_irq_handler, NULL, 0);

    nrfx_err = nrfx_timer_init(&timer_to_sample_saadc_via_ppi_instance,
                               &timer_config,
                               NULL);
    nrfx_err = nrfx_timer_init(&timer_to_measure_ultrasonic_echo_instance,
                               &timer_config,
                               timer_event_handler);
    NRFX_ERR_CHECK(nrfx_err, "timer initialization failed");

    nrfx_timer_extended_compare(&timer_to_sample_saadc_via_ppi_instance,
                                NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_us_to_ticks(&timer_to_sample_saadc_via_ppi_instance, SAADC_SAMPLINGRATE_US),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);

    nrfx_err = nrfx_ppi_channel_alloc(&ppi_channel_to_sample_saadc_via_timmer);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_assign(ppi_channel_to_sample_saadc_via_timmer,
                                       nrfx_timer_event_address_get(&timer_to_sample_saadc_via_ppi_instance, NRF_TIMER_EVENT_COMPARE0),
                                       nrf_saadc_task_address_get((const NRF_SAADC_Type *)NRF_SAADC_BASE, NRF_SAADC_TASK_SAMPLE));
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_enable(ppi_channel_to_sample_saadc_via_timmer);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    // Channels 1, 2 and 3 are used to change the detection limit of the received echo signal
    // Channel 0 is used to record the time==distance of the echo
    nrfx_timer_compare(&timer_to_measure_ultrasonic_echo_instance,
                       NRF_TIMER_CC_CHANNEL1,
                       nrfx_timer_us_to_ticks(&timer_to_measure_ultrasonic_echo_instance, SHORT_DISTANCE_MODE_DELAY_US),
                       true);

    nrfx_timer_compare(&timer_to_measure_ultrasonic_echo_instance,
                       NRF_TIMER_CC_CHANNEL2,
                       nrfx_timer_us_to_ticks(&timer_to_measure_ultrasonic_echo_instance, MEDIUM_DISTANCE_MODE_DELAY_US),
                       true);

    nrfx_timer_compare(&timer_to_measure_ultrasonic_echo_instance,
                       NRF_TIMER_CC_CHANNEL3,
                       nrfx_timer_us_to_ticks(&timer_to_measure_ultrasonic_echo_instance, LONG_DISTANCE_MODE_DELAY_US),
                       true);
}

void timer_start(void)
{
    nrfx_timer_enable(&timer_to_measure_ultrasonic_echo_instance);
}

void timer_stop(void)
{
    nrfx_timer_disable(&timer_to_measure_ultrasonic_echo_instance);
}

void saadc_sampling_start(void)
{
    nrfx_timer_enable(&timer_to_sample_saadc_via_ppi_instance);
}

void saadc_sampling_stop(void)
{
    nrfx_timer_disable(&timer_to_sample_saadc_via_ppi_instance);
}

uint32_t get_echo_time_us(void)
{
    return nrfx_timer_capture(&timer_to_measure_ultrasonic_echo_instance, NRF_TIMER_CC_CHANNEL0);
}

void set_transducer_decay_time_us(uint32_t decay_time)
{
    nrf_timer_cc_set((&timer_to_measure_ultrasonic_echo_instance)->p_reg, NRF_TIMER_CC_CHANNEL1, decay_time);
}
