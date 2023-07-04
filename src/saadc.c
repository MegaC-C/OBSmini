#include "saadc.h"

LOG_MODULE_REGISTER(saadc);

static nrf_saadc_value_t saadc_samples[SAADC_BUF_SIZE];

static const nrfx_saadc_adv_config_t saadc_peripheral_config = {
    .oversampling      = NRF_SAADC_OVERSAMPLE_DISABLED,
    .burst             = NRF_SAADC_BURST_DISABLED,
    .internal_timer_cc = 0,
    .start_on_end      = true};

static const nrfx_saadc_channel_t saadc_left_sensor_channel_config = {
    .channel_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_2,
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_3US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED},
    .pin_p         = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN1,
    .pin_n         = NRF_SAADC_INPUT_DISABLED,
    .channel_index = LEFT_SENSOR};

static const nrfx_saadc_channel_t saadc_right_sensor_channel_config = {
    .channel_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_2,
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_3US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED},
    .pin_p         = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN4,
    .pin_n         = NRF_SAADC_INPUT_DISABLED,
    .channel_index = RIGHT_SENSOR};

static const nrfx_saadc_channel_t saadc_battery_voltage_channel_config = {
    .channel_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_2,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED},
    .pin_p         = (nrf_saadc_input_t)NRF_SAADC_INPUT_VDDHDIV5,
    .pin_n         = NRF_SAADC_INPUT_DISABLED,
    .channel_index = 2};

void saadc_init(nrfx_saadc_event_handler_t saadc_event_handler)
{
    IRQ_CONNECT(SAADC_IRQn, IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, NULL, 0);

    nrfx_err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ERR_CHECK(nrfx_err, "initializing SAADC peripheral failed");

    nrfx_err = nrfx_saadc_offset_calibrate(NULL);
    NRFX_ERR_CHECK(nrfx_err, "Cannot calibrate SAADC")

    nrfx_err = nrfx_saadc_channel_config(&saadc_left_sensor_channel_config);
    nrfx_err = nrfx_saadc_channel_config(&saadc_right_sensor_channel_config);
    NRFX_ERR_CHECK(nrfx_err, "Configuring SAADC channels failed");

    nrfx_err = nrfx_saadc_advanced_mode_set(nrfx_saadc_channels_configured_get(),
                                            NRF_SAADC_RESOLUTION_12BIT,
                                            &saadc_peripheral_config,
                                            saadc_event_handler);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC mode failed");

    nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC buffer failed failed");

    nrfx_err = nrfx_saadc_mode_trigger();
    NRFX_ERR_CHECK(nrfx_err, "triggering SAADC mode failed");
}

int16_t get_battery_voltage_mV(nrfx_saadc_event_handler_t saadc_event_handler)
{
    // reinitialize SAADC for single VDDH (battery voltage) conversion in blocking mode
    nrfx_saadc_uninit();

    nrfx_err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ERR_CHECK(nrfx_err, "initializing SAADC peripheral failed");

    nrfx_err = nrfx_saadc_offset_calibrate(NULL);
    NRFX_ERR_CHECK(nrfx_err, "Cannot calibrate SAADC")

    nrfx_err = nrfx_saadc_channel_config(&saadc_battery_voltage_channel_config);
    NRFX_ERR_CHECK(nrfx_err, "Configuring SAADC channels failed");

    nrfx_err = nrfx_saadc_simple_mode_set(nrfx_saadc_channels_configured_get(),
                                          NRF_SAADC_RESOLUTION_12BIT,
                                          NRF_SAADC_OVERSAMPLE_DISABLED,
                                          NULL);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC mode failed");

    nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], 1);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC buffer failed failed");

    nrfx_err = nrfx_saadc_mode_trigger();
    NRFX_ERR_CHECK(nrfx_err, "triggering SAADC mode failed");

    // reinitialize SAADC for continous PPI triggered conversions
    nrfx_saadc_uninit();
    saadc_init(saadc_event_handler);
    

    // 1.465 results from combining NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_GAIN1_2, NRF_SAADC_REFERENCE_INTERNAL and NRF_SAADC_INPUT_VDDHDIV5
    return (saadc_samples[0] * 1.465);
}

void saadc_buffer_set(void)
{
    nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
    NRFX_ERR_CHECK(nrfx_err, " failed");
}