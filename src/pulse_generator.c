#include "pulse_generator.h"

#define NRFX_PWM_VALUES_LENGTH(array) (sizeof(array) / (sizeof(uint16_t)))
#define PWM_MAX                       25 // must be lower than (2^16)/2 = 32768, 1 as MSB is the problem

LOG_MODULE_REGISTER(pulse_generator);

static const nrfx_pwm_t pwm_for_ultrasonic_pulses_instance = NRFX_PWM_INSTANCE(0);

static const nrfx_pwm_config_t pwm_for_ultrasonic_pulses_peripheral_config = {
    .output_pins = {
        TRAFO_L1,
        TRAFO_R1,
        TRAFO_L2,
        TRAFO_R2},
    .irq_priority  = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
    .base_clock    = NRF_PWM_CLK_2MHz,
    .count_mode    = NRF_PWM_MODE_UP,
    .top_value     = PWM_MAX,
    .load_mode     = NRF_PWM_LOAD_GROUPED,
    .step_mode     = NRF_PWM_STEP_AUTO,
    .skip_gpio_cfg = false};

// nrf_pwm_values_grouped_t arrays cannot be allocated on stack (hence "static")
// and they must be in RAM (hence no "const", though its content is not changed).
// the arrays are subsequently used to generate a pwm sequence, but we only use 0
// and the value of nrfx_pwm_config_t.top_value so we aren't really using the pwm
// functionality but rather use 0% and 100% PWM to generate a given amount of ON/OFF
// pulses. With the ON/OFF time being dependent on the nrfx_pwm_config_t.base_clock
// and .top_value. How the output pins are connected to the elements of
// nrf_pwm_values_grouped_t arrays is defined by .load_mode
static nrf_pwm_values_grouped_t /*const*/ pwm_15plus1pulses[] = {{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {PWM_MAX, 0}, {0, PWM_MAX}};

static const nrf_pwm_sequence_t pwm_15plus1pulses_seq = {
    .values.p_grouped = pwm_15plus1pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_15plus1pulses),
    .repeats          = 0,
    .end_delay        = 0};

void pulse_generator_init()
{
    nrfx_err = nrfx_pwm_init(&pwm_for_ultrasonic_pulses_instance, &pwm_for_ultrasonic_pulses_peripheral_config, NULL, NULL);
    NRFX_ERR_CHECK(nrfx_err, "Cannot start PWM peripheral")
}

void burst_ultrasonic_pulse_sequence()
{
    nrfx_pwm_simple_playback(&pwm_for_ultrasonic_pulses_instance, &pwm_15plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
}
