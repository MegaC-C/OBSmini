// This code is written for the OBSmini with the aim to visualize the recieved echo signals as measured by the nRF52833.
// The results can be used to tune limits/timings etc.
// 38.5mA (36mA from ultrasonic bursts + 2.5mA from CPU, BLE, OpAmps etc.) when connected and sampling with 15+1 pulses every 20ms
// 8µA when off (deep sleeping waiting for NFC)
//
// Most code comes from official examples:
// NFC/Powermanagement: nrf/samples/nfc/system_off
// SAADC/PPI: https://github.com/NordicPlayground/nRF52-ADC-examples/tree/master/nrfx_saadc_multi_channel_ppi

// BLE: webinar: Developing Bluetooth Low Energy products using nRF Connect SDK
// PWM: nRF5_SDKv17.0.2/examples/peripheral/pwm_driver
// DFU_OTA: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/working_with_nrf/nrf52/developing.html#fota-updates
// with DFU_OTA: the "merged.hex" is used to drag´n´drop flash and the "app_update.bin" is used for DFU.
// without DFU_OTA: the "zephyr.bin" is used to drag´n´drop flash (all these files are found in ...\myProjectFolder\build\zephyr)
// MCUBoot: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/app_dev/bootloaders_and_dfu/index.html
//		and https://github.com/hellesvik-nordic/samples_for_nrf_connect_sdk/tree/1111836cd720127c7f2b0dc0bec9f7ef496b8954/bootloader_samples
//
// in general when starting from scratch only 4 files are touched: main.c, CMakeLists.txt, prj.conf and a new custom key in /custom_key_dir needs to be added
// the rest (build configurations, devicetree, ncs root files, etc.) are untouched (except for missed ncs/zephyr pull requests which are added manually, should be obsolete in future)
// https://github.com/zephyrproject-rtos/zephyr/pull/57886/commits/15e7ab19fc0b8d51942c3cca5d1b13df0ebdbec0
// https://github.com/zephyrproject-rtos/zephyr/pull/56309/commits/2094e19a3c58297125c1289ea0ddec89db317f96
//
//
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include <nfc/ndef/launchapp_msg.h>
#include <nfc_t2t_lib.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>

#include <nrfx_ppi.h>
#include <nrfx_pwm.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_wdt.h>

#include <helpers/nrfx_reset_reason.h>

// forward declarations	------------------------------------------------------------------------------------------------------------------------
void ble_connected_handler(struct bt_conn *conn, uint8_t err);
void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason);
void ble_chrc_ccc_cfg_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
int16_t get_battery_voltage_mV();
void burst_ultrasonic_pulse_sequence();
void encode_16bit_to_8bit_array(int16_t *data_array, int8_t *ble_send_array, uint16_t ble_send_array_length);
void error_handling();

#define NRFX_ERR_CHECK(nrfx_err, msg)   \
    if (nrfx_err != NRFX_SUCCESS)       \
    {                                   \
        LOG_ERR(msg " - %d", nrfx_err); \
        error_handling();               \
    }
#define ERR_CHECK(err, msg)        \
    if (err)                       \
    {                              \
        LOG_ERR(msg " - %d", err); \
        error_handling();          \
    }
#define NRFX_PWM_VALUES_LENGTH(array) (sizeof(array) / (sizeof(uint16_t)))

#define BT_UUID_REMOTE_SERV_VAL        BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_REMOTE_SERVICE         BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_BUTTON_CHRC_VAL BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_REMOTE_BUTTON_CHRC     BT_UUID_DECLARE_128(BT_UUID_REMOTE_BUTTON_CHRC_VAL)
#define BT_UUID_REMOTE_BUTTON_CHRC     BT_UUID_DECLARE_128(BT_UUID_REMOTE_BUTTON_CHRC_VAL)
#define BLE_DEVICE_NAME                CONFIG_BT_DEVICE_NAME
#define BLE_DEVICE_NAME_LEN            (sizeof(BLE_DEVICE_NAME) - 1)

#define BLUE_LED          NRF_GPIO_PIN_MAP(0, 17)
#define RED_LED           NRF_GPIO_PIN_MAP(0, 20)
#define SYSTEM_ON_LED     BLUE_LED
#define BLE_CONNECTED_LED BLUE_LED
#define NFC_FIELD_LED     RED_LED
#define ERROR_LED         RED_LED
#define TRAFO_L1          NRF_GPIO_PIN_MAP(0, 4)
#define TRAFO_L2          NRF_GPIO_PIN_MAP(0, 5)
#define TRAFO_R1          NRF_GPIO_PIN_MAP(0, 11)
#define TRAFO_R2          NRF_GPIO_PIN_MAP(1, 9)
#define OPAMPS_ON_OFF     NRF_GPIO_PIN_MAP(0, 15)

#define SAADC_BUF_SIZE          4000 // at 200kHz it takes 20ms to fill the 4000 buffer
#define SAADC_SAMPLINGRATE_US   5    // sample every 5 µs to get the max possible 200 kHz SAADC
#define MAX_REC_COUNT           1
#define NDEF_MSG_BUF_SIZE       512
#define PWM_MAX                 25 // must be lower than (2^16)/2 = 32768, 1 as MSB is the problem
#define TIME_TO_SYSTEM_OFF_S    30
#define WDT_TIME_TO_RESET_MS    300000 // 5min = 5*60000ms
#define LEFT_SENSOR             0
#define RIGHT_SENSOR            1
#define ECHO_RECEIVE_LIMIT_HIGH 2000
#define INITIAL_PULSE_LIMIT_LOW 200
#define MAX_MEASUREMENTS        101
#define SAADC_EVENT_BIT         BIT(0)

// pre kernel initialization ------------------------------------------------------------------------------------------------------------------------
LOG_MODULE_REGISTER(logging, LOG_LEVEL_DBG);
K_EVENT_DEFINE(saadc_done);
int nrfx_err = NRFX_SUCCESS;
int err      = 0;

// Prevent deep sleep (system off) from being entered on long timeouts
// or `K_FOREVER` due to the default residency policy.
// This has to be done before anything tries to sleep, which means
// before the threading system starts up between PRE_KERNEL_2 and
// POST_KERNEL.  Do it at the start of PRE_KERNEL_2.
int prevent_system_off(void)
{
    pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
    return 0;
}
SYS_INIT(prevent_system_off, PRE_KERNEL_2, 0);

// if GPIO output voltage (REGOUT0) is still set to default '111' (1.8 volts) increase GPIO voltage to '100' (3.0 volts) to drive mosfets properly.
// it's in non-volatile register so we must use NVMC. Only bit flips from '1' to '0' are possible without debugger.
// this is copy paste from "zephyr/boards/arm/nrf52840dongle_nrf52840/board.c"
int set_REGOUT0_to_3V0(void)
{
    if ((nrf_power_mainregstatus_get(NRF_POWER) == NRF_POWER_MAINREGSTATUS_HIGH) &&
        ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) == (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
        }
        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) | (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);
        NRF_NVMC->CONFIG  = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
        }
        /* a reset is required for changes to take effect */
        NVIC_SystemReset();
    }
    return 0;
}
SYS_INIT(set_REGOUT0_to_3V0, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

// BLE ------------------------------------------------------------------------------------------------------------------------
BT_GATT_SERVICE_DEFINE(remote_srv,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_BUTTON_CHRC,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              NULL, NULL, NULL),
                       BT_GATT_CCC(ble_chrc_ccc_cfg_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

struct bt_conn *current_ble_conn;

const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN)};

struct bt_conn_cb bluetooth_callbacks = {
    .connected    = ble_connected_handler,
    .disconnected = ble_disconnected_handler};

const int8_t ble_left_sensor_flag[2]     = {0b00000000, 0b10000000};
const int8_t ble_right_sensor_flag[2]    = {0b00000001, 0b10000000};
const int8_t ble_battery_voltage_flag[2] = {0b00000010, 0b10000000};
int8_t ble_send_array[MAX_MEASUREMENTS * 2];
bool ble_notif_enabled = false;

// SAADC ------------------------------------------------------------------------------------------------------------------------
nrfx_saadc_adv_config_t saadc_peripheral_config = {
    .oversampling      = NRF_SAADC_OVERSAMPLE_DISABLED,
    .burst             = NRF_SAADC_BURST_DISABLED,
    .internal_timer_cc = 0,
    .start_on_end      = true};

nrfx_saadc_channel_t saadc_left_sensor_channel_config = {
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

nrfx_saadc_channel_t saadc_right_sensor_channel_config = {
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

nrfx_saadc_channel_t saadc_battery_voltage_channel_config = {
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

nrf_saadc_value_t saadc_samples[SAADC_BUF_SIZE];
uint8_t measurements      = 0;
bool saadc_buffer_is_full = false;
bool is_SAADC_done        = false;

// TIMER/PPI ------------------------------------------------------------------------------------------------------------------------
nrfx_timer_t timer_to_sample_saadc_via_ppi_instance        = NRFX_TIMER_INSTANCE(1); // never use TIMER0 as it is used by BLE SoftDevice
nrfx_timer_t timer_to_measure_ultrasonic_response_instance = NRFX_TIMER_INSTANCE(2);

nrfx_timer_config_t timer_config = {
    .frequency          = NRF_TIMER_FREQ_1MHz,
    .mode               = NRF_TIMER_MODE_TIMER,
    .bit_width          = NRF_TIMER_BIT_WIDTH_16,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context          = NULL};

uint16_t ultrasonic_response_time[MAX_MEASUREMENTS];
uint16_t ultrasonic_response_time_left  = UINT16_MAX;
uint16_t ultrasonic_response_time_right = UINT16_MAX;
nrf_ppi_channel_t ppi_channel_to_sample_saadc_via_timmer;

// PWM ------------------------------------------------------------------------------------------------------------------------
nrfx_pwm_t pwm_for_ultrasonic_pulses_instance = NRFX_PWM_INSTANCE(0);

nrfx_pwm_config_t const pwm_for_ultrasonic_pulses_peripheral_config = {
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
// TRAFO_L1  NRFX_PWM_PIN_NOT_USED
// TRAFO_R1  NRFX_PWM_PIN_NOT_USED
// TRAFO_L2  NRFX_PWM_PIN_NOT_USED
// TRAFO_R2  NRFX_PWM_PIN_NOT_USED

// nrf_pwm_values_grouped_t arrays cannot be allocated on stack (hence "static")
// and they must be in RAM (hence no "const", though its content is not changed).
// the arrays are subsequently used to generate a pwm sequence, but we only use 0
// and the value of nrfx_pwm_config_t.top_value so we aren't really using the pwm
// functionality but rather use 0% and 100% PWM to generate a given amount of ON/OFF
// pulses. With the ON/OFF time being dependent on the nrfx_pwm_config_t.base_clock
// and .top_value. How the output pins are connected to the elements of
// nrf_pwm_values_grouped_t arrays is defined by .load_mode
static nrf_pwm_values_grouped_t /*const*/ pwm_15plus1pulses[] = {{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {PWM_MAX, 0}, {0, PWM_MAX}};

const nrf_pwm_sequence_t pwm_15plus1pulses_seq = {
    .values.p_grouped = pwm_15plus1pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_15plus1pulses),
    .repeats          = 0,
    .end_delay        = 0};

// NFC ------------------------------------------------------------------------------------------------------------------------
const uint8_t android_pkg_name[] = {
    'a', 'p', 'p', 'i', 'n', 'v', 'e', 'n', 't', 'o', 'r', '.', 'a', 'i', '_', 'c', 'o', 'r', 'v', 'i', 'n', 'l', 'o', 's', 's', 'i', 'n', '.', 'O', 'B', 'S', 'm', 'i', 'n', 'i', '_', 'A', 'D', 'C'};
// 'c', 'o', 'm', '.', 'e', 'x', 'a', 'm', 'p', 'l', 'e', '.', 'o', 'b', 's', 'm', 'i', 'n', 'i'};

uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

// WDT ------------------------------------------------------------------------------------------------------------------------
const nrfx_wdt_t wdt_instance = NRFX_WDT_INSTANCE(0);
nrfx_wdt_channel_id wdt_channel_0;

nrfx_wdt_config_t wdt_config_normal = {
    .behaviour          = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
    .reload_value       = WDT_TIME_TO_RESET_MS,
    .interrupt_priority = NRFX_WDT_DEFAULT_CONFIG_IRQ_PRIORITY};

// interrupt handlers ------------------------------------------------------------------------------------------------------------------------
void ble_connected_handler(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        LOG_ERR("connection err: %d", err);
        return;
    }
    LOG_INF("Connected.");
    current_ble_conn = bt_conn_ref(conn);
    nrf_gpio_pin_clear(BLE_CONNECTED_LED);
}

void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason: %d)", reason);
    nrf_gpio_pin_set(BLE_CONNECTED_LED);
    if (current_ble_conn)
    {
        bt_conn_unref(current_ble_conn);
        current_ble_conn = NULL;
    }
}

void ble_chrc_ccc_cfg_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    ble_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", ble_notif_enabled ? "enabled" : "disabled");
    nrfx_timer_enable(&timer_to_measure_ultrasonic_response_instance);
    nrfx_timer_enable(&timer_to_sample_saadc_via_ppi_instance);
}

void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
    switch (p_event->type)
    {
    case NRFX_SAADC_EVT_LIMIT:
        // here we activate the ECHO_RECEIVE_LIMIT_HIGH only after the signal from the initial ultrasonic pulse (INITIAL_PULSE_LIMIT_LOW) has subsided,
        // after we received an echo we deactivate ECHO_RECEIVE_LIMIT_HIGH to detect only the first echoed signal
        if (p_event->data.limit.channel == LEFT_SENSOR)
        {
            if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_LOW)
            {
                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INT16_MIN, ECHO_RECEIVE_LIMIT_HIGH);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
            }
            else if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_HIGH)
            {
                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INT16_MIN, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
                ultrasonic_response_time_left = nrfx_timer_capture(&timer_to_measure_ultrasonic_response_instance, NRF_TIMER_CC_CHANNEL0);
            }
        }
        else if (p_event->data.limit.channel == RIGHT_SENSOR)
        {
            if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_LOW)
            {
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INT16_MIN, ECHO_RECEIVE_LIMIT_HIGH);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
            }
            else if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_HIGH)
            {
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INT16_MIN, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
                ultrasonic_response_time_right = nrfx_timer_capture(&timer_to_measure_ultrasonic_response_instance, NRF_TIMER_CC_CHANNEL0);
            }
        }
        break;
    case NRFX_SAADC_EVT_DONE:
        nrfx_timer_disable(&timer_to_measure_ultrasonic_response_instance);
        nrfx_timer_disable(&timer_to_sample_saadc_via_ppi_instance); // should be called in NRFX_SAADC_EVT_FINISHED, but that event was never generated,
        is_SAADC_done = true;                                        // perhaps because one dimensional buffer was used instead of double buffer?
        k_event_set(&saadc_done, SAADC_EVENT_BIT);
        break;
    case NRFX_SAADC_EVT_BUF_REQ:
        nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
        NRFX_ERR_CHECK(nrfx_err, " failed");
        break;
    default:
        LOG_INF("SAADC evt %d", p_event->type);
        break;
    }
}

void nfc_handler(void *context,
                 nfc_t2t_event_t event,
                 const uint8_t *data,
                 size_t data_length)
{
    ARG_UNUSED(context);
    ARG_UNUSED(data);
    ARG_UNUSED(data_length);

    switch (event)
    {
    case NFC_T2T_EVENT_DATA_READ:
        nrf_gpio_pin_clear(NFC_FIELD_LED);
        break;
    case NFC_T2T_EVENT_FIELD_OFF:
        nrf_gpio_pin_set(NFC_FIELD_LED);
        break;
    default:
        break;
    }
}

void wdt_handler(void)
{
    // only two 32kHz ticks are left before WDT reset happens
    nrf_gpio_pin_set(OPAMPS_ON_OFF);
    nrf_gpio_pin_clear(RED_LED); // a short red blink can be seen before reset, if WDT is responsible
}

// functions ------------------------------------------------------------------------------------------------------------------------
void timer_init(void)
{
    nrfx_err = nrfx_timer_init(&timer_to_sample_saadc_via_ppi_instance,
                               &timer_config,
                               NULL);
    nrfx_err = nrfx_timer_init(&timer_to_measure_ultrasonic_response_instance,
                               &timer_config,
                               NULL);
    NRFX_ERR_CHECK(nrfx_err, "timer initialization failed");

    nrfx_timer_extended_compare(&timer_to_sample_saadc_via_ppi_instance,
                                NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_us_to_ticks(&timer_to_sample_saadc_via_ppi_instance, SAADC_SAMPLINGRATE_US),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);
}

void ppi_init(void)
{
    nrfx_err = nrfx_ppi_channel_alloc(&ppi_channel_to_sample_saadc_via_timmer);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_assign(ppi_channel_to_sample_saadc_via_timmer,
                                       nrfx_timer_event_address_get(&timer_to_sample_saadc_via_ppi_instance, NRF_TIMER_EVENT_COMPARE0),
                                       nrf_saadc_task_address_get((NRF_SAADC_Type *const)NRF_SAADC_BASE, NRF_SAADC_TASK_SAMPLE));
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_enable(ppi_channel_to_sample_saadc_via_timmer);
    NRFX_ERR_CHECK(nrfx_err, " failed");
}

void saadc_init(void)
{
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
                                            saadc_handler);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC mode failed");

    nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
    NRFX_ERR_CHECK(nrfx_err, "setting SAADC buffer failed failed");

    nrfx_err = nrfx_saadc_mode_trigger();
    NRFX_ERR_CHECK(nrfx_err, "triggering SAADC mode failed");
}

void nfc_init(void)
{
    size_t ndef_msg_buf_len = sizeof(ndef_msg_buf);

    err = nfc_t2t_setup(nfc_handler, NULL);
    ERR_CHECK(err, "Cannot setup NFC T2T library!");

    err = nfc_launchapp_msg_encode(android_pkg_name,
                                   sizeof(android_pkg_name),
                                   NULL,
                                   0,
                                   ndef_msg_buf,
                                   &ndef_msg_buf_len);
    ERR_CHECK(err, "Cannot encode NFC messages!");

    err = nfc_t2t_payload_set(ndef_msg_buf, ndef_msg_buf_len);
    ERR_CHECK(err, "Cannot set NFC payload");

    err = nfc_t2t_emulation_start();
    ERR_CHECK(err, "Cannot start NFC emulation!");

    LOG_INF("NFC configuration done");
}

void wdt_init()
{
    nrfx_err = nrfx_wdt_init(&wdt_instance, &wdt_config_normal, wdt_handler);
    NRFX_ERR_CHECK(nrfx_err, "initializing watchdog failed");

    nrfx_err = nrfx_wdt_channel_alloc(&wdt_instance, &wdt_channel_0);
    NRFX_ERR_CHECK(nrfx_err, "allocationg watchdog channel failed");

    nrfx_wdt_enable(&wdt_instance);
}

// https://devzone.nordicsemi.com/f/nordic-q-a/50415/sending-32-bit-of-data-over-ble-onto-nrf52832
void encode_16bit_to_8bit_array(int16_t *data_array, int8_t *ble_send_array, uint16_t ble_send_array_length)
{
    memcpy(ble_send_array, data_array, ble_send_array_length);
}

void send_sensor_values(int left_right)
{
    if (LEFT_SENSOR == left_right)
    {
        err = bt_gatt_notify(current_ble_conn, &remote_srv.attrs[2], ble_left_sensor_flag, sizeof(ble_left_sensor_flag));
    }
    else if (RIGHT_SENSOR == left_right)
    {
        err = bt_gatt_notify(current_ble_conn, &remote_srv.attrs[2], ble_right_sensor_flag, sizeof(ble_right_sensor_flag));
    }
    else
    {
        LOG_ERR("Wrong sensor selected!");
        return;
    }
    ERR_CHECK(err, "BLE notification failed");

    for (int i = left_right; i < SAADC_BUF_SIZE; i += 2)
    {
        encode_16bit_to_8bit_array(&saadc_samples[i], ble_send_array, 2);
        if (!ble_notif_enabled)
            break;
        err = bt_gatt_notify(current_ble_conn, &remote_srv.attrs[2], ble_send_array, sizeof(ble_send_array));
        ERR_CHECK(err, "BLE notification failed");
    }
}

int16_t get_battery_voltage_mV()
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
    saadc_init();

    // 1.465 results from combining NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_GAIN1_2, NRF_SAADC_REFERENCE_INTERNAL and NRF_SAADC_INPUT_VDDHDIV5
    return (saadc_samples[0] * 1.465);
}

void burst_ultrasonic_pulse_sequence()
{
    nrfx_pwm_simple_playback(&pwm_for_ultrasonic_pulses_instance, &pwm_15plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
}

void turn_opamps_on()
{
    // simply setting the pin is to much inrush current resulting in a SOC reset, so the OpAmps must be slowly toggled on
    k_msleep(100);
    for (int i = 0; i < 1000; i++)
    {
        nrf_gpio_pin_toggle(OPAMPS_ON_OFF);
    }
    nrf_gpio_pin_clear(OPAMPS_ON_OFF);
    k_msleep(100); // take a pause before and after turn on to stablize the LDO
}

void turn_system_off()
{
    LOG_INF("Entering system off.\nApproach a NFC reader to restart.");

    nrf_gpio_pin_set(SYSTEM_ON_LED);
    nrf_gpio_pin_set(OPAMPS_ON_OFF);

    // needed to finish logging before system off
    k_msleep(1);

    // Above we disabled entry to deep sleep based on duration of
    // controlled delay.  Here we need to override that, then
    // force entry to deep sleep (system off) on any delay.
    pm_state_force(0, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

    // Going into sleep will actually go to system off mode, because we
    // forced it above.
    k_msleep(1);

    // k_sleep will never exit, so below two lines will never be executed
    // if system off was correct. On the other hand if someting gone wrong
    // we will see it on terminal and LED.
    nrf_gpio_pin_clear(SYSTEM_ON_LED);
    LOG_ERR("ERROR: System off failed\n");
}

void error_handling()
{
    for (int i = 0; i < 5; i++)
    {
        nrf_gpio_pin_toggle(ERROR_LED);
        k_msleep(200);
    }
    nrf_gpio_pin_set(ERROR_LED);
}

void main(void)
{
    uint32_t events;
    uint8_t system_off_counter = TIME_TO_SYSTEM_OFF_S;

    LOG_INF("Hello World! %s\n", CONFIG_BOARD);

    IRQ_CONNECT(SAADC_IRQn, IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, NULL, 0);

    nrf_gpio_cfg_output(BLUE_LED);
    nrf_gpio_pin_clear(BLUE_LED);

    nrf_gpio_cfg_output(RED_LED);
    nrf_gpio_pin_set(RED_LED);

    nrf_gpio_cfg_output(OPAMPS_ON_OFF);
    nrf_gpio_pin_set(OPAMPS_ON_OFF);

    bt_conn_cb_register(&bluetooth_callbacks);
    err = bt_enable(NULL);
    ERR_CHECK(err, "Cannot enable BLE!");
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    ERR_CHECK(err, "Cannot start advertising");

    wdt_init();
    timer_init();
    ppi_init();
    saadc_init();
    nfc_init();
    nrfx_err = nrfx_pwm_init(&pwm_for_ultrasonic_pulses_instance, &pwm_for_ultrasonic_pulses_peripheral_config, NULL, NULL);
    NRFX_ERR_CHECK(nrfx_err, "Cannot start PWM peripheral")

    turn_opamps_on();

    while (true)
    {
        while (ble_notif_enabled)
        {
            events = k_event_wait(&saadc_done, SAADC_EVENT_BIT, false, K_MSEC(5000)); // this approach consumes 2.5mA vs 5mA when activly polling (without US-Trafos)
            if (events == 0)
            {
                error_handling();
                LOG_ERR("no SAADC converion done!");
            }
            else
            {
                k_event_clear(&saadc_done, SAADC_EVENT_BIT);
                is_SAADC_done = false;
                nrf_gpio_pin_set(SYSTEM_ON_LED);
                ultrasonic_response_time[measurements] = ultrasonic_response_time_left;
                ultrasonic_response_time_left          = UINT16_MAX;
                ++measurements;
                ultrasonic_response_time[measurements] = ultrasonic_response_time_right;
                ultrasonic_response_time_right         = UINT16_MAX;
                ++measurements;

                if (measurements >= MAX_MEASUREMENTS - 1)
                {
                    nrf_gpio_pin_clear(SYSTEM_ON_LED);
                    LOG_INF("SAADC_send");
                    ultrasonic_response_time[measurements] = get_battery_voltage_mV();
                    measurements                           = 0;
                    encode_16bit_to_8bit_array(ultrasonic_response_time, ble_send_array, sizeof(ble_send_array));
                    err = bt_gatt_notify(current_ble_conn, &remote_srv.attrs[2], ble_send_array, sizeof(ble_send_array));
                    ERR_CHECK(err, "BLE notification failed");
                    nrfx_wdt_feed(&wdt_instance);
                }
                burst_ultrasonic_pulse_sequence();
                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INT16_MIN, INT16_MAX);
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INT16_MIN, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
                nrfx_timer_enable(&timer_to_measure_ultrasonic_response_instance);
                nrfx_timer_enable(&timer_to_sample_saadc_via_ppi_instance);
                k_usleep(100); // short delay needed to let voltage rise before setting INITIAL_PULSE_LIMIT_LOW as limit
                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INITIAL_PULSE_LIMIT_LOW, INT16_MAX);
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INITIAL_PULSE_LIMIT_LOW, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
            }
        }

        if (NULL == current_ble_conn)
        {
            --system_off_counter;
            if (0 == system_off_counter)
            {
                turn_system_off();
            }
        }
        else
        {
            system_off_counter = TIME_TO_SYSTEM_OFF_S;
        }

        nrf_gpio_pin_toggle(SYSTEM_ON_LED);
        nrfx_wdt_feed(&wdt_instance);
        k_msleep(1000);
    }
}
