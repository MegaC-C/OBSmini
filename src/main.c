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
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

#include <hal/nrf_power.h>

#include "ble.h"
#include "error_handling.h"
#include "global_config.h"
#include "nfc.h"
#include "pulse_generator.h"
#include "saadc.h"
#include "timer_and_ppi.h"
#include "watchdog.h"

#define SAADC_EVENT_DONE BIT(0)

// forward declarations	------------------------------------------------------------------------------------------------------------------------
void ble_connected_handler(struct bt_conn *conn, uint8_t err);
void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason);

struct bt_conn *current_ble_conn;
struct bt_conn_cb bluetooth_handlers = {
    .connected    = ble_connected_handler,
    .disconnected = ble_disconnected_handler};

// global variables	------------------------------------------------------------------------------------------------------------------------
bool ble_notifications_enabled      = false;
uint8_t measurements                = 1;
uint16_t ultrasonic_echo_time_left  = UINT16_MAX;
uint16_t ultrasonic_echo_time_right = UINT16_MAX;
uint16_t ultrasonic_echo_times_us[MAX_MEASUREMENTS];

// pre kernel initialization ------------------------------------------------------------------------------------------------------------------------
LOG_MODULE_REGISTER(main);
K_EVENT_DEFINE(saadc_done);

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
}

void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason: %d)", reason);
    if (current_ble_conn)
    {
        bt_conn_unref(current_ble_conn);
        current_ble_conn = NULL;
    }
}

void ble_notification_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    ble_notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", ble_notifications_enabled ? "enabled" : "disabled");
    if (ble_notifications_enabled)
    {
        timer_start();
    }
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
                ultrasonic_echo_time_left = get_echo_time_us();
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
                ultrasonic_echo_time_right = get_echo_time_us();
            }
        }
        break;
    case NRFX_SAADC_EVT_DONE:
        timer_stop(); // should be called in NRFX_SAADC_EVT_FINISHED, but that event was never generated,
                      // perhaps because one dimensional buffer was used instead of double buffer?
        k_event_set(&saadc_done, SAADC_EVENT_DONE);
        break;
    case NRFX_SAADC_EVT_BUF_REQ:
        saadc_buffer_set();
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
    switch (event)
    {
    case NFC_T2T_EVENT_DATA_READ:
        nrf_gpio_pin_clear(RED_LED);
        break;
    case NFC_T2T_EVENT_FIELD_OFF:
        nrf_gpio_pin_set(RED_LED);
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
    LOG_INF("Entering system off. Approach a NFC reader to restart.");

    nrf_gpio_pin_set(BLUE_LED);
    nrf_gpio_pin_set(RED_LED);
    nrf_gpio_pin_set(OPAMPS_ON_OFF);

    // needed to finish turn off before system off
    k_msleep(1);

    // Above we disabled entry to deep sleep based on duration of
    // controlled delay.  Here we need to override that, then
    // force entry to deep sleep (system off) on any delay.
    pm_state_force(0, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

    // Going into sleep will actually go to system off mode, because we
    // forced it above.
    k_sleep(K_FOREVER);

    // k_sleep will never exit, so below two lines will never be executed
    // if system off was correct. On the other hand if someting gone wrong
    // we will see it on terminal and LED.
    nrf_gpio_pin_clear(BLUE_LED);
    LOG_ERR("ERROR: System off failed");
}

void main(void)
{
    uint32_t events;
    uint16_t system_off_counter = TIME_TO_SYSTEM_OFF_S;

    LOG_INF("Hello OBSmini! Config = %s", CONFIG_BOARD);

    nrf_gpio_cfg_output(BLUE_LED);
    nrf_gpio_pin_clear(BLUE_LED); // => ON

    nrf_gpio_cfg_output(RED_LED);
    nrf_gpio_pin_set(RED_LED); // => OFF

    nrf_gpio_cfg_output(OPAMPS_ON_OFF);
    nrf_gpio_pin_set(OPAMPS_ON_OFF); // => OFF

    watchdog_init(wdt_handler);
    saadc_init(saadc_handler);
    timer_and_ppi_init();
    nfc_init(nfc_handler);
    pulse_generator_init();
    ble_init(&bluetooth_handlers);

    turn_opamps_on();

    while (true)
    {
        while (ble_notifications_enabled)
        {
            events = k_event_wait(&saadc_done, SAADC_EVENT_DONE, false, K_MSEC(5000)); // this approach consumes 2.5mA vs 5mA when activly polling (without US-Trafos)
            if (events == 0)
            {
                error_handling();
                LOG_ERR("no SAADC converion done!");
            }
            else
            {
                nrf_gpio_pin_set(BLUE_LED);

                ultrasonic_echo_times_us[measurements] = ultrasonic_echo_time_left;
                ultrasonic_echo_time_left              = UINT16_MAX;
                ++measurements;
                ultrasonic_echo_times_us[measurements] = ultrasonic_echo_time_right;
                ultrasonic_echo_time_right             = UINT16_MAX;
                ++measurements;

                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INT16_MIN, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INT16_MIN, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");

                if (measurements >= MAX_MEASUREMENTS)
                {
                    nrf_gpio_pin_clear(BLUE_LED);
                    LOG_INF("SAADC_send");
                    ultrasonic_echo_times_us[0] = get_battery_voltage_mV(saadc_handler);
                    measurements                = 1;
                    ble_send(current_ble_conn, ultrasonic_echo_times_us, sizeof(ultrasonic_echo_times_us));
                    watchdog_feed();
                }
                burst_ultrasonic_pulse_sequence();
                timer_start();
                k_usleep(100); // short delay needed to let voltage rise before setting INITIAL_PULSE_LIMIT_LOW as limit
                nrfx_err = nrfx_saadc_limits_set(LEFT_SENSOR, INITIAL_PULSE_LIMIT_LOW, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");
                nrfx_err = nrfx_saadc_limits_set(RIGHT_SENSOR, INITIAL_PULSE_LIMIT_LOW, INT16_MAX);
                NRFX_ERR_CHECK(nrfx_err, "setting SAADC comperator limits failed");

                k_event_clear(&saadc_done, SAADC_EVENT_DONE);
            }
        }

        if (current_ble_conn == NULL)
        {
            --system_off_counter;

            if (system_off_counter == 0)
            {
                turn_system_off();
            }
        }
        else
        {
            system_off_counter = TIME_TO_SYSTEM_OFF_S;
        }

        nrf_gpio_pin_toggle(BLUE_LED);
        watchdog_feed();
        k_msleep(1000);
    }
}
