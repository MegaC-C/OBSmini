#include "ble.h"

#define BLE_DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define BLE_DEVICE_NAME_LEN (sizeof(BLE_DEVICE_NAME) - 1)

LOG_MODULE_REGISTER(ble);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN)};

void ble_notification_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);

BT_GATT_SERVICE_DEFINE(remote_srv,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_OBS_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_OBS_DISTANCES_CHRC,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              NULL, NULL, NULL),
                       BT_GATT_CCC(ble_notification_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

void ble_init(struct bt_conn_cb *bt_cb)
{
    LOG_INF("Initializing BLE...");

    bt_conn_cb_register(bt_cb);
    err = bt_enable(NULL);
    ERR_CHECK(err, "Cannot enable BLE!");
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    ERR_CHECK(err, "Cannot start advertising");
}

void ble_send(struct bt_conn *ble_conn, int16_t *data_array, uint16_t data_array_length)
{
    int8_t ble_send_buffer[data_array_length];

    // https://devzone.nordicsemi.com/f/nordic-q-a/50415/sending-32-bit-of-data-over-ble-onto-nrf52832
    memcpy(ble_send_buffer, data_array, sizeof(ble_send_buffer));

    err = bt_gatt_notify(ble_conn, &remote_srv.attrs[2], ble_send_buffer, sizeof(ble_send_buffer));
    ERR_CHECK(err, "BLE notification failed");
}
