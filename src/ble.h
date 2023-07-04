#ifndef BLE_H
#define BLE_H

#include "error_handling.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#define BT_UUID_OBS_SERVICE_VAL        BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_OBS_SERVICE            BT_UUID_DECLARE_128(BT_UUID_OBS_SERVICE_VAL)
#define BT_UUID_OBS_DISTANCES_CHRC_VAL BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_OBS_DISTANCES_CHRC     BT_UUID_DECLARE_128(BT_UUID_OBS_DISTANCES_CHRC_VAL)

void ble_init(struct bt_conn_cb *bt_cb);
void ble_send(struct bt_conn *ble_conn, int16_t *data_array, uint16_t data_array_length);

#endif