#ifndef NFC_H
#define NFC_H

#include "error_handling.h"
#include <nfc/ndef/launchapp_msg.h>
#include <nfc_t2t_lib.h>
#include <zephyr/logging/log.h>

void nfc_init(nfc_t2t_callback_t nfc_cb_handler);

#endif