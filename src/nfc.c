#include "nfc.h"

#define MAX_REC_COUNT     1
#define NDEF_MSG_BUF_SIZE 512

LOG_MODULE_REGISTER(nfc);

const uint8_t android_pkg_name[] = {
    'a', 'p', 'p', 'i', 'n', 'v', 'e', 'n', 't', 'o', 'r', '.', 'a', 'i', '_', 'c', 'o', 'r', 'v', 'i', 'n', 'l', 'o', 's', 's', 'i', 'n', '.', 'O', 'B', 'S', 'm', 'i', 'n', 'i', '_', 'A', 'D', 'C'};
// 'c', 'o', 'm', '.', 'e', 'x', 'a', 'm', 'p', 'l', 'e', '.', 'o', 'b', 's', 'm', 'i', 'n', 'i'};

static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

void nfc_init(nfc_t2t_callback_t nfc_cb_handler)
{
    size_t ndef_msg_buf_len = sizeof(ndef_msg_buf);

    err = nfc_t2t_setup(nfc_cb_handler, NULL);
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