#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include "global_config.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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

extern int nrfx_err;
extern int err;

void error_handling();

#endif