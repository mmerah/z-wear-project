/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file battery.h
 * @defgroup battery Sensor Handler: Battery fuel gauge 
 * handler functions and threads.
 * @brief Module for the handling of the fuel gauge.
 */

#ifndef _BAS_HANDLER_H
#define _BAS_HANDLER_H

/* Standard and Zephyr libraries */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>

/* Sensor interface functions */
#include "../sensor_interface.h"

/** Battery fuel gauge sampling in ms */
#define BAS_SAMPLING_TIME_MS 5000
/** Stacksize for the fuel gauge processing thread */
#define STACKSIZE_BAS_PROCESS 600
/** Stacksize for the fuel gauge notifying thread */
#define STACKSIZE_BAS_NOTIFY 600
/** Priority for the fuel gauge processing thread */
#define PRIORITY_BAS_PROCESS 9
/** Priority for the fuel gauge notifying thread */
#define PRIORITY_BAS_NOTIFY 10

/**
 * @brief Start the battery sampling sensor sampling.
 * @return 0 on success.
 */
int bas_init(void);

/**
 * @brief Stops the battery sampling sensor sampling.
 * @return 0 on success.
 */
int bas_stop(void);

/**
 * @brief Restarts the battery sampling sensor sampling.
 * @return 0 on success.
 */
int bas_start(void);

#endif /* _BAS_HANDLER_H */