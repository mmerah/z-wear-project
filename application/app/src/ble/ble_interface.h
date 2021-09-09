/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file ble_interface.h
 * @defgroup ble_interface BLE Layer: Z-Wear BLE interface functions
 * @brief Various definitions for the Z-Wear application to interface to the BLE layer.
 */

#ifndef _BLE_INTERFACE_H
#define _BLE_INTERFACE_H

/* Standard and Zephyr libraries */
#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>
#include <sys/byteorder.h>
#include <sys/util.h>

/* Bluetooth Stack Libraries */
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

/* Settings library for DIS */
#include <settings/settings.h>

/* Versionning header */
#include "../version.h"

/** Device Name as defined in the prj.conf */
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
/** device name length */
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

/** Stacksize for the BLE timeout handler */
#define STACKSIZE_BLE_HANDLER     256
/** Priority for the BLE timeout handler */
#define PRIORITY_BLE_HANDLER      2

/**
 * @brief Initialize the BLE layer.
 * 
 * Starts all device services and begins advertising.
 * Also sets up all callbacks for connection, disconnection
 * and LE parameters updates/requires.
 * 
 * @return Error code. 0 if successful. The rest is not yet implemented.
 */
int ble_layer_init(void);

/**
 * @brief Pause the BLE layer.
 * 
 * @return Error code. 0 if successful, rest is not yet implemented.
 */
int ble_layer_pause(void);

/**
 * @brief Resume BLE layer operations.
 * 
 * @return Error code. 0 if successful, rest is not yet implemented.
 */
int ble_layer_resume(void);

/**
 * @brief Starts the timeout for BLE layer.
 * 
 * @return Zero on success.
 */
int ble_timeout_start(void);

/**
 * @brief Returns the current ble connection
 * 
 * @return bt_conn structure.
 */
struct bt_conn* connection_get(void);

 #endif /* _BLE_INTERFACE_H */