/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file authentication_service.h
 * @defgroup zwear_ble_service BLE Services: Z-Wear BLE service definitions
 * @brief Definitions for the Z-Wear BLE services.
 */

#ifndef _AUTHENTICATION_SERVICE_H
#define _AUTHENTICATION_SERVICE_H

/* Standard and Zephyr libraries */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include <init.h>

/* Bluetooth Stack Libraries */
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

/* BLE interface functions */
#include "../ble_interface.h"

/************************************************
 *  ... SERVICE UUIDs
 ***********************************************/
/** @def AUTH_SERVICE_UUID
 *  @brief Authentication service UUID value.
 */
#define AUTH_SERVICE_UUID 0x43, 0x1b, 0x3f, 0xa3, 0x99, 0x66, 0x40, 0x11, 0xb5, 0x98, 0x42, 0x00, 0x18, 0xaa, 0x17, 0x86

/** @def BT_UUID_AUTH_SERVICE
 *  @brief Declare the Authentication service UUID value.
 */
#define BT_UUID_AUTH_SERVICE BT_UUID_DECLARE_128(AUTH_SERVICE_UUID)

/** @def AUTHE_NONCE_UUID
 *  @brief Authentication nonce attribute UUID value.
 */
#define AUTHE_NONCE_UUID 0xf6, 0x9d, 0x7a, 0x7f, 0x2d, 0x30, 0x45, 0x27, 0xbe, 0xcd, 0x5c, 0xdb, 0x71, 0x87, 0x0c, 0xcb

/** @def BT_UUID_AUTH_NON
 *  @brief Declare the authentication nonce UUID value.
 */
#define BT_UUID_AUTH_NON BT_UUID_DECLARE_128(AUTHE_NONCE_UUID)

/************************************************
 *  ... SERVICE SEND FUNCTION
 ***********************************************/

/**
 * @brief Sends a characteristic update for authentication attribute.
 * @param[in] conn bt_conn structure for the current connection.
 * @param[in] data pointer to the data that needs to be sent (in uint8_t).
 * @param[in] len size of the data that needs to be sent.
 */
int auth_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);

#endif /* _AUTHENTICATION_SERVICE_H */