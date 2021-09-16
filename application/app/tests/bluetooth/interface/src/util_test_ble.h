/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_ble.h

#ifndef _UTIL_TEST_BLE_H
#define _UTIL_TEST_BLE_H

/* Standard and Zephyr libraries */
#include <zephyr.h>
#include <stdint.h>

/* Code tested */
#include "../../../../src/ble/ble_interface.h"

/************************************************
 *  ... FAKE
 ***********************************************/
/**
 * @brief Activate bt_conn_info function
 */
void set_bt_conn_info(void);
/**
 * @brief Deactivate bt_conn_info function
 */
void unset_bt_conn_info(void);

/** @brief Get connection info
 *
 *  @param conn Connection object.
 *  @param info Connection info object.
 *
 *  @return Zero on success or (negative) error code on failure.
 */
int bt_conn_get_info(const struct bt_conn *conn, struct bt_conn_info *info);

/** @brief Register connection callbacks.
 *
 *  Register callbacks to monitor the state of connections.
 *
 *  @param cb Callback struct. Must point to memory that remains valid.
 */
void bt_conn_cb_register(struct bt_conn_cb *cb);

/**
 * @brief Start advertising
 *
 * Set advertisement data, scan response data, advertisement parameters
 * and start advertising.
 *
 * When the advertisement parameter peer address has been set the advertising
 * will be directed to the peer. In this case advertisement data and scan
 * response data parameters are ignored. If the mode is high duty cycle
 * the timeout will be @ref BT_GAP_ADV_HIGH_DUTY_CYCLE_MAX_TIMEOUT.
 *
 * @param param Advertising parameters.
 * @param ad Data to be used in advertisement packets.
 * @param ad_len Number of elements in ad
 * @param sd Data to be used in scan response packets.
 * @param sd_len Number of elements in sd
 *
 * @return Zero on success or (negative) error code otherwise.
 * @return -ENOMEM No free connection objects available for connectable
 *                 advertiser.
 * @return -ECONNREFUSED When connectable advertising is requested and there
 *                       is already maximum number of connections established
 *                       in the controller.
 *                       This error code is only guaranteed when using Zephyr
 *                       controller, for other controllers code returned in
 *                       this case may be -EIO.
 */
int bt_le_adv_start(const struct bt_le_adv_param *param,
                    const struct bt_data *ad, size_t ad_len,
                    const struct bt_data *sd, size_t sd_len);

/**
 * @brief Stop advertising.
 * 
 * Stops ongoing advertising.
 * 
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_le_adv_stop(void);

/** @brief Get destination (peer) address of a connection.
 *
 *  @param conn Connection object.
 *
 *  @return Destination address.
 */
const bt_addr_le_t *bt_conn_get_dst(const struct bt_conn *conn);

/** @brief Exchange MTU
 *
 *  This client procedure can be used to set the MTU to the maximum possible
 *  size the buffers can hold.
 *
 *  @note Shall only be used once per connection.
 *
 *  @param conn Connection object.
 *  @param params Exchange MTU parameters.
 *
 *  @return 0 in case of success or negative value in case of error.
 */
int bt_gatt_exchange_mtu(struct bt_conn *conn,
                         struct bt_gatt_exchange_params *params);

/**
 * @brief Enable Bluetooth
 *
 * Enable Bluetooth. Must be the called before any calls that
 * require communication with the local Bluetooth hardware.
 *
 * @param cb Callback to notify completion or NULL to perform the
 * enabling synchronously.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int bt_enable(bt_ready_cb_t cb);

/************************************************
 *  ... FAKE
 ***********************************************/

int auth_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);

#endif /* _UTIL_TEST_BLE_H */