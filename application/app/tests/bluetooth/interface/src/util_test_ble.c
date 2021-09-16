/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_ble.c

#include "util_test_ble.h"

/************************************************
 *  ... STUBS DECLARATION
 ***********************************************/
static int bt_conn_get_info_mock;

void set_bt_conn_info(void)
{
    bt_conn_get_info_mock = 0;
}

void unset_bt_conn_info(void)
{
    bt_conn_get_info_mock = -1;
}

int bt_conn_get_info(const struct bt_conn *conn, struct bt_conn_info *info)
{
    /* Stubs the connexion info */
    info->role = 1;
    info->le.interval = 36;
    info->le.latency = 0;
    info->le.timeout = 42;
    return bt_conn_get_info_mock;
}

void bt_conn_cb_register(struct bt_conn_cb *cb)
{
}

int bt_le_adv_start(const struct bt_le_adv_param *param,
                    const struct bt_data *ad, size_t ad_len,
                    const struct bt_data *sd, size_t sd_len)
{
    return 0;
}

int bt_le_adv_stop(void)
{
    return 0;
}

const bt_addr_le_t *bt_conn_get_dst(const struct bt_conn *conn)
{
    bt_addr_le_t *fake_addr_le_t;
    return fake_addr_le_t;
}

int bt_gatt_exchange_mtu(struct bt_conn *conn,
                         struct bt_gatt_exchange_params *params)
{
    return 0;
}

static int bt_init(void)
{
    return 0;
}

static bt_ready_cb_t ready_cb;
int bt_enable(bt_ready_cb_t cb)
{
    int err;
    ready_cb = cb;

    err = bt_init();
    if (ready_cb)
    {
        ready_cb(err);
    }

    return 0;
}

int auth_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    if (len > 0)
    {
        return 0;
    }
    return -1;
}