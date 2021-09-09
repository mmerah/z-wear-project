/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_services.c

#include "util_test_services.h"

/*******************************************************************************
 *** MOCK FUNCTIONS UTILITIES
 *******************************************************************************/

static bool notif_enabled;
static int notify_param;
static int indicate_enabled;

void enable_notify(void)
{
    notif_enabled = true;
}
void disable_notify(void)
{
    notif_enabled = false;
}

void enable_param(void)
{
    notify_param = 1;
}
void disable_param(void)
{
    notify_param = 0;
}

void enable_indicate(void)
{
    indicate_enabled = 0;
}

void disable_indicate(void)
{
    indicate_enabled = -1;
}

/*******************************************************************************
 *** FAKE
 *******************************************************************************/

struct bt_conn* connection_get(void)
{
    struct bt_conn *fakeconnect;
    return fakeconnect;
}

/*******************************************************************************
 *** STUBS
 *******************************************************************************/

ssize_t bt_gatt_attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t buf_len, uint16_t offset,
			  const void *value, uint16_t value_len)
{
   return buf_len;
}

ssize_t bt_gatt_attr_read_service(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
   return 0;
}

ssize_t bt_gatt_attr_read_chrc(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
    return 0;
}

ssize_t bt_gatt_attr_read_ccc(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
    return 0;
}

ssize_t bt_gatt_attr_write_ccc(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, const void *buf,
			       uint16_t len, uint16_t offset, uint8_t flags)
{
    return 0;
}

void hmac_compute(const char *host_challenge, uint16_t length_challenge, uint8_t *result)
{
    return 0;
}

/*******************************************************************************
 *** MOCKS
 *******************************************************************************/
bool bt_gatt_is_subscribed(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, uint16_t ccc_value)
{
    return notif_enabled;
    
}

int bt_gatt_notify_cb(struct bt_conn *conn,
		      struct bt_gatt_notify_params *params)
{
    if (notify_param > 0) {
        return 0;
    } else {
        return -1;
    }
}

int bt_gatt_indicate(struct bt_conn *conn,
		     struct bt_gatt_indicate_params *params)
{
    return indicate_enabled;
}