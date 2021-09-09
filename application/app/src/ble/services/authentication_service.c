/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file authentication_service.c
 * @addtogroup zwear_ble_service
 * @{
 */

#include "authentication_service.h"

/* Authentication function */
#include "../authentication/authentication.h"

/* Settings storage functions */
#include "../../settings/settings_storage.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(authentication_service, CONFIG_BT_BAS_LOG_LEVEL);

/************************************************
 *  ... RECEIVE CALLBACK
 ***********************************************/

/**
 * This callback function is called when the authentication nounce attribute 
 * receives data from the client. It then does the HMAC-SHA256 compute and 
 * sends back the result of the challenge.
 */
static ssize_t on_receive_auth(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags)
{
    const char * buffer = buf;

    /* Buffer is the challenge data */
    uint8_t sha_256_result[32];
    /* Initialize the secret */
    char secret[16];
    get_secret_key(secret);
    secret[15] = '\0';
    /* Compute the MAC and send it for notification */
    hmac_compute(buffer, secret, len, sha_256_result);
    auth_service_send(connection_get(), sha_256_result, sizeof(sha_256_result));
	return len;
}

/************************************************
 *  ... CCCD CALLBACK
 ***********************************************/

/**
 * This callback function is called when the client subscribes or unsubscribes
 * from the notifications.
 */
static void auth_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Authentication Nonce notifications %s", notif_enabled ? "enabled" : "disabled");
}

/************************************************
 *  ... SERVICE DEFINITION
 ***********************************************/
BT_GATT_SERVICE_DEFINE(auth_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_AUTH_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUTH_NON, 
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
			        BT_GATT_PERM_WRITE_ENCRYPT, 
                    NULL, on_receive_auth, NULL),
    BT_GATT_CCC(auth_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/************************************************
 *  ... SERVICE SEND FUNCTION
 ***********************************************/
int auth_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &auth_svc.attrs[1]; 
    struct bt_gatt_notify_params params = {
        .uuid   = BT_UUID_AUTH_NON,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    /* Check whether notifications are enabled or not */
    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
        /* Send the notification */
	    if(bt_gatt_notify_cb(conn, &params)){
            LOG_ERR("Error, unable to send Auth notification\n");
            return -EINVAL;
        }
        return 0;
    }
    else{
        LOG_ERR("Warning, notification not enabled on the selected attribute\n");
        return -ENOTCONN;
    }
}

/** @} */