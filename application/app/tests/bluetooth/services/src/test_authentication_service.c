/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file test_authentication_service.c

/**
 * @addtogroup ble_services_tests
 * @{
 */

/* ZTEST library */
#include <ztest.h>
#include <zephyr.h>
#include <stdint.h>

/* Fake functions */
#include "util_test_services.h"

/* Code tested */
#include "../../../../src/ble/services/authentication_service.c"
#include "../../../../src/ble/services/authentication_service.h"

/* FAKE CONNEXION */
static struct bt_conn *fakeconnexion;

/************************************************
 *  ...FAKE
 ***********************************************/
int get_secret_key(char stored_key[16])
{
	stored_key[16] = "aXs6h5cE9h0mDHr";
	return 0;
}

/************************************************
 *  ...TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test Authentication attribute notification send
 * @details test auth_service_send() function.
 * @see auth_service_send()
 */
void test_auth_service_send(void)
{
	uint8_t fakedata = 0;
	uint16_t fakelen = 1;
	enable_notify();
	enable_param();
	zassert_ok(auth_service_send(fakeconnexion, &fakedata, fakelen), 
                    "Auth notification not sent");
}

/**
 * @brief Test Authentication attribute notification send fail
 * @details test auth_service_send() function.
 * @see auth_service_send()
 */
void test_auth_service_send_bad_param(void)
{
	uint8_t fakedata = 0;
	uint16_t fakelen = 1;
	enable_notify();
	disable_param();
	zassert_equal(auth_service_send(fakeconnexion, &fakedata, fakelen), -EINVAL, 
                    "Auth notification sent but should not");
}

/**
 * @brief Test Authentication attribute notification no subscriber
 * @details test auth_service_send() function.
 * @see auth_service_send()
 */
void test_auth_service_send_not_sub(void)
{
	uint8_t fakedata = 0;
	uint16_t fakelen = 1;
	disable_notify();
	zassert_equal(auth_service_send(fakeconnexion, &fakedata, fakelen), -ENOTCONN, 
                    "Auth notification subscribed but should not");
}

/**
 * @brief Test reception of authentication challenge
 * @details test on_receive_auth() callback function.
 * @see on_receive_auth()
 */
void test_on_auth_received(void)
{
	const struct bt_gatt_attr *fakeattr;
	const uint8_t buf[2] = {0, 1};
	uint16_t fakelen = 2;
	uint16_t fakeoffset = 0;
	uint8_t fakeflags = 0;

	zassert_equal(on_receive_auth(fakeconnexion, fakeattr,
						buf, fakelen, fakeoffset, fakeflags), fakelen, "Auth received callback problem");
}

/**
 * @brief Test CCCD Authentication Notification enabled
 * @details test auth_ccc_cfg_changed() callback function.
 * @see auth_ccc_cfg_changed()
 */
void test_cccd_changed_auth(void)
{
	const struct bt_gatt_attr *fakeattr;
	uint16_t fakevalue = BT_GATT_CCC_NOTIFY;
	auth_ccc_cfg_changed(fakeattr, fakevalue);
	zassert_equal(0, 0, "Auth notification was not enabled");
}

/**
 * @brief Test CCCD Authentication Notification disabled
 * @details test auth_ccc_cfg_changed() callback function.
 * @see auth_ccc_cfg_changed()
 */
void test_cccd_changed_auth_disable(void)
{
	const struct bt_gatt_attr *fakeattr;
	uint16_t fakevalue = 0x0000;
	auth_ccc_cfg_changed(fakeattr, fakevalue);
	zassert_equal(0, 0, "Auth notification was not disabled");
}

/**
 * @}
 */