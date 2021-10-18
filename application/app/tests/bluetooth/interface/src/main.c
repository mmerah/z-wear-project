/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/**
 * @brief Tests for the BLE interface module
 *
 * Verify that the BLE interface module function as
 * expected.
 *
 * @defgroup ble_tests BLE
 * @ingroup all_tests
 * @{
 * @}
 */

/* ZTEST library */
#include <ztest.h>
#include <zephyr.h>
#include <stdint.h>

/* Code tested */
#include "../../../../src/ble/ble_interface.h"
#include "../../../../src/ble/ble_interface.c"

/************************************************
 *  ... STUBS
 ***********************************************/
#define SYS_REBOOT_COLD 1
void sys_reboot(int type) {}

int bt_conn_set_security(struct bt_conn *conn, bt_security_t sec)
{
    return 0;
}

/************************************************
 *  ... FAKES
 ***********************************************/
/* FAKE SEMAPHORES */
K_SEM_DEFINE(ble_conn, 0, 1);
K_SEM_DEFINE(ble_deconn, 0, 1);
K_SEM_DEFINE(ble_init_c, 0, 1);

/* Faked functions */
#include "util_test_ble.h"

/* FAKE CONNEXION */
struct bt_conn *fakeconnexion;

extern struct k_sem ble_timeout;

/************************************************
 *  ... MOCKS
 ***********************************************/

/************************************************
 *  ... UTILITIES
 ***********************************************/
#define STACK_SIZE 2048
#define MY_PRIORITY 4
static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static struct k_thread tdata;

/************************************************
 *  ... TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test BLE layer initialization
 * @details test ble_layer_init function
 * @see ble_layer_init()
 */
static void test_ble_layer_init(void)
{
    zassert_ok(ble_layer_init(), "BLE Layer did not initialize.");
}

/**
 * @brief Test BLE layer pause advertising
 * @details test ble_layer_pause function
 * @see ble_layer_pause()
 */
static void test_ble_layer_pause(void)
{
    zassert_ok(ble_layer_pause(), "BLE Layer did not pause.");
}

/**
 * @brief Test BLE layer resume advertising
 * @details test ble_layer_resume function
 * @see ble_layer_resume()
 */
static void test_ble_layer_resume(void)
{
    zassert_ok(ble_layer_resume(), "BLE Layer did not pause.");
}

/**
 * @brief Test BLE connection semaphore
 * @details test connected callback function
 * @see connected()
 */
static void test_ble_connection(void)
{
    set_bt_conn_info();
    connected(fakeconnexion, 0);
    zassert_equal(k_sem_count_get(&ble_conn), 1, "BLE connection semaphore not given");
    int err = k_sem_take(&ble_conn, K_MSEC(0));
    if (!err)
    {
        zassert_equal(k_sem_count_get(&ble_conn), 0, "BLE connection semaphore count wrong");
    }
    else
    {
        zassert_unreachable("BLE connection semaphore was not taken");
    }
}

/**
 * @brief Test BLE connection no bt_conn_info
 * @details test connected callback function
 * @see connected()
 */
static void test_ble_connection_noinfo(void)
{
    unset_bt_conn_info();
    connected(fakeconnexion, 0);
    zassert_equal(k_sem_count_get(&ble_conn), 1, "BLE connection semaphore not given");
    int err = k_sem_take(&ble_conn, K_MSEC(0));
    if (!err)
    {
        zassert_equal(k_sem_count_get(&ble_conn), 0, "BLE connection semaphore count wrong");
    }
    else
    {
        zassert_unreachable("BLE connection semaphore was not taken");
    }
}

/**
 * @brief Test BLE connection failing
 * @details test connected callback function
 * @see connected()
 */
static void test_ble_connection_failed(void)
{
    set_bt_conn_info();
    connected(fakeconnexion, 1);
    int err = k_sem_take(&ble_conn, K_MSEC(0));
    if (!err)
    {
        zassert_unreachable("BLE connection semaphore was not taken");
    }
    else
    {
        zassert_equal(k_sem_count_get(&ble_conn), 0, "BLE connection semaphore count wrong");
    }
}

/**
 * @brief Test BLE deconnection semaphore
 * @details test disconnected callback function
 * @see disconnected()
 */
static void test_ble_deconnection(void)
{
    disconnected(fakeconnexion, 0);
    zassert_equal(k_sem_count_get(&ble_deconn), 1, "BLE disconnected semaphore not given");
    int err = k_sem_take(&ble_deconn, K_MSEC(0));
    if (!err)
    {
        zassert_equal(k_sem_count_get(&ble_deconn), 0, "BLE disconnected semaphore count wrong");
    }
    else
    {
        zassert_unreachable("BLE disconnected semaphore was not taken");
    }
}

/**
 * @brief Test BLE MTU exchange
 * @details test exchange_func function
 * @see exchange_func()
 */
static void test_exchange_mtu(void)
{
    struct bt_gatt_exchange_params *param;
    set_bt_conn_info();
    exchange_func(fakeconnexion, 0, param);
    zassert_ok(0, "Stuck in exchange function");

    unset_bt_conn_info();
    exchange_func(fakeconnexion, 0, param);
    zassert_ok(0, "Stuck in exchange function error handling");
}

/**
 * @brief Test BLE Timeout timer
 * @details test ble_timeout_timer_handler function
 * @see ble_timeout_timer_handler()
 */
static void test_ble_timeout_timer_handler(void)
{
    struct k_timer *dummy;
    ble_timeout_timer_handler(dummy);
    zassert_true(k_sem_count_get(&ble_timeout) == 1, "Ble timeout semaphore not given.");
}

/**
 * @brief Test BLE Starting timeout
 * @details test ble_timeout_start function
 * @see ble_timeout_start()
 */
static void test_ble_timeout_start(void)
{
    zassert_ok(ble_timeout_start(), "Could not start the ble timeout subsystem");
}

/**
 * @brief Test BLE Timeout handler thread
 */
/**
 * @brief Test BLE Timeout handler thread
 * @details test ble_handler thread access
 * @see ble_handler()
 */
static void test_ble_handler(void)
{
    k_sem_give(&ble_timeout);
    k_thread_create(&tdata, tstack, STACK_SIZE,
                    ble_handler, NULL, NULL, NULL,
                    MY_PRIORITY,
                    K_INHERIT_PERMS, K_NO_WAIT);
    k_sleep(K_USEC(200));
    zassert_true(k_sem_count_get(&ble_timeout) == 0, "Ble timeout handler was not executed");
}

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
    ztest_test_suite(test_interface,
                     ztest_unit_test(test_ble_layer_init),
                     ztest_unit_test(test_ble_layer_pause),
                     ztest_unit_test(test_ble_layer_resume),
                     ztest_unit_test(test_ble_connection),
                     ztest_unit_test(test_ble_connection_noinfo),
                     ztest_unit_test(test_ble_connection_failed),
                     ztest_unit_test(test_ble_deconnection),
                     ztest_unit_test(test_exchange_mtu),
                     ztest_unit_test(test_ble_timeout_timer_handler),
                     ztest_unit_test(test_ble_timeout_start),
                     ztest_unit_test(test_ble_handler));

    ztest_run_test_suite(test_interface);
}
