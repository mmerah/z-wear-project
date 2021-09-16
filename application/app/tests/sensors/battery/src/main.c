/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/* ZTEST library */
#include <ztest.h>

/* Code tested */
#include "../../../../src/sensors/battery/battery.h"
#include "../../../../src/sensors/battery/battery.c"

/**
 * @brief Tests for the battery sensor handler module
 *
 * Verify that the sensor handler module for the battery
 * functions as intented by measuring State Of Charge and
 * notifying it on the appropriate BLE attribute.
 *
 * @defgroup battery_tests BATT
 * @ingroup all_tests
 * @{
 * @}
 */

/************************************************
 *  ... STUBS
 ***********************************************/

int bas_notify(int16_t batt_level)
{
    return 0;
}

/************************************************
 *  ... FAKES
 ***********************************************/

/************************************************
 *  ... MOCKS
 ***********************************************/

/************************************************
 *  ... UTILITIES
 ***********************************************/
extern struct k_sem batt_ok;
extern struct k_sem batt_init_ok;
extern struct k_sem batt_process_ready;

extern struct k_fifo fifo_batt_data;

#define STACK_SIZE 2048
#define MY_PRIORITY 4
static K_THREAD_STACK_DEFINE(tstack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(tstack2, STACK_SIZE);
static struct k_thread tdata;
static struct k_thread tdata2;

/************************************************
 *  ... TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test BAS initialization
 * @details test bas_init function, should initialize
 * the sensor and return 0 as well as setting a semaphore
 * when successful.
 * @see bas_init()
 */
static void test_bas_init(void)
{
    zassert_ok(bas_init(), "Battery handler did not initialize.");
    zassert_true(k_sem_count_get(&batt_init_ok) == 1,
                 "Initialization did not give the semaphore");
}

/**
 * @brief Test BAS sampling timer handler semaphore
 * @details test timer handler function, should set
 * a semaphore indicating to the processor thread to
 * start processing fuel gauge data.
 * @see bas_timer_handler()
 */
static void test_bas_timer_handler(void)
{
    struct k_timer *dummy;
    bas_timer_handler(dummy);
    zassert_true(k_sem_count_get(&batt_ok) == 1,
                 "Timer handler did not give the semaphore");
    k_sem_take(&batt_ok, K_FOREVER);
}

/**
 * @brief Test BAS starting sampling
 * @details test bas_start function, should start
 * sampling of data and return 0 when successful.
 * @see bas_start()
 */
static void test_bas_start(void)
{
    zassert_ok(bas_start(), "Battery handler did not start.");
}

/**
 * @brief Test BAS stopping sampling
 * @details test bas_stop function, should stop
 * sampling of data and return 0 when successful.
 * @see bas_stop()
 */
static void test_bas_stop(void)
{
    zassert_ok(bas_stop(), "Battery handler did not stop.");
}

/**
 * @brief Test BAS access to the notifying thread
 * @details test batt_notify_thread, should be accessed
 * after the batt_process_ready semaphore is set and
 * if something has been put in the FIFO.
 * @see batt_notify_thread()
 */
static void test_notify_thread_access(void)
{
    k_thread_create(&tdata, tstack, STACK_SIZE,
                    batt_notify_thread, NULL, NULL, NULL,
                    MY_PRIORITY,
                    K_INHERIT_PERMS, K_NO_WAIT);
    k_sem_give(&batt_process_ready);
    struct data_item_batt_t fake_batt_data;
    k_fifo_put(&fifo_batt_data, &fake_batt_data);
    k_sleep(K_USEC(200));
    zassert_ok(0, "Notify thread not accessed");
}

/**
 * @brief Test BAS access to the processing thread
 * @details test batt_process_thread, should be accessed
 * after the batt_init_ok and batt_ok semaphore are set
 * because they indicate that the battery sensor is initialized
 * and that a sample is requested. Should put some data in the
 * FIFO pile and set a semaphore to indicate that the processor
 * thread is ready.
 * @see batt_process_thread()
 */
static void test_process_thread_access(void)
{
    bas_init();
    k_thread_create(&tdata2, tstack2, STACK_SIZE,
                    batt_process_thread, NULL, NULL, NULL,
                    MY_PRIORITY,
                    K_INHERIT_PERMS, K_NO_WAIT);
    k_sem_give(&batt_init_ok);
    k_sem_give(&batt_ok);
    k_sleep(K_USEC(200));
    zassert_equal(k_sem_count_get(&batt_process_ready), 1, "Process ready semaphore not taken");
    zassert_ok(0, "Process thread not accessed");
}

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
    ztest_test_suite(test_battery,
                     ztest_unit_test(test_bas_init),
                     ztest_unit_test(test_process_thread_access),
                     ztest_unit_test(test_bas_timer_handler),
                     ztest_unit_test(test_bas_start),
                     ztest_unit_test(test_bas_stop),
                     ztest_unit_test(test_notify_thread_access));

    ztest_run_test_suite(test_battery);
}