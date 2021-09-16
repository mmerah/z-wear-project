/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file test_sensor_control.c

/**
 * @addtogroup sensor_interface_tests
 * @{
 */

/* ZTEST library */
#include <ztest.h>

/* Fake functions */
#include "util_test_sensor.h"

/************************************************
 *  ...TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test start all sensors
 * @details test start_all_sensors() in case
 * all sensors can be started successfully. Should
 * return 0.
 * @see start_all_sensors()
 */
void test_start_all_sensors(void)
{
    enable_batt();
    zassert_ok(start_all_sensors(), "Could not start all sensors");
}

/**
 * @brief Test start all sensors, no battery
 * @details test start_all_sensors() in case
 * the battery sensor cannot be started successfully.
 * Should return -EINVAL.
 * @see start_all_sensors()
 */
void test_fail_bas_start(void)
{
    disable_batt();
    zassert_equal(start_all_sensors(), -EINVAL,
                  "Started sensors despite battery sensor disabled");
}

/**
 * @brief Test stops all sensors
 */
void test_stop_all_sensors(void)
{
    enable_batt();
    zassert_ok(stop_all_sensors(), "Could not stop all sensors");
}

/**
 * @brief Test stop all sensors, battery sensor disabled
 */
void test_fail_bas_stop(void)
{
    disable_batt();
    zassert_equal(stop_all_sensors(), -EINVAL,
                  "stoped sensors despite battery sensor disabled");
}