/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file test_notify_interface.c

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
 * @brief Test bas notify
 * @details test bas_notify() with a battery level.
 * The function should return 0.
 * @see bas_notify()
 */
void test_bas_notify(void)
{
    int16_t fake_batt_level = 90;
    zassert_ok(bas_notify(fake_batt_level), "Failed BAS notify interface");
}