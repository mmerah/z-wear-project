/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/**
 * @brief Tests for the sensor interface module
 *
 * Verify sensor handler commands and abstractions
 *
 * @defgroup sensor_interface_tests SENS
 * @ingroup all_tests
 * @{
 * @}
 */

#include <ztest.h>

/* Code tested */
#include "../../../../src/sensors/sensor_interface.h"
#include "../../../../src/sensors/sensor_interface.c"

/************************************************
 *  ... SENSOR CONTROL INTERFACES
 ***********************************************/

extern void test_start_all_sensors(void);
extern void test_fail_bas_start(void);

extern void test_stop_all_sensors(void);
extern void test_fail_bas_stop(void);

/************************************************
 *  ... NOTIFYING INTERFACES
 ***********************************************/

extern void test_bas_notify(void);

/************************************************
 *  ... SENSOR SCHEDULER
 ***********************************************/

//extern void test_sensor_handler_init(void);

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
    ztest_test_suite(test_interface,
                     ztest_unit_test(test_start_all_sensors),
                     ztest_unit_test(test_fail_bas_start),
                     ztest_unit_test(test_stop_all_sensors),
                     ztest_unit_test(test_fail_bas_stop),
                     ztest_unit_test(test_bas_notify));

    ztest_run_test_suite(test_interface);
}