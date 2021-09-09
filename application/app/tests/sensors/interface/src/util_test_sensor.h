/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_sensor.h

#ifndef _UTIL_TEST_SENSOR_H
#define _UTIL_TEST_SENSOR_H

#include <zephyr.h>
#include <stdint.h>

/************************************************
 *  ...STUBS DECLARATIONS
 ***********************************************/
int z_impl_k_poll(struct k_poll_event *events, int num_events,
		  k_timeout_t timeout);

int bt_bas_set_battery_level(uint8_t batt);

/************************************************
 *  ...MOCKS UTILITIES
 ***********************************************/
void enable_batt(void);
void disable_batt(void);

#endif /* _UTIL_TEST_SENSOR_H */