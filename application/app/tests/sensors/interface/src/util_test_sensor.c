/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_sensor.c

#include "util_test_sensor.h"

/************************************************
 *  ...STUBS
 ***********************************************/
int z_impl_k_poll(struct k_poll_event *events, int num_events,
		  k_timeout_t timeout)
{
    return 0;
}

int bt_bas_set_battery_level(uint8_t batt)
{
    return 0;
}

/************************************************
 *  ...MOCKS UTILITIES
 ***********************************************/
static int battery_sensor_ok;

void enable_batt(void)
{
    battery_sensor_ok = 0;
}
void disable_batt(void)
{
    battery_sensor_ok = -1;
}

/************************************************
 *  ...MOCKS
 ***********************************************/
int bas_start(void)
{
    return battery_sensor_ok;
}

int bas_stop(void)
{
    return battery_sensor_ok;
}