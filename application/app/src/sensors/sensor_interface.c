/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file sensor_interface.c
 * @addtogroup sensor_interface
 * @{
 */

 #include "sensor_interface.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(sensor_interface, LOG_LEVEL_ERR);

/*******************************************************************************
 *** SENSOR CONTROL INTERFACES
 *******************************************************************************/
int start_all_sensors(void)
{
	if (bas_start() < 0) {
		LOG_ERR("Could not start the battery sensor");
		return -EINVAL;
	}

	return 0;
}

int stop_all_sensors(void)
{
	if (bas_stop() < 0) {
		LOG_ERR("Could not stop the battery sensor");
		return -EINVAL;
	}
	
	return 0;
}

/*******************************************************************************
 *** NOTIFYING INTERFACES
 *******************************************************************************/

int bas_notify(int16_t batt_level)
{
	/* Sets the battery level to be read by the BAS */
	int err = bt_bas_set_battery_level((uint8_t)batt_level);
	return err;
}