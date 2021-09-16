/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file sensor_interface.h
 * @defgroup sensor_interface Sensor Interface: Z-Wear sensor 
 * interface definitions and functions.
 * @brief Functions and definitions for the app to interface with the sensors.
 */

#ifndef _SENSOR_INTERFACE_H
#define _SENSOR_INTERFACE_H

/* BLE interface functions */
#include "../ble/ble_interface.h"

#include <sys_clock.h>

/* Bluetooth Stack Libraries */
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

/* Sensors functions */
#include "battery/battery.h"

/** Stacksize for the sensor handler thread */
#define STACKSIZE_S_HANDLER 256
/** Priority for the sensor handler thread */
#define PRIORITY_S_HANDLER 2

/******************************************************************************
 *** SENSOR CONTROL INTERFACES
 ******************************************************************************/
/**
 * @brief Starts all sensors.
 * 
 * @return 0 on success. 1 if sensors not initialized and -1 if failure.
 */
int start_all_sensors(void);

/**
 * @brief Stops all sensors.
 * 
 * @return 0 on success. 1 if sensors not initialized and -1 if failure.
 */
int stop_all_sensors(void);

/******************************************************************************
 *** NOTIFYING INTERFACES
 ******************************************************************************/
/**
 * @brief Sends a notification of battery level.
 * 
 * @param[in] batt_level Level of battery measured by the sensor.
 * @return 0 on success.
 */
int bas_notify(int16_t batt_level);

#endif /* _SENSOR_INTERFACE_H */