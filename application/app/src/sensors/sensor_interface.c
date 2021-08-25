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


/*******************************************************************************
 *** SENSOR SCHEDULER
 *******************************************************************************/
/** Semaphore for indicating that a connexion has started. */
K_SEM_DEFINE(ble_conn, 0, 1);

/** Semaphore for indicating that a deconnexion has started. */
K_SEM_DEFINE(ble_deconn, 0, 1);

/** Semaphore for indicating that the BLE layer has been initialized */
K_SEM_DEFINE(ble_init_c, 0, 1);

/** Structure for the ble polling events */
struct k_poll_event events[3] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &ble_init_c, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &ble_conn, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &ble_deconn, 0),
};

/**
 * @brief Sensor handler thread
 * Use the polling API on ble events to do 3 things:
 * 1. Initialize BLE services when BLE layer has been initialized.
 * 2. Starts all sensors sampling/processing on BLE connection.
 * 3. Stops all sensors sampling/processing on BLE disconnection.
 */
void sensor_handler(void)
{
	int rc = 0;
	while (1) {
		rc = k_poll(events, 3, K_FOREVER);

		/* BLE INITIALIZATION */
        if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			k_sem_take(events[0].sem, K_MSEC(0));

		/* BLE CONNECTION */
		} else if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(events[1].sem, K_MSEC(0));
			start_all_sensors();

		/* BLE DECONNECTION */
        } else if (events[2].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(events[2].sem, K_MSEC(0));
            stop_all_sensors();
        }

		/* Reset the events */
        events[0].state = K_POLL_STATE_NOT_READY;
        events[1].state = K_POLL_STATE_NOT_READY;
		events[2].state = K_POLL_STATE_NOT_READY;

		#if defined(CONFIG_ARCH_POSIX)
             k_cpu_idle();
        #endif
	}
}
/* Defines the handler thread with its stacksize (to be optimized) and priority. */
K_THREAD_DEFINE(sensor_handler_id, STACKSIZE_S_HANDLER, sensor_handler, NULL, NULL,
		NULL, PRIORITY_S_HANDLER, 0, 0);

/** @} */