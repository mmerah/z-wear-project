/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file main.c
 * @defgroup main Main: Z-Wear main thread process.
 * @brief Main thread.
 */
 
 /* Standard and Zephyr libraries */
#include <zephyr.h>
#include <stdio.h>

/* BLE Interface functions */
#include "ble/ble_interface.h"

/* Sensor interface functions */
#include "sensors/sensor_interface.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_ERR);

#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_SMP_BT
#include <mgmt/mcumgr/smp_bt.h>
#endif

#if defined(CONFIG_ZTEST)
int main_posix(void)
#else
int main(void)
#endif
{
	/* Register the built-in mcumgr command handlers. */
	#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
	os_mgmt_register_group();
	#endif
	#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
	img_mgmt_register_group();
	#endif

	/* Initialize the BLE layer */
	int err = ble_layer_init();
    if (err) {
		LOG_ERR("BLE initialization failed, err %d", err);
		return err;
	}
    LOG_INF("BLE initialization successful");

	#ifdef CONFIG_MCUMGR_SMP_BT
	/* Initialize the Bluetooth mcumgr transport. */
	smp_bt_register();
	#endif
	
	/* Initialize the BLE timeout for no connection */
	err = ble_timeout_start();
	if (err) {
		LOG_ERR("BLE timeout did not initialize, err %d", err);
		return err;
	}

    while(1)
    {
		#if defined(CONFIG_ARCH_POSIX)
			return 0;
        #endif
		/* Sleeps to let the idle thread running */
		k_sleep(K_FOREVER);
	}
	return 0;
}
