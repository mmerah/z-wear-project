/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file ble_interface.c
 * @addtogroup ble_interface
 * @{
 */

#include "ble_interface.h"
/* Sensor interface functions */
#include "../sensors/sensor_interface.h"

/** Semaphore for extern use for indicating that a device has connected 
 * to the sensor. */
extern struct k_sem ble_conn;

/** Semaphore for extern use for indicating that a device has deconnected 
 * from the sensor. */
extern struct k_sem ble_deconn;

/** Semaphore for extern use for indicating that the BLE layer has been 
 * initialized. */
extern struct k_sem ble_init_c;

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(ble_interface, LOG_LEVEL_INF);

/* Reboot library */
#include <sys/reboot.h>

/** Static semaphore for ble initialization threads communication. */
static K_SEM_DEFINE(ble_init_ok, 0, 1);

/******************************************************************************
 *** BLE TIMEOUT
 ******************************************************************************/

/** Semaphore for pushing processing work to the processor thread. */
static K_SEM_DEFINE(ble_timeout, 0, 1);

/**
 * @brief No connection timeout ISR
 */
static void ble_timeout_timer_handler(struct k_timer *dummy)
{
    k_sem_give(&ble_timeout);
}

/** Defines a k_timer for handling the sampling of the sensor. */
K_TIMER_DEFINE(ble_timeout_timer, ble_timeout_timer_handler, NULL);

int ble_timeout_start(void)
{
    /**
	 * Starts the ble timeout timer on disconnection. 
	 * This is a one shot timer that expires after 30 seconds.
	 */
    int rc = ble_layer_resume();
    if (rc)
    {
        LOG_ERR("Could not restart the BLE layer (err %d)", rc);
        return rc;
    }
    k_timer_start(&ble_timeout_timer, K_SECONDS(30), K_NO_WAIT);
    return 0;
}

/**
 * @brief Thread for processing a BLE timeout.
 * This thread activates once the 30 seconds, no BLE connection timeout
 * activates. It stops advertising and reboots the system to get back
 * to the lowest current consumption state.
 */
void ble_handler(void)
{
    while (1)
    {
        k_sem_take(&ble_timeout, K_FOREVER);

        /* Pause the advertising to shut off the radio (TBV) */
        int rc = ble_layer_pause();
        if (rc)
        {
            LOG_ERR("Could not pause the advertising (err %d)", rc);
        }

        /* Reboots the device. Uncomment to reactivate. */
        /*sys_reboot(SYS_REBOOT_COLD);*/

#if defined(CONFIG_ARCH_POSIX)
        k_cpu_idle();
#endif
    }
}

/* Defines the handler thread with its stacksize and priority. */
K_THREAD_DEFINE(ble_handler_id, STACKSIZE_BLE_HANDLER, ble_handler, NULL, NULL,
                NULL, PRIORITY_BLE_HANDLER, 0, 0);

/******************************************************************************
 *** BLE INTERFACES
 ******************************************************************************/

/**
 * Static bt_gatt_exchange_params for exchanging the MTU.
 * Needs to be static or Usage Fault -> TBI
 */
static struct bt_gatt_exchange_params exchange_params;

/** A static bt_conn structure. */
static struct bt_conn *zwear_connection;

struct bt_conn *connection_get(void)
{
    return zwear_connection;
}

/**
 * @brief Grabs the current connection
 * 
 * @param[in] conn_grabbed bt_conn structure
 */
static void connection_grab(struct bt_conn *conn_grabbed)
{
    zwear_connection = conn_grabbed;
}

/** Static bt_data struct for the advertisement data. */
static const struct bt_data ad[] =
    {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))};

/**
 * @brief Exchange MTU data.
 * @param[in] conn Pointer to the current bt connection
 * @param[in] att_err Indicates if the MTU exchange is successful
 * @param[in] param Gatt exchange parameters
 */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    struct bt_conn_info info = {0};
    int err;

    LOG_INF("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");

    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d\n", err);
        return;
    }
}

/**
 * @brief Connection callback
 * @param[in] conn Pointer to the current bt connection
 * @param[in] err Indicates if the connection is successful
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];

    connection_grab(conn);

    if (err)
    {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    else if (bt_conn_get_info(conn, &info))
    {
        LOG_ERR("Could not parse connection info");
    }
    else
    {
#ifndef CONFIG_ARCH_POSIX
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        LOG_INF("Connection established!");
        LOG_INF("Connected to: %s", log_strdup(addr));
#endif
        LOG_INF("Role: %u", info.role);
        LOG_INF("Connection interval: %u", info.le.interval);
        LOG_INF("Slave latency: %u", info.le.latency);
        LOG_INF("Connection supervisory timeout: %u", info.le.timeout);
    }

    exchange_params.func = exchange_func;
    bt_gatt_exchange_mtu(connection_get(), &exchange_params);

    /* Starts sampling data on connection */
    k_sem_give(&ble_conn);

    /* Stops the ble timeout timer on connection */
    k_timer_stop(&ble_timeout_timer);
}

/**
 * @brief Disconnection callback
 * @param[in] conn Pointer to the current bt connection
 * @param[in] reason Indicates disconnection reason
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)\n", reason);

    /* Stop sampling data on disconnect */
    k_sem_give(&ble_deconn);

    /**
	 * Starts the ble timeout timer on disconnection. 
	 * This is a one shot timer that expires after 30 seconds.
	 */
    k_timer_start(&ble_timeout_timer, K_SECONDS(30), K_NO_WAIT);
}

/**
 * @brief Data structure for the BLE callback functions
 */
static struct bt_conn_cb conn_callbacks =
    {
        .connected = connected,
        .disconnected = disconnected,
};

/**
 * @brief Sets the DIS parameters in NVS
 * @return 0 on success
 */
static int settings_runtime_load(void)
{
#if defined(CONFIG_BT_DIS_SETTINGS)
    settings_runtime_set("bt/dis/model",
                         CONFIG_BT_DIS_MODEL,
                         sizeof(CONFIG_BT_DIS_MODEL));
    settings_runtime_set("bt/dis/manuf",
                         CONFIG_BT_DIS_MANUF,
                         sizeof(CONFIG_BT_DIS_MANUF));
#if defined(CONFIG_BT_DIS_SERIAL_NUMBER)
    settings_runtime_set("bt/dis/serial",
                         CONFIG_BT_DIS_SERIAL_NUMBER_STR,
                         sizeof(CONFIG_BT_DIS_SERIAL_NUMBER_STR));
#endif
#if defined(CONFIG_BT_DIS_SW_REV)
    settings_runtime_set("bt/dis/sw",
                         SW_VERSION, sizeof(SW_VERSION));
#endif
#if defined(CONFIG_BT_DIS_FW_REV)
    settings_runtime_set("bt/dis/fw",
                         FW_VERSION, sizeof(FW_VERSION));
#endif
#if defined(CONFIG_BT_DIS_HW_REV)
    settings_runtime_set("bt/dis/hw",
                         CONFIG_BT_DIS_HW_REV_STR,
                         sizeof(CONFIG_BT_DIS_HW_REV_STR));
#endif
#endif
    return 0;
}

/**
 * @brief Ready up the BLE layer
 * @param[in] Indicates if the BLE initialization is successful
 */
static void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("BLE init failed with error code %d\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        settings_load();
    }
    err = settings_runtime_load();

    /* Configure connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    k_sem_give(&ble_init_ok);
}

int ble_layer_init(void)
{
    LOG_INF("Starting BLE peripheral\n");
    int err = bt_enable(bt_ready);

    if (err)
    {
        LOG_ERR("BLE initialization failed\n");
        return -EINVAL;
    }
    /* 	Bluetooth stack should be ready in less than 100 msec.
	We use this semaphore to wait for bt_enable to call bt_ready before 
	we proceed to the main loop. By using the semaphore to block execution we 
	allow the RTOS to execute other tasks while we wait. */
    err = k_sem_take(&ble_init_ok, K_MSEC(500));

    if (!err)
    {
        k_sem_give(&ble_init_c);
        LOG_INF("Bluetooth initialized\n");
    }
    else
    {
        LOG_ERR("BLE initialization did not complete in time\n");
        return -EINVAL;
    }

    return 0;
}

int ble_layer_pause(void)
{
    int err = bt_le_adv_stop();
    return err;
}

int ble_layer_resume(void)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad),
                              NULL, 0);
    return err;
}

/** @} */