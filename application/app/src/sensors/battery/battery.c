/*
 * Copyright (c) 2021, Nanoleq AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file battery.c
 * @addtogroup battery
 * @{
 */
#include "battery.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/** Device structure for the Battery sensor device. */
const struct device *dev_batt;

/** Defines a battery data FIFO file for inter-thread communication */
static K_FIFO_DEFINE(fifo_batt_data);

/** Semaphore for indicating that the battery sampling is initialized */
static K_SEM_DEFINE(batt_init_ok, 0, 1);

/** Semaphore for indicating that the battery processor thread is ready. */
static K_SEM_DEFINE(batt_process_ready, 0, 1);

/** Semaphore for pushing processing work to the processor thread */
static K_SEM_DEFINE(batt_ok, 0, 1);

/**
 * @brief Battery measures ISR
 */
static void bas_timer_handler(struct k_timer *dummy)
{
    k_sem_give(&batt_ok);
}

/** Defines a k_timer for handling the sampling of the sensor. */
K_TIMER_DEFINE(batt_timer, bas_timer_handler, NULL);

int bas_init(void)
{
	/* Indicate to the other threads that the initialization is finished. */
    k_sem_give(&batt_init_ok);
    return 0;
}

int bas_stop(void)
{
    k_timer_stop(&batt_timer);
    LOG_INF("BAS stopped\n");
    return 0;
}

int bas_start(void)
{
    /* start periodic timer that expires once every "samplingTime" */
    k_timer_start(&batt_timer, K_MSEC(1), K_MSEC(BAS_SAMPLING_TIME_MS));
    LOG_INF("BAS started\n");
    return 0;
}

/**
 * @brief Battery level data structure for the FIFO file.
 * This structure containes a void* for the FIFO initialization. Also,
 * it contains a one sample, and a timestamp in ms for the data. 
 */
struct data_item_batt_t {
    /** 1st word reserved for use by FIFO */
    void *fifo_reserved;   
    /** Contains the data for battery value */
    int16_t data;
};

/**
 * @brief Thread for processing a battery level sample.
 * This thread activates once the battery simulation is initialized. It first
 * indicates to the notify thread that it is ready to launch. Then it waits
 * for a battery sample before starting to process the data. Finally, it 
 * puts the results in a FIFO using a data_item_batt_t structure.
 */
void batt_process_thread(void)
{
    /* Wait for the sensor to be initialized before starting the thread loop. */
    k_sem_take(&batt_init_ok, K_FOREVER);
    k_sem_give(&batt_process_ready);
    struct data_item_batt_t batt_data;
    while (1) {
        /* Semaphore is available every sampling time */
        k_sem_take(&batt_ok, K_FOREVER);

		batt_data.data = 42;

        /* send data to consumers */
        k_fifo_put(&fifo_batt_data, &batt_data);

        #if defined(CONFIG_ARCH_POSIX)
             k_cpu_idle();
        #endif
    }
}

/**
 * @brief Thread for notifying the Battery service.
 * This thread waits for the processor thread to be ready before starting. Then,
 * it wakes up when the FIFO pile for battery data has an element to sends it through
 * a notification.
 */
void batt_notify_thread(void)
{
	k_sem_take(&batt_process_ready, K_FOREVER);
    struct data_item_batt_t  *rx_data_batt;
    while (1) {
        /* Notify what is in the fifo as soon as there is something */
        rx_data_batt = k_fifo_get(&fifo_batt_data, K_FOREVER);
        LOG_INF("rx_data_batt = %d \n\n", rx_data_batt->data);
        bas_notify(rx_data_batt->data);

        #if defined(CONFIG_ARCH_POSIX)
             k_cpu_idle();
        #endif
    }
}

/* Defines the processor thread with its stacksize (to be optimized) and priority. */
K_THREAD_DEFINE(batt_producer_thread_id, STACKSIZE_BAS_PROCESS, batt_process_thread, NULL, NULL,
		NULL, PRIORITY_BAS_PROCESS, 0, 0);
/* Defines the notifying thread with its stacksize (to be optimized) and priority. */
K_THREAD_DEFINE(batt_notify_thread_id, STACKSIZE_BAS_NOTIFY, batt_notify_thread, NULL, NULL,
		NULL, PRIORITY_BAS_NOTIFY, 0, 0);

/** @} */