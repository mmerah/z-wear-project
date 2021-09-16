/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file mc3672_trigger.c

/* Standard and Zephyr libraries */
#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "mc3672.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_DECLARE(MC3672, CONFIG_SENSOR_LOG_LEVEL);

int mc3672_trigger_set(const struct device *dev,
                       const struct sensor_trigger *trig,
                       sensor_trigger_handler_t handler)
{
    struct mc3672_data *drv_data = dev->data;
    const struct mc3672_config *cfg = dev->config;

    if ((trig->type != SENSOR_TRIG_DOUBLE_TAP) && (trig->type != SENSOR_TRIG_DATA_READY))
    {
        LOG_ERR("Trig type other than double tap or data ready not supported.");
        return -ENOTSUP;
    }

    if (mc3672_init_interrupt(dev, trig) < 0)
    {
        LOG_ERR("Failed to initialize interrupts.");
        return -EIO;
    }

    gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
                                 GPIO_INT_DISABLE);

    drv_data->data_ready_handler = handler;
    if (handler == NULL)
    {
        return 0;
    }

    drv_data->data_ready_trigger = *trig;

    gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
                                 GPIO_INT_LEVEL_LOW);

    return 0;
}

static void mc3672_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb, uint32_t pins)
{
    struct mc3672_data *drv_data =
        CONTAINER_OF(cb, struct mc3672_data, gpio_cb);
    const struct mc3672_config *cfg = drv_data->dev->config;

    ARG_UNUSED(pins);

    gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
                                 GPIO_INT_DISABLE);

    LOG_INF("In the callback for gpio interrupt.\n");

#if defined(CONFIG_MC3672_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MC3672_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

static void mc3672_thread_cb(const struct device *dev)
{
    struct mc3672_data *drv_data = dev->data;
    const struct mc3672_config *cfg = dev->config;

    if (drv_data->data_ready_handler != NULL)
    {
        drv_data->data_ready_handler(dev,
                                     &drv_data->data_ready_trigger);
    }

    /* Clears the interrupt on the sensor */
    s_m_drv_mc36xx_int_t ptINT_Event_c;
    M_DRV_MC36XX_HandleINT(drv_data->i2c, &ptINT_Event_c);
    LOG_INF("Int Event was: bWAKE: %d bACQ: %d", ptINT_Event_c.bWAKE, ptINT_Event_c.bACQ);

    /* Reactivate the double tap interrupt */
    //M_DRV_MC36XX_SetMode(drv_data->i2c, E_M_DRV_MC36XX_MODE_SNIFF);

    LOG_INF("Reconfigure the interrupt gpio.");
    gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
                                 GPIO_INT_LEVEL_LOW);
}

#ifdef CONFIG_MC3672_TRIGGER_OWN_THREAD
static void mc3672_thread(struct mc3672_data *drv_data)
{
    while (1)
    {
        k_sem_take(&drv_data->gpio_sem, K_FOREVER);
        mc3672_thread_cb(drv_data->dev);
    }
}
#endif

#ifdef CONFIG_MC3672_TRIGGER_GLOBAL_THREAD
static void mc3672_work_cb(struct k_work *work)
{
    struct mc3672_data *drv_data =
        CONTAINER_OF(work, struct mc3672_data, work);

    mc3672_thread_cb(drv_data->dev);
}
#endif

static int mc3672_set_double_tap(const struct device *dev)
{
    /* enable interrupt */
    M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    M_DRV_MC36XX_SetSampleRate(dev, E_M_DRV_MC36XX_CWAKE_SR_LP_54Hz, E_M_DRV_MC36XX_SNIFF_SR_6Hz);

    M_DRV_MC36XX_ConfigINT(dev, 0, 0, 0, 0, 1);

    M_DRV_MC36XX_SetSniffThreshold(dev, M_DRV_MC36XX_AXIS_X, 4);
    M_DRV_MC36XX_SetSniffThreshold(dev, M_DRV_MC36XX_AXIS_Y, 4);
    M_DRV_MC36XX_SetSniffThreshold(dev, M_DRV_MC36XX_AXIS_Z, 4);

    M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_SNIFF);
    return 0;
}

static int mc3672_set_fifo_full(const struct device *dev)
{
    /* enable interrupt */
    M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);
    M_DRV_MC36XX_SetSampleRate(dev, E_M_DRV_MC36XX_CWAKE_SR_LP_7Hz, E_M_DRV_MC36XX_SNIFF_SR_0p4Hz);
    M_DRV_MC36XX_EnableFIFO(dev, E_M_DRV_MC36XX_FIFO_CTL_ENABLE, E_M_DRV_MC36XX_FIFO_MODE_WATERMARK, 0);
    M_DRV_MC36XX_ConfigINT(dev, 0, 1, 0, 0, 0);
    M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_CWAKE);
    M_DRV_MC36XX_ReadRegMap(dev, NULL);
    return 0;
}

int mc3672_init_interrupt(const struct device *dev, const struct sensor_trigger *trig)
{
    struct mc3672_data *drv_data = dev->data;
    const struct mc3672_config *cfg = dev->config;

    /* setup data ready gpio interrupt */
    drv_data->gpio = device_get_binding(cfg->int_label);
    if (drv_data->gpio == NULL)
    {
        LOG_ERR("Failed to get pointer to %s device",
                cfg->int_label);
        return -EINVAL;
    }

    drv_data->dev = dev;

    gpio_pin_configure(drv_data->gpio, cfg->int_pin,
                       GPIO_INPUT | cfg->int_flags);

    gpio_init_callback(&drv_data->gpio_cb,
                       mc3672_gpio_callback,
                       BIT(cfg->int_pin));

    if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0)
    {
        LOG_ERR("Failed to set gpio callback");
        return -EIO;
    }

    if (trig->type == SENSOR_TRIG_DOUBLE_TAP)
    {
        mc3672_set_double_tap(drv_data->i2c);
    }
    else if (trig->type == SENSOR_TRIG_DATA_READY)
    {
        mc3672_set_fifo_full(drv_data->i2c);
    }

#if defined(CONFIG_MC3672_TRIGGER_OWN_THREAD)
    k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&drv_data->thread, drv_data->thread_stack,
                    CONFIG_MC3672_THREAD_STACK_SIZE,
                    (k_thread_entry_t)mc3672_thread, drv_data,
                    NULL, NULL, K_PRIO_COOP(CONFIG_MC3672_THREAD_PRIORITY),
                    0, K_NO_WAIT);
#elif defined(CONFIG_MC3672_TRIGGER_GLOBAL_THREAD)
    drv_data->work.handler = mc3672_work_cb;
#endif

    gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
                                 GPIO_INT_EDGE_TO_INACTIVE);

    return 0;
}
