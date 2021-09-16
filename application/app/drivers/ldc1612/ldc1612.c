/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file ldc1612.c
#include <zephyr/types.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <soc.h>
#include <devicetree.h>
#include <logging/log.h>

#include <pm/pm.h>
#include <hal/nrf_power.h>

#include "ldc1612.h"

LOG_MODULE_REGISTER(LDC1612, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT ti_ldc1612 ///< Reference to the binding and compatible on the Device Tree.

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "RESP SIMU driver enabled without any devices"
#endif

/**
 * @brief Respiration data
 *
 * This structure represents the data from the simulated Respiration sensor.
 * Contains the bind to the I2C device and the raw data.
 */
struct ldc1612_data
{
    /** I2C Device binding */
    const struct device *i2c;
    const struct device *int_gpio;
    const struct device *sd_gpio;
    const struct device *clk_st_gpio;
    /** Table of Respiration raw samples */
    uint16_t resp_raw_sample_chan0;
    uint16_t resp_raw_sample_chan1;

    uint32_t resp_sample_dsp;
};

/**
 * @brief Respiration config
 *
 * This structure represents the configuration from the simulated Respiration sensor.
 * Contains the bus label (temporary).
 */
struct ldc1612_config
{
    const char *i2c_label;
    uint16_t i2c_addr;

    uint8_t int_pin;
    uint8_t int_flags;
    const char *int_label;

    uint8_t sd_pin;
    uint8_t sd_flags;
    const char *sd_label;

    uint8_t clk_st_pin;
    uint8_t clk_st_flags;
    const char *clk_st_label;
};

/**
 * @brief Reads from the sensor on i2c bus.
 * @param[in] dev i2c bus device.
 * @param[in] reg_addr Address of the register to be read.
 * @param[out] value Value read on the register.
 * @return 0 on success. -1 on failure.
 */
static int ldc1612_i2c_read(const struct device *dev, uint8_t reg_addr, uint16_t *value)
{
    uint8_t buf[2];

    if (i2c_burst_read(dev, DT_INST_REG_ADDR(0), reg_addr, (uint8_t *)buf, 2) < 0)
    {
        LOG_ERR("Error reading register.");
        return -1;
    }

    *value = buf[0] << 8 | buf[1];

    return 0;
}

/**
 * @brief Writes to the sensor on i2c bus.
 * @param[in] dev i2c bus device.
 * @param[in] reg_addr Address of the register to be written to.
 * @param[out] value Value to write on the register.
 * @return 0 on success. -1 on failure.
 */
static int ldc1612_i2c_write(const struct device *dev, uint8_t reg_addr, uint16_t value)
{
    uint8_t tx_buf[2] = {value >> 8, value & 0xFF};

    if (i2c_burst_write(dev, DT_INST_REG_ADDR(0), reg_addr, tx_buf, 2) < 0)
    {
        LOG_ERR("Error writing register.");
        return -1;
    }

    return 0;
}

static int ldc1612_wakeup(const struct device *dev)
{
    if (ldc1612_i2c_write(dev, LDC1612_REG_CONFIG, LDC1612_WAKEUP) < 0)
    {
        LOG_ERR("Failed to wakeup");
        return -1;
    }

    ldc1612_sleep_flag = false;

    return 0;
}

static int ldc1612_sleep(const struct device *dev)
{
    if (ldc1612_i2c_write(dev, LDC1612_REG_CONFIG, LDC1612_PUT_TO_SLEEP) < 0)
    {
        LOG_ERR("Failed to put to sleep");
        return -1;
    }

    ldc1612_sleep_flag = true;

    return 0;
}

static int ldc1612_read_channels(const struct device *dev, uint32_t *channel_0, uint32_t *channel_1)
{
    //TODO: wakeup and wait for 710 us
    // Read channel 0 and channel 1, channel1 should be zero
    //LOG_DBG("LDC1612_READING");
    uint16_t value;
    uint16_t status;
    ldc1612_i2c_read(dev, LDC1612_REG_STATUS, &status);
    // Make sure that data conversion is done
    // TODO: use interrupt line
    if (status & 0x0008)
    {
        ldc1612_i2c_read(dev, LDC1612_REG_DATA0_MSB, &value);
        // Delete error flags
        value = value & 0x0FFF;
        *channel_0 = value << 16;
        value = 0;
        ldc1612_i2c_read(dev, LDC1612_REG_DATA0_LSB, &value);
        *channel_0 = *channel_0 | value;

        *channel_1 = 0;
    }

    // ldc1612_i2c_read(dev, LDC1612_REG_DATA0_MSB, &value);
    // // Delete error flags
    // value = value & 0x0FFF;
    // *channel_0 = value << 16;
    // value = 0;
    // ldc1612_i2c_read(dev, LDC1612_REG_DATA0_LSB, &value);
    // *channel_0 = *channel_0 | value;

    // *channel_1 = 0;
    //ldc1612_i2c_read(dev, LDC1612_REG_DATA1_MSB, &value);
    //// Delete error flags
    //value = value & 0x0FFF;
    //*channel_1 = value << 16;
    //value = 0;
    //ldc1612_i2c_read(dev, LDC1612_REG_DATA1_LSB, &value);
    //*channel_1 = *channel_1 | value;

    ldc1612_sleep(dev);

    if (*channel_0 == 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/**
 * Fetch a sample from the sensor channels and stores it in an internal buffer. 
 * The data is generated randomly.
 * @param[in] dev Sensor device binding
 * @param[in] chan Sensor channel
 */
static int ldc1612_sample_fetch(const struct device *dev,
                                enum sensor_channel chan)
{
    struct ldc1612_data *drv_data = dev->data;
    uint32_t ldc1612_channel_0 = 0;
    uint32_t ldc1612_channel_1 = 0;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    // Wake the sensor up, wait for acq time and read the channels

    ldc1612_wakeup(drv_data->i2c);
    k_busy_wait(LDC1612_ACQ_TIME * 1000);
    ldc1612_read_channels(drv_data->i2c, &ldc1612_channel_0, &ldc1612_channel_1);
    ldc1612_sleep(drv_data->i2c);

    if (ldc1612_channel_0 != LDC1612_MAX_VALUE_28BIT)
    {
        device_on_body = true;
    }
    else
    {
        device_on_body = false;
    }

    //Invert signal such that peaks corresponds to inhalation
    uint32_t inverted_signal = LDC1612_MAX_VALUE_28BIT - ldc1612_channel_0;

    if (inverted_signal == 0 && prev_signal != 0)
    {
        inverted_signal = prev_signal;
    }
    else if (inverted_signal != 0)
    {
        prev_signal = inverted_signal;
    }

    // convert samples to 16 bits
    ldc1612_channel_0 = inverted_signal >> 12;

    ldc1612_channel_1 = ldc1612_channel_1 >> 12;

    // store into the structure accessible through the sensor API
    drv_data->resp_raw_sample_chan0 = (uint16_t)ldc1612_channel_0;
    drv_data->resp_raw_sample_chan1 = (uint16_t)ldc1612_channel_1;
    drv_data->resp_sample_dsp = (uint32_t)inverted_signal;

    return 0;
}

/**
 * Get a data from an individual sensor device. 
 * The data is generated randomly.
 * @param[in] dev Sensor device binding
 * @param[in] chan Channel of data
 * @param[out] val Returned data in a sensor_value structure
 */
static int ldc1612_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct ldc1612_data *drv_data = dev->data;
    val->val1 = drv_data->resp_raw_sample_chan0;
    (val + 1)->val1 = drv_data->resp_sample_dsp;
    return 0;
}

/**
 * Connects the sensor driver api to the Respiration sensor.
 */
static const struct sensor_driver_api ldc1612_driver_api = {
    .sample_fetch = ldc1612_sample_fetch,
    .channel_get = ldc1612_channel_get};

/**
 * Initialize the sensor device. 
 * Sleeps for 1 ms.
 * @param[in] dev Sensor device binding
 * @return 0 if success. -EINVAL on failure.
 */
static int ldc1612_init(const struct device *dev)
{
    struct ldc1612_data *drv_data = dev->data;
    const struct ldc1612_config *cfg = dev->config;

    prev_signal = 0;

    /* GET THE I2C BUS */
    drv_data->i2c = device_get_binding(cfg->i2c_label);
    if (drv_data->i2c == NULL)
    {
        LOG_ERR("Failed to get pointer to %s device!", log_strdup(cfg->i2c_label));
        return -EINVAL;
    }

    /* GET THE SD PIN */
    drv_data->sd_gpio = device_get_binding(cfg->sd_label);
    if (drv_data->sd_gpio == NULL)
    {
        LOG_ERR("Failed to get pointer to %s device!", log_strdup(cfg->sd_label));
        return -EINVAL;
    }

    int err;
    err = gpio_pin_configure(drv_data->sd_gpio, cfg->sd_pin,
                             GPIO_OUTPUT | GPIO_OUTPUT_LOW);
    err = gpio_pin_set(drv_data->sd_gpio, cfg->sd_pin, 0);

    k_busy_wait(3000);

    /* GET THE CLK_ST PIN */
    drv_data->clk_st_gpio = device_get_binding(cfg->clk_st_label);
    if (drv_data->clk_st_gpio == NULL)
    {
        LOG_ERR("Failed to get pointer to %s device!", log_strdup(cfg->clk_st_label));
        return -EINVAL;
    }

    err = gpio_pin_configure(drv_data->clk_st_gpio, cfg->clk_st_pin,
                             GPIO_OUTPUT | GPIO_OUTPUT_HIGH);
    err = gpio_pin_set(drv_data->clk_st_gpio, cfg->clk_st_pin, 1);

    k_busy_wait(3000);

    /* CLOCK DIVIDER 0 TO ... */
    if (ldc1612_write(LDC1612_REG_CLOCK_DIVIDERS0,
                      LDC1612_FIN_DIV0_1 | LDC1612_FREF_DIV0_1) < 0)
    {
        LOG_ERR("Error writing the clock divider 0.\n");
        return -EINVAL;
    }

    /* SETTLE COUNT 0 TO 250 */
    if (ldc1612_write(LDC1612_REG_SETTLECOUNT0,
                      LDC1612_SET_COUNT0_500) < 0)
    {
        LOG_ERR("Error writing the settle count 0.\n");
        return -EINVAL;
    }

    /* Channel 0 reference count conversion interval time set to 1200 */
    if (ldc1612_write(LDC1612_REG_RCOUNT0,
                      LDC1612_RCOUNT0_2400) < 0)
    {
        LOG_ERR("Error writing the rcount0.\n");
        return -EINVAL;
    }

    /* Channel 0 conversion offset set to 4210 */
    if (ldc1612_write(LDC1612_REG_OFFSET0,
                      LDC1612_OFFSET0_4210) < 0)
    {
        LOG_ERR("Error writing the offset0.\n");
        return -EINVAL;
    }

    /* Channel 0 L-C Sensor Drive Current to 16
	and Channel 0 intial sensor drive current to 0 */
    if (ldc1612_write(LDC1612_REG_DRIVE_CURRENT0,
                      LDC1612_IDRIVE0_16 | LDC1612_INIT_IDRIVE0_0) < 0)
    {
        LOG_ERR("Error writing the current0.\n");
        return -EINVAL;
    }

    /* CLOCK DIVIDER 1 TO ... */
    if (ldc1612_write(LDC1612_REG_CLOCK_DIVIDERS1,
                      LDC1612_FIN_DIV1_1 | LDC1612_FREF_DIV1_1) < 0)
    {
        LOG_ERR("Error writing the clock divider 1.\n");
        return -EINVAL;
    }

    /* SETTLE COUNT 1 TO 250 */
    if (ldc1612_write(LDC1612_REG_SETTLECOUNT1,
                      LDC1612_SET_COUNT1_250) < 0)
    {
        LOG_ERR("Error writing the settle count 1.\n");
        return -EINVAL;
    }

    /* Channel 1 reference count conversion interval time set to 1200 */
    if (ldc1612_write(LDC1612_REG_RCOUNT1,
                      LDC1612_RCOUNT1_1200) < 0)
    {
        LOG_ERR("Error writing the rcount1.\n");
        return -EINVAL;
    }

    /* Channel 1 conversion offset set to 4210 */
    if (ldc1612_write(LDC1612_REG_OFFSET1,
                      LDC1612_OFFSET1_4210) < 0)
    {
        LOG_ERR("Error writing the offset1.\n");
        return -EINVAL;
    }

    /* Channel 1 L-C Sensor Drive Current to 16
	and Channel 1 intial sensor drive current to 0 */
    if (ldc1612_write(LDC1612_REG_DRIVE_CURRENT1,
                      LDC1612_IDRIVE1_16 | LDC1612_INIT_IDRIVE1_0) < 0)
    {
        LOG_ERR("Error writing the current1.\n");
        return -EINVAL;
    }

    /* Configuration for NO Errors */
    if (ldc1612_write(LDC1612_REG_ERROR_CONFIG,
                      LDC1612_ERROR_CONFIG_NONE) < 0)
    {
        LOG_ERR("Error writing the error config register.\n");
        return -EINVAL;
    }

    /* MUX Configuration. See datasheet. */
    if (ldc1612_write(LDC1612_REG_MUX_CONFIG,
                      LDC1612_MUXCONFIG_AUTOSC_0 | LDC1612_MUXCONFIG_RR_SEQ_0 | LDC1612_MUXCONFIG_RESERVED | LDC1612_MUXCONFIG_DEGLITCH_10MHz) < 0)
    {
        LOG_ERR("Error writing the mux config register.\n");
        return -EINVAL;
    }

    /* Reset device (not really ?) */
    if (ldc1612_write(LDC1612_REG_RESET_DEV,
                      LDC1612_RESET_DEV_SET) < 0)
    {
        LOG_ERR("Error writing the error config register.\n");
        return -EINVAL;
    }

    /* General Configuration. See datasheet. */
    if (ldc1612_write(LDC1612_REG_CONFIG,
                      LDC1612_CONFIG_CONTIN_CONV_CH0 | LDC1612_CONFIG_SLEEPMODE_EN | LDC1612_CONFIG_RP_OVERR_EN_1 | LDC1612_CONFIG_SENSOR_ACT_SEL_1 | LDC1612_CONFIG_REF_CLK_SRC_INT_OSC | LDC1612_CONFIG_INTB_DIS_0 | LDC1612_CONFIG_HIGH_CUR_DRV_0 | LDC1612_CONFIG_RESERVED) < 0)
    {
        LOG_ERR("Error writing the config register.\n");
        return -EINVAL;
    }

    gpio_pin_set(drv_data->clk_st_gpio, cfg->clk_st_pin, 0);

    return 0;
}

static struct ldc1612_data ldc1612_data;
static struct ldc1612_config ldc1612_config = {
    .i2c_label = DT_INST_BUS_LABEL(0),
    .i2c_addr = DT_INST_REG_ADDR(0),
#ifdef CONFIG_LDC1612_TRIGGER
    .int_pin = DT_INST_GPIO_PIN(0, int_gpios),
    .int_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
    .int_label = DT_INST_GPIO_LABEL(0, int_gpios),
#endif /* CONFIG_LDC1612_TRIGGER */
    .sd_pin = DT_INST_GPIO_PIN(0, sd_gpios),
    .sd_flags = DT_INST_GPIO_FLAGS(0, sd_gpios),
    .sd_label = DT_INST_GPIO_LABEL(0, sd_gpios),
    .clk_st_pin = DT_INST_GPIO_PIN(0, clk_st_gpios),
    .clk_st_flags = DT_INST_GPIO_FLAGS(0, clk_st_gpios),
    .clk_st_label = DT_INST_GPIO_LABEL(0, clk_st_gpios),
};

DEVICE_DT_INST_DEFINE(0, &ldc1612_init, NULL,
                      &ldc1612_data, &ldc1612_config, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &ldc1612_driver_api);
