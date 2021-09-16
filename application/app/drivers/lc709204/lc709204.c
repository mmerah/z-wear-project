/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file lc709204.c

#define DT_DRV_COMPAT onnn_lc709204

#include "lc709204.h"

/* Standard and Zephyr librairies */
#include <zephyr.h>
#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>

/*
Datasheet: https://www.onsemi.com/pdf/datasheet/lc709204f-d.pdf
Application Note: https://www.onsemi.com/pub/collateral/and9985-d.pdf

I2C Address should be 0x16
In the DTC, the sensor should appear as:
&i2c0 {
	lc709204@16 {
		compatible = "onnn,lc709204";
		label = "LC709204";
		reg = <0x16>;
		design-voltage = <3700>;
		design-capacity = <90>;
		terminate-voltage = <3000>;
		tsense1-b-constant = <3380>;
	};
};

Flow with Thermistor Mode (MINIMAL):
1. Write APA (0x0B): We use a 230mAh battery 
    Thus: we should write 0x1818 to APA
2. Write Change Of The Parameter (0x12): Battery of type-01 (3.7V, charging 4.2V)
    Thus: we should write 0x0000 to CTP
3. Write TSENSE1 Thermistor B (0x06): B-constant of the 
    thermistor connected to TSENSE1, refer to the specification (3380K)
4. Write Status Bit (0x16): 0x0001 for TSENSE1 thermistor
5. Write IC Power Mode (0x15): 0x0001 for Operational mode
-> Read RSOC (0x0D) periodically

Flow at I2C mode:
1. Write APA (0x0B): We use a 230mAh battery 
    Thus: we should write 0x1818 to APA
2. Write Change Of The Parameter (0x12): Battery of type-01 (3.7V, charging 4.2V)
    Thus: we should write 0x0000 to CTP
3. Write IC Power Mode (0x15): 0x0001 for Operational mode
-> Read RSOC (0x0D) and write cell temperature (0x08) periodically
We should be able to use sleep mode instead of operational and just
put into operational before reading RSOC, then putting back into sleep.
*/

/**
 * @brief Read a command register for the LC709204
 * @param[in] lc709204 Pointer to the data structure linked to the sensor instance
 * @param[in] reg_addr Address of the register to be read
 * @param[out] val Value read on the register
 * @return 0 on success, error code on failure.
 */
static int lc709204_command_reg_read(struct lc709204_data *lc709204, uint8_t reg_addr,
                                     uint16_t *val)
{
    uint8_t i2c_data[2];
    int status;

    status = i2c_burst_read(lc709204->i2c, DT_INST_REG_ADDR(0), reg_addr,
                            i2c_data, 2);
    if (status < 0)
    {
        LOG_ERR("Unable to read register");
        return -EIO;
    }

    *val = (i2c_data[1] << 8) | i2c_data[0];

    return 0;
}

/**
 * @brief Writes a value to a command register for the LC709204
 * @param[in] lc709204 Pointer to the data structure linked to the sensor instance
 * @param[in] reg_addr Address of the register to be written to
 * @param[out] val Value to be written on the register
 * @return 0 on success, error code on failure.
 */
static int lc709204_command_reg_write(struct lc709204_data *lc709204, uint8_t reg_addr,
                                      uint16_t value)
{
    uint8_t tx_buf[2] = {value >> 8, value & 0xFF};
    int status = -EIO;

    status = i2c_burst_write(lc709204->i2c, DT_INST_REG_ADDR(0), reg_addr,
                             tx_buf, 2);
    if (status < 0)
    {
        LOG_ERR("Unable to write register.");
        return -EIO;
    }
    return 0;
}

/**
 * @brief Sensor value get
 * 
 * @param[in] dev Pointer to the device structure instance
 * @param[in] chan Sensor channel to be read
 * @param[out] val Sensor value read on the specified channel
 *
 * @return -ENOTSUP for unsupported channels
 */
static int lc709204_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
    struct lc709204_data *lc709204 = dev->data;

    switch (chan)
    {
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = lc709204->state_of_charge;
        val->val2 = 0;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

/**
 * @brief Sensor sample fetch
 * 
 * @param[in] dev Pointer to the device structure instance
 * @param[in] chan Sensor channel to be read
 *
 * @return -ENOTSUP for unsupported channels, -EIO for invalid data reading
 */
static int lc709204_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{
    struct lc709204_data *lc709204 = dev->data;
    int status = 0;

    switch (chan)
    {
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        status = lc709204_command_reg_write(
            lc709204, LC709204_REG_RSOC, &lc709204->state_of_charge);
        ;
        if (status < 0)
        {
            LOG_ERR("Failed to read state of charge");
            return -EIO;
        }
        break;

    default:
        return -ENOTSUP;
    }

    return status;
}

/**
 * @brief Initialize the fuel gauge
 *
 * @return 0 for success
 */
static int lc709204_gauge_init(const struct device *dev)
{
    struct lc709204_data *lc709204 = dev->data;
    const struct lc709204_config *const config = dev->config;
    int status = 0;
    uint16_t id;

    lc709204->i2c = device_get_binding(config->bus_name);
    if (lc709204->i2c == NULL)
    {
        LOG_ERR("Could not get pointer to %s device.",
                config->bus_name);
        return -EINVAL;
    }

    /* Write APA */
    status = lc709204_command_reg_write(lc709204,
                                        LC709204_REG_APA, 0x1818);
    if (status < 0)
    {
        LOG_ERR("Could not set APA register, error: %d", status);
        return -EIO;
    }

    /* Write Change of Parameter */
    status = lc709204_command_reg_write(lc709204,
                                        LC709204_REG_CHANGE_OF_PARAMETER, 0x0000);
    if (status < 0)
    {
        LOG_ERR("Could not set Change of Parameter register, error: %d", status);
        return -EIO;
    }

    /* Write TSENSE1 Thermistor B constant */
    status = lc709204_command_reg_write(lc709204,
                                        LC709204_REG_TSENSE1_B,
                                        config->tsense1_b_constant);
    if (status < 0)
    {
        LOG_ERR("Could not set TSENSE1 Themistor B, error: %d", status);
        return -EIO;
    }

    /* Write Status Bit */
    status = lc709204_command_reg_write(lc709204,
                                        LC709204_REG_STATUS_BIT, 0x0001);
    if (status < 0)
    {
        LOG_ERR("Could not set Status bit, error: %d", status);
        return -EIO;
    }

    /* Write IC Power Mode */
    status = lc709204_command_reg_write(lc709204,
                                        LC709204_REG_IC_POWER_MODE, 0x0001);
    if (status < 0)
    {
        LOG_ERR("Could not set IC Power Mode, error: %d", status);
        return -EIO;
    }

    return status;
}

static const struct sensor_driver_api lc709204_battery_driver_api = {
    .sample_fetch = lc709204_sample_fetch,
    .channel_get = lc709204_channel_get,
};

#define LC709204_INIT(index)                                           \
    static struct lc709204_data lc709204_driver_##index;               \
                                                                       \
    static const struct lc709204_config lc709204_config_##index = {    \
        .bus_name = DT_INST_BUS_LABEL(index),                          \
        .design_voltage = DT_INST_PROP(index, design_voltage),         \
        .design_capacity = DT_INST_PROP(index, design_capacity),       \
        .terminate_voltage = DT_INST_PROP(index, terminate_voltage),   \
        .tsense1_b_constant = DT_INST_PROP(index, tsense1_b_constant), \
    };                                                                 \
                                                                       \
    DEVICE_DT_INST_DEFINE(index, &lc709204_gauge_init, NULL,           \
                          &lc709204_driver_##index,                    \
                          &lc709204_config_##index, POST_KERNEL,       \
                          CONFIG_SENSOR_INIT_PRIORITY,                 \
                          &lc709204_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LC709204_INIT)