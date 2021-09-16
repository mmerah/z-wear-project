/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file settings_storage.c
 * @addtogroup settings
 * @{
 */

#include "settings_storage.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(settings_storage, LOG_LEVEL_ERR);

/** Battery design voltage in mV */
static int16_t voltage_val = 3700;
/** Battery capacity in mAh */
static int16_t capacity_val = 230;
/** Power cycle iterator */
static uint16_t iteration = 0;
/** Battery type */
static char battery_type[6] = "Lipo";

/** Authentication key */
static char secret_key_st[17] = SECRET_AUTH_KEY;

/* Static subtree handler */
SETTINGS_STATIC_HANDLER_DEFINE(param, "zwear/param", param_handle_get,
                               param_handle_set, NULL,
                               param_handle_export);

int param_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                     void *cb_arg)
{
    const char *next;
    size_t name_len;
    int rc;

    name_len = settings_name_next(name, &next);

    if (!next)
    {
        if (!strncmp(name, "voltage", name_len))
        {
            rc = read_cb(cb_arg, &voltage_val, sizeof(voltage_val));
            LOG_INF("<zwear/param/voltage> = %d", voltage_val);
            return 0;
        }

        if (!strncmp(name, "capacity", name_len))
        {
            rc = read_cb(cb_arg, &capacity_val, sizeof(capacity_val));
            LOG_INF("<zwear/param/capacity> = %d", capacity_val);
            return 0;
        }

        if (!strncmp(name, "iteration", name_len))
        {
            rc = read_cb(cb_arg, &iteration, sizeof(iteration));
            LOG_INF("<zwear/param/iteration> = %d", iteration);
            return 0;
        }

        if (!strncmp(name, "type", name_len))
        {
            rc = read_cb(cb_arg, battery_type,
                         sizeof(battery_type));
            if (rc < 0)
            {
                return rc;
            }
            else if (rc > 0)
            {
                LOG_INF("<zwear/param/type> = %s",
                        log_strdup(battery_type));
            }
            return 0;
        }

        if (!strncmp(name, "secret", name_len))
        {
            rc = read_cb(cb_arg, secret_key_st,
                         sizeof(secret_key_st));
            if (rc < 0)
            {
                return rc;
            }
            else if (rc > 0)
            {
                LOG_INF("<zwear/param/secret> = %s",
                        log_strdup(secret_key_st));
            }
            return 0;
        }
    }

    return -ENOENT;
}

int param_handle_export(int (*cb)(const char *name,
                                  const void *value, size_t val_len))
{
    (void)cb("zwear/param/voltage", &voltage_val, sizeof(voltage_val));
    (void)cb("zwear/param/capacity", &capacity_val, sizeof(capacity_val));
    (void)cb("zwear/param/iteration", &iteration, sizeof(iteration));
    (void)cb("zwear/param/type", battery_type,
             strlen(battery_type) + 1);
    (void)cb("zwear/param/secret", secret_key_st, strlen(secret_key_st) + 1);
    return 0;
}

int param_handle_get(const char *name, char *val, int val_len_max)
{
    const char *next;

    if (settings_name_steq(name, "type", &next) && !next)
    {
        val_len_max = MIN(val_len_max, strlen(battery_type));
        memcpy(val, battery_type, val_len_max);
        return val_len_max;
    }

    if (settings_name_steq(name, "secret", &next) && !next)
    {
        val_len_max = MIN(val_len_max, strlen(secret_key_st));
        memcpy(val, secret_key_st, val_len_max);
        return val_len_max;
    }

    return -ENOENT;
}

/**
 * @brief Loads the subtree from the storage containing the data.
 * Also update the number of power cycles.
 * @return 0 on success, 0 otherwise
 */
static int load_settings(void)
{
    int rc = -1;

    /* Loading the subtree */
    rc = settings_load_subtree("zwear/param");
    if (rc)
    {
        LOG_ERR("Could not load subtree (err %d)", rc);
        return rc;
    }

    /* Increment the power cycle */
    //iteration++;

    /* Save any changes */
    rc = settings_save();
    if (rc)
    {
        LOG_ERR("Could not save current items");
        return rc;
    }

    return 0;
}

int storage_initialization(void)
{
    int rc = -1;

    rc = settings_subsys_init();
    if (rc)
    {
        LOG_ERR("settings subsys initialization: fail (err %d)", rc);
        return rc;
    }
    
    rc = load_settings();
    {
        return rc;
    }

    return 0;
}

int get_battery_parameters(struct batt_param_t *batt_param)
{
    batt_param->voltage = voltage_val;
    batt_param->capacity = capacity_val;
    batt_param->n_power = iteration;
    strncpy(batt_param->batt_typ, battery_type, strlen(battery_type));
    return 0;
}

int get_secret_key(char stored_key[16])
{
    strncpy(stored_key, secret_key_st, strlen(secret_key_st));
    return 0;
}

/** @} */