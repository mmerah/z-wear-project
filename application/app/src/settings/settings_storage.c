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
	(void)cb("zwear/param/secret", secret_key_st, strlen(secret_key_st) + 1);
	return 0;
}

int param_handle_get(const char *name, char *val, int val_len_max)
{
	const char *next;

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
 */
static void just_load_stuff(void)
{
    /* Loading the subtree */
	settings_load_subtree("zwear/param");
	/* Increment the power cycle */
	//iteration++;
	/* Save any changes */
	settings_save();
}

int storage_initialization(void)
{
	int rc = -1;
	
	rc = settings_subsys_init();
	if (rc) {
		LOG_ERR("settings subsys initialization: fail (err %d)", rc);
		return rc;
	}
	just_load_stuff();
	return rc;
}

int get_secret_key(char stored_key[16])
{
	strncpy(stored_key, secret_key_st, strlen(secret_key_st));
	return 0;
}

/** @} */