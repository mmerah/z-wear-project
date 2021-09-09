/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file settings_storage.h
 * @defgroup settings Storage: Non-volatile storage of settings functions.
 * @brief Module for the storage of various setting in NVS.
 */

#ifndef _SETTINGS_STORAGE_H
#define _SETTINGS_STORAGE_H

/* Standard and Zephyr libraries */
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <settings/settings.h>
#include <errno.h>
#include <sys/printk.h>

/* Versionning header */
#include "../version.h"

/**
 * @brief Settings API set handler.
 * Set value handler of settings items identified by keyword names.
 * @param[in] name Or key. The name with skipped part that was used as name in handler registration.
 * @param[in] len The size of the data found in the backend.
 * @param[in] read_cb Function provided to read the data from the backend.
 * @param[in] cb_arg Arguments for the read function provided by the backend.
 * @return 0 on success, non-zero on failure.
 */
int param_handle_set(const char *name, size_t len, settings_read_cb read_cb,
		  void *cb_arg);

/**
 * @brief Settings API export handler.
 * This gets called to dump all current settings items. This happens when settings_save tries to save the settings.
 * @param cb Or export_func. The pointer to the internal function which appends a single key-value pair to persisted settings. Don’t store duplicated value. The name is subtree/key string, val is the string with value.
 * @returns 0 on success, non-zero on failure.
 */
int param_handle_export(int (*cb)(const char *name,
			       const void *value, size_t val_len));

/**
 * @brief Settings API get handler.
 * Get values handler of settings items identified by keyword names.
 * @param[in] name Or key. The name with skipped part that was used as name in handler registration.
 * @param[out] val Buffer to receive value.
 * @param[in] val_len_max Size of that buffer.
 * @return length of data read on success, negative on failure.
 */
int param_handle_get(const char *name, char *val, int val_len_max);

/**
 * @brief Initialize the storage API.
 * @return rc on failure. 0 on success.
 */
int storage_initialization(void);

/**
 * @brief Gets the secret key.
 * @param[out] stored_key Secret Key fetched from the flash storage.
 * @return 0 if success.
 */
int get_secret_key(char stored_key[16]);

#endif /* _SETTINGS_STORAGE_H */