/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_settings.c

#ifndef _UTIL_TEST_SETTINGS_H
#define _UTIL_TEST_SETTINGS_H

#include <zephyr.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SETTINGS_NAME_SEPARATOR '/'
#define SETTINGS_NAME_END '\0'

void set_stub_rc(void);
void reset_stub_rc(void);

void get_settings_name(char *name);

void set_settings_sec_key(void);

#endif /* _UTIL_TEST_SETTINGS_H */