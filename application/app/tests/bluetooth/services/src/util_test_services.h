/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_services.h

#ifndef _UTIL_TEST_SERVICES_H
#define _UTIL_TEST_SERVICES_H

/* Standard and Zephyr libraries */
#include <ztest.h>
#include <zephyr.h>
#include <stdint.h>
#include <logging/log.h>

#define CONFIG_BT_BAS_LOG_LEVEL LOG_LEVEL_ERR

#ifndef CONFIG_ARCH_POSIX
typedef int ssize_t;
#endif /* CONFIG_ARCH_POSIX */

/******************************************************************************
 *** MOCK FUNCTIONS UTILITIES
 ******************************************************************************/

void enable_notify(void);
void disable_notify(void);

void enable_param(void);
void disable_param(void);

void enable_indicate(void);
void disable_indicate(void);

/******************************************************************************
 *** FAKE
 ******************************************************************************/

struct bt_conn *connection_get(void);

#endif /* _UTIL_TEST_SERVICES_H */