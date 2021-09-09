/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file util_test_settings.c

#include "util_test_settings.h"

/*******************************************************************************
 *** STUBS
 *******************************************************************************/
int settings_load_subtree(const char *subtree)
{
	return 0;
}

int settings_save(void)
{
	return 0;
}

/*******************************************************************************
 *** MOCKS
 *******************************************************************************/
static int stub_rc;
void set_stub_rc(void)
{
    stub_rc = 1;
}
void reset_stub_rc(void)
{
    stub_rc = 0;
}

int settings_subsys_init(void)
{
	return stub_rc;
}

/*******************************************************************************
 *** FAKES
 *******************************************************************************/
static char settings_name[30];

void get_settings_name(char *name)
{
    strncpy(name, settings_name, strlen(settings_name));
}

void set_settings_sec_key(void)
{
    strncpy(settings_name, "secret", strlen("secret"));
    settings_name[strlen("secret")] = '\0';
}


int settings_name_next(const char *name, const char **next)
{
    int rc = 0;

    while ((*name != SETTINGS_NAME_END)) {
        rc++;
        name++;
    }

    *next = NULL;

    return rc;
}

int settings_name_steq(const char *name, const char *key, const char **next)
{
    *next = NULL;

    if (!strncmp(name, key, strlen(name)))
    {
        return 1;
    }
    return 0;
}