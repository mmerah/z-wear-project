/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/**
 * @brief Tests for the storage settings module
 *
 * Verify that the module can store settings successfully
 *
 * @defgroup storage_settings_tests STOR
 * @ingroup all_tests
 * @{
 * @}
 */

/* ZTEST library */
#include <ztest.h>

/* Fake functions */
#include "util_test_settings.h"

/* Code tested */
#include "../../../../src/settings/settings_storage.h"
#include "../../../../src/settings/settings_storage.c"

/************************************************
 *  ... TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test storage initialization
 */
static void test_storage_init(void)
{
	reset_stub_rc();
	zassert_ok(storage_initialization(), "Storage not initialized correctly");
}

/**
 * @brief Test storage initialization failing
 */
static void test_storage_init_fail(void)
{
	set_stub_rc();
	zassert_false(storage_initialization() == 0, "Storage initialized but should not");
}

/**
 * @brief Test fetching secret key from storage
 */
static void test_get_secret_key(void)
{
	char fake_key[16];
	zassert_ok(get_secret_key(fake_key), "Could not fetch the secret key");
	/* Needs to add a verification to the values obtained ? */
}

ssize_t read_callback(void *cb_arg, void *data, size_t len)
{
	return 0;
}

/**
 * @brief Test handler set for secret key
 */
static void test_param_handle_set_sec_key(void)
{
	char fakename[30];
	set_settings_sec_key();
	get_settings_name(fakename);
	size_t fakelen = strlen(fakename);
	int fake_cb_arg = 0;
	zassert_ok(param_handle_set(fakename, fakelen, read_callback, fake_cb_arg),
				"Could not set the secret key");
}

/**
 * @brief Test handler set for secret key with incorrect length
 */
static void test_param_handle_set_sec_key_bad_len(void)
{
	char fakename[30];
	set_settings_sec_key();
	get_settings_name(fakename);
	size_t fakelen = strlen(fakename) + 30;
	int fake_cb_arg = 0;
	zassert_equal(param_handle_set(fakename, fakelen,
								   read_callback, fake_cb_arg),
				  0, "Secret key set with wrong length");
}

/**
 * @brief Test handler set for NULL parameter
 */
static void test_param_handle_set_null(void)
{
	char fakename[10] = "TEST";
	size_t fakelen = strlen(fakename);
	int fake_cb_arg = 0;
	zassert_equal(param_handle_set(fakename, fakelen, NULL, fake_cb_arg),
				  -ENOENT, "Not expected (param_handle_set)");
}

int export_callback(const char *name, const void *value, size_t val_len)
{
	return 0;
}

/**
 * @brief Test handler export
 */
static void test_param_handle_export(void)
{
	zassert_ok(param_handle_export(export_callback),
			   "Export handle did not finish as expected");
}

/**
 * @brief Test handler get for NULL parameter
 */
static void test_param_handle_get_null(void)
{
	char fakename[10] = "TEST";
	char fakeval[10] = "fake";
	int fakeval_len_max = 0;
	zassert_equal(param_handle_get(fakename, fakeval, fakeval_len_max), -ENOENT,
				  "Not expected (param_handle_export");
}

/**
 * @brief Test handler get for secret key
 */
static void test_param_handle_get_secret(void)
{
	char fakename[10] = "secret";
	char fakeval[10] = "fake";
	int fakeval_len_max = 0;
	zassert_ok(param_handle_get(fakename, fakeval, fakeval_len_max),
			   "Could not get secret key param");
}

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_storage,
					 ztest_unit_test(test_storage_init),
					 ztest_unit_test(test_storage_init_fail),
					 ztest_unit_test(test_get_secret_key),
					 ztest_unit_test(test_param_handle_set_sec_key),
					 ztest_unit_test(test_param_handle_set_sec_key_bad_len),
					 ztest_unit_test(test_param_handle_set_null),
					 ztest_unit_test(test_param_handle_export),
					 ztest_unit_test(test_param_handle_get_null),
					 ztest_unit_test(test_param_handle_get_secret));

	ztest_run_test_suite(test_storage);
}
