/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/* ZTEST library */
#include <ztest.h>

/* Code tested */
#include "../../../src/ble/authentication/authentication.h"
#include "../../../src/ble/authentication/authentication.c"

/**
 * @brief Tests for the Authentication module
 *
 * Verify that the authentication module computes the
 * correct HMAC SHA256 results.
 *
 * @defgroup authentication_tests AUTH
 * @ingroup all_tests
 * @{
 * @}
 */

/************************************************
 *  ... STUBS
 ***********************************************/

/************************************************
 *  ... FAKES
 ***********************************************/

/************************************************
 *  ... MOCKS
 ***********************************************/

/************************************************
 *  ... UTILITIES
 ***********************************************/

/************************************************
 *  ... TESTS DECLARATIONS
 ***********************************************/

/**
 * @brief Test Authentication computing
 * @details test sha256 hmac compute, given a certain
 * secret key and challenge, the result given by the
 * module should be equal to expected.
 * @see hmac_compute()
 */
static void test_sha256_secret(void)
{
    char secret_key[16] = "aXs6h5cE9h0mDHr";
    char host_challenge[5] = "TEST";
    
    uint8_t result[32];
    uint8_t expected_result[32] = {0xc3, 0xda, 0x49, 0xeb, 0x01, 0x27, 0x9d, 0x8a, 0xc5, \
                                0x3f, 0x21, 0xf5, 0x4b, 0x22, 0x5c, 0x49, 0xa8, 0x09, \
                                0x21, 0x20, 0x13, 0xe2, 0xbd, 0x8c, 0x5c, 0xb3, 0xa5, \
                                0x3a, 0x9f, 0x53, 0xd4, 0xe2};
                                
	hmac_compute(host_challenge, secret_key, strlen(host_challenge), result);

	zassert_mem_equal(result, expected_result, 32, "Result MAC was not equal to expected");
}

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
    ztest_test_suite(test_hmac,
		ztest_unit_test(test_sha256_secret)
	);

	ztest_run_test_suite(test_hmac);
}