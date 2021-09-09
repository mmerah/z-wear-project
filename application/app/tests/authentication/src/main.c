/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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