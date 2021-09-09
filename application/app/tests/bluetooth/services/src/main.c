/*
 * Copyright (c) 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file main.c

/**
 * @brief Tests for the ble services definitions
 *
 * Verify correct service definition
 *
 * @defgroup ble_services_tests SERV
 * @ingroup all_tests
 * @{
 * @}
 */

/* ZTEST library */
#include <ztest.h>

/************************************************
 *  ... TEST AUTHENTICATION SERVICE
 ***********************************************/
extern void test_auth_service_send(void);
extern void test_auth_service_send_bad_param(void);
extern void test_auth_service_send_not_sub(void);

extern void test_on_auth_received(void);

extern void test_cccd_changed_auth(void);
extern void test_cccd_changed_auth_disable(void);

/************************************************
 *  ... TEST MAIN
 ***********************************************/

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_services,
		ztest_unit_test(test_auth_service_send),
		ztest_unit_test(test_auth_service_send_bad_param),
		ztest_unit_test(test_auth_service_send_not_sub),
		ztest_unit_test(test_on_auth_received),
		ztest_unit_test(test_cccd_changed_auth),
		ztest_unit_test(test_cccd_changed_auth_disable)
	);

	ztest_run_test_suite(test_services);
}
