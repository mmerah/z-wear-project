/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file authentication.c
 * @addtogroup authentication
 * @{
 */

#include "authentication.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(authentication, LOG_LEVEL_ERR);

void hmac_compute(const char *host_challenge, const char *secret,
                  uint16_t length_challenge, uint8_t *result)
{
    LOG_INF("Secret key is: %s and length is %d\n", secret, strlen(secret));

    char challenge[CHALLENGE_MAX_LENGTH] = {0};
    strncpy(challenge, host_challenge, length_challenge);

    LOG_INF("Challenge key is: %s and length is %d\n", challenge, strlen(challenge));

    /* Defines the tc_hmac_state_struct from tinycrypt. */
    struct tc_hmac_state_struct h;
    (void)memset(&h, 0x00, sizeof(h));

    /* Sets the secrey key */
    (void)tc_hmac_set_key(&h, (const uint8_t *)secret, strlen(secret));

    /* Initialize the HMAC manager and compute the result after 
    initializing the challenge. */
    (void)tc_hmac_init(&h);
    (void)tc_hmac_update(&h, (const uint8_t *)challenge, strlen(challenge));
    (void)tc_hmac_final(result, TC_SHA256_DIGEST_SIZE, &h);
}

/** @} */