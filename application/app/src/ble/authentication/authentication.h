/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** 
 * @file authentication.h
 * @defgroup authentication Authentication: Process functions for SHA-256 based HMAC.
 * @brief Module for the authentication process.
 */

#ifndef _AUTHENTICATION_H
#define _AUTHENTICATION_H

/* Tinycrypt libraries */
#include <tinycrypt/sha256.h>
#include <tinycrypt/hmac.h>
#include <tinycrypt/constants.h>

/* Standard and Zephyr libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/types.h>
#include <sys/printk.h>

#define CHALLENGE_MAX_LENGTH        32

/**
 * @brief Hash-based Message Authentication Code computing.
 * This function computes the MAC using a host challenge received
 * through an encrypted BLE connection and the secret key stored
 * in a separate file.
 * @param[in] host_challenge Received host_challenge to be computed.
 * @param[in] secret Secret key used for computing the MAC
 * @param[in] length_challenge Size of the challenge.
 * @param[out] result Table with the 32 bytes-sized SHA-256 result.
 */
void hmac_compute(const char *host_challenge, const char *secret, uint16_t length_challenge, uint8_t *result);

#endif /* _AUTHENTICATION_H */