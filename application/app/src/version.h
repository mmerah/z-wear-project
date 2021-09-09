/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VERSION__H
#define VERSION__H

/* Stringify firmware build data included by cmake */
#define _xstr(s) _str(s)
#define _str(s) #s
#define TIME_OF_BUILD _xstr(CURRENT_TIME_OF_BUILD)
#define FW_VERSION _xstr(FW_BUILD)
#define SW_VERSION _xstr(SW_REVISION)

/* Stringify secret key data included by cmake */
#define SECRET_AUTH_KEY _xstr(SECRET_KEY)

#endif /* VERSION__H */