/*
  bind_addr.c - listen-address literal parser

  Part of grblHAL-glowforge. See bind_addr.h.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "bind_addr.h"

#include <arpa/inet.h>
#include <string.h>

int bind_addr_parse (const char *text, struct in6_addr *out)
{
    struct in_addr v4;
    struct in6_addr v6;

    if(text == NULL || out == NULL)
        return -1;

    if(inet_pton(AF_INET6, text, &v6) == 1) {
        *out = v6;
        return 0;
    }

    // inet_pton() accepts only the dotted-quad form, so "1.2.3" and a
    // host name both fall through to the rejection.
    if(inet_pton(AF_INET, text, &v4) == 1) {
        // v4-mapped: 80 zero bits, 16 one bits, then the IPv4 address in
        // network order.
        memset(out, 0, sizeof(*out));
        out->s6_addr[10] = 0xff;
        out->s6_addr[11] = 0xff;
        memcpy(&out->s6_addr[12], &v4.s_addr, sizeof(v4.s_addr));
        return 0;
    }

    return -1;
}
