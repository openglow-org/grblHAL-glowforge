/*
  bind_addr_test.c - host unit test for the -b listen-address parser

  Part of grblHAL-glowforge. The controller binds one dual-stack AF_INET6
  socket. bind_addr_parse() (src/bind_addr.c) turns the -b literal into
  the address that socket binds:

    - an IPv6 literal binds as written (:: is in6addr_any, ::1 is
      in6addr_loopback)
    - an IPv4 literal binds in its v4-mapped form (::ffff:a.b.c.d), so
      127.0.0.1 is loopback on the same socket
    - anything that is not a literal is rejected: a host name, an empty
      string, a short dotted form, a bad hex group

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "bind_addr.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

/* The literal must parse and must equal `want`. */
static void expect_addr (const char *text, const struct in6_addr *want, const char *want_name)
{
    struct in6_addr got;
    char buf[INET6_ADDRSTRLEN];

    /* A stale pattern the parser must overwrite in full. */
    memset(&got, 0xa5, sizeof(got));

    if(bind_addr_parse(text, &got) != 0) {
        printf("FAIL: \"%s\": rejected, want %s\n", text, want_name);
        failures++;
        return;
    }
    if(memcmp(&got, want, sizeof(got)) != 0) {
        printf("FAIL: \"%s\": got %s, want %s\n", text,
               inet_ntop(AF_INET6, &got, buf, sizeof(buf)), want_name);
        failures++;
        return;
    }
    printf("ok:   \"%s\" -> %s\n", text, want_name);
}

/* The literal must be rejected with -1. */
static void expect_reject (const char *text)
{
    struct in6_addr got;

    if(bind_addr_parse(text, &got) != -1) {
        printf("FAIL: \"%s\": accepted, want -1\n", text);
        failures++;
        return;
    }
    printf("ok:   \"%s\" rejected\n", text);
}

/* Build an expected address from its canonical IPv6 text. */
static struct in6_addr addr (const char *text)
{
    struct in6_addr a;
    if(inet_pton(AF_INET6, text, &a) != 1) {
        printf("FAIL: test expectation \"%s\" is not an IPv6 literal\n", text);
        exit(EXIT_FAILURE);
    }
    return a;
}

int main (void)
{
    struct in6_addr a;

    /* ---- IPv6 literals bind as written ---- */

    expect_addr("::", &in6addr_any, "in6addr_any");
    expect_addr("::1", &in6addr_loopback, "in6addr_loopback");
    a = addr("fe80::1");
    expect_addr("fe80::1", &a, "fe80::1");

    /* ---- IPv4 literals bind in the v4-mapped form ---- */

    a = addr("::ffff:127.0.0.1");
    expect_addr("127.0.0.1", &a, "::ffff:127.0.0.1");
    a = addr("::ffff:0.0.0.0");
    expect_addr("0.0.0.0", &a, "::ffff:0.0.0.0");

    /* The mapped form is what the kernel recognizes as IPv4 on a
       dual-stack socket. */
    if(bind_addr_parse("127.0.0.1", &a) == 0 && IN6_IS_ADDR_V4MAPPED(&a))
        printf("ok:   \"127.0.0.1\" is IN6_IS_ADDR_V4MAPPED\n");
    else {
        printf("FAIL: \"127.0.0.1\" is not IN6_IS_ADDR_V4MAPPED\n");
        failures++;
    }

    /* ---- not a literal: rejected ---- */

    expect_reject("localhost");
    expect_reject("");
    expect_reject("1.2.3");
    expect_reject("::g");

    if(failures) {
        printf("FAIL: %d bind-address case(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS: bind-address literal parser holds\n");
    return EXIT_SUCCESS;
}
