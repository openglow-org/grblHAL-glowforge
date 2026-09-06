/*
  bind_addr.h - listen-address literal parser

  Part of grblHAL-glowforge. The controller listens on one dual-stack
  AF_INET6 socket. bind_addr_parse() turns the -b option's literal into
  the address that socket binds.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef BIND_ADDR_H
#define BIND_ADDR_H

#include <netinet/in.h>

// Parse a listen-address literal. An IPv6 literal (::, ::1, fe80::1) is
// used as written. An IPv4 literal (0.0.0.0, 127.0.0.1) maps to its
// v4-mapped form (::ffff:a.b.c.d) so the same dual-stack socket serves
// it. Host names are not resolved. Returns 0 on success and -1 when the
// text is not an IPv4 or IPv6 literal; *out is defined only on success.
int bind_addr_parse (const char *text, struct in6_addr *out);

#endif
