/*
  platform.h - Linux platform interface (threads, clock, sleep, stdin)

  Part of grblHAL-glowforge. Descended from the grblHAL Simulator's
  platform layer (Copyright (c) 2014 Adam Shelly), reduced to the Linux
  implementation - this driver targets the Glowforge board's Linux
  userspace only.

  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <inttypes.h>
#include <pthread.h>

void platform_init (void);



uint8_t platform_poll_stdin (void); // non-blocking; 0 = no data, 0xFF = EOF
