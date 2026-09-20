/*
  ctlport.h - the controller port: a local, non-displacing second channel

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <poll.h>

// Creates the port's socket and hooks the stream's line multiplexer. From
// driver setup, after the core's report handlers are in place.
void ctlport_init (void);

// Services the port. Called from serial_poll() (protocol thread).
void ctlport_poll (void);

// Adds the port's fds (at most two) to the stream's poll set. Returns how many.
unsigned ctlport_pollfds (struct pollfd *fds);
