/*
  serial.h - Grbl protocol stream over TCP or stdio

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "grbl/stream.h"

const io_stream_t *serialInit (void);

// Transport selection: a listening TCP socket fd (main.c owns it; must be
// non-blocking), or -1 for stdin/stdout.
void serial_set_listen_fd (int fd);

// Pump the transport: accept/drop clients, move bytes between the fds and
// the stream ring buffers, dispatch real-time commands. Runs on the grbl
// protocol thread (chained on grbl.on_execute_realtime and called from
// blocking delays).
void serial_poll (void);

// Pace the protocol loop: flush pending TX, then block until serial
// traffic needs service or the timeout expires. Wakes instantly on a new
// connection, client RX, or (while output is pending) TX writability, so
// a coarse idle timeout adds no input latency. Protocol thread only.
void serial_wait (long timeout_us);

// Client-session generation: bumps on every connect and disconnect, so a
// consumer can tell that the sender changed between two observations.
unsigned serial_client_generation (void);

// The sender session, for the published state file: whether a client is
// connected, its address ("" when none or unknown), and the
// CLOCK_MONOTONIC time it connected. Protocol thread only.
bool serial_client_connected (void);
const char *serial_client_peer (void);
double serial_client_since (void);
// An RX overrun happened since the last call: a sender wrote past the
// free count Bf: reports, the overrunning line was dropped whole and
// the lines after it are missing. The driver takes this once per event
// and aborts the job (protocol thread).
bool serial_rx_overflow_take (void);
