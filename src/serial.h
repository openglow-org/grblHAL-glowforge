/*
  serial.h - Grbl protocol stream over TCP or stdio

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
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

// --- line multiplexing, for the controller port (ctlport.c) -------------
//
// The core reads one stream. A second source of lines shares it by
// injecting whole lines into the RX ring between the sender's lines; the
// status of an injected line goes to its source and never to the sender,
// and the sender's held parser error (COMPATIBILITY_LEVEL 0) is put back
// after it, so the sender's response count and error state are what they
// would have been without the injection. Protocol thread only.

// Called with the status of the injected line, or -1 when the line was
// flushed unexecuted (a reset, a stop, a sender's jog cancel).
typedef void (*serial_inject_done_ptr)(int status);

// Hooks the status report. Once, from driver setup, after the core's
// report handlers are in place.
void serial_inject_init (serial_inject_done_ptr done);

// Queues one line (no terminator) behind what the ring already holds.
// False, with nothing queued: the sender is in the middle of a line, an
// injected line is still in flight, or the ring lacks the room.
bool serial_inject_line (const char *line);
bool serial_inject_busy (void);

// The read gate. While held, the core is handed no sender byte: the bytes
// stay in the ring, unread and unanswered, until the gate opens. An
// injected line behind them waits with them, and a cancel (ASCII_CAN)
// always passes.
void serial_hold_sender (bool hold);

// Sender bytes are waiting in the ring, and the CLOCK_MONOTONIC time the
// sender last completed a line (0 when it never has).
bool serial_sender_pending (void);

// The ring holds nothing but the sender's empty lines, which the core is
// about to answer: an injection refused now goes through a moment later.
bool serial_sender_empty_lines_only (void);
double serial_sender_last_line (void);
