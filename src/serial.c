/*
  serial.c - Grbl protocol stream over TCP or stdio

  Part of grblHAL-glowforge. The io_stream_t + ring-buffer shape is from
  the grblHAL Simulator's serial.c (Copyright (c) 2017-2025 Terje Io);
  the emulated-UART pump is replaced by serial_poll(), which moves bytes
  between real fds and the rings on the protocol thread.

  A client disconnect or write failure must never exit the process: the
  pulse-device fd is flock'd as the kernel dead man's switch, so dying on
  a UI disconnect would e-stop a running job. Drop the client, keep
  running, let it reconnect.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef _GNU_SOURCE     /* the build defines it globally; keep this for
                           standalone compilation */
#define _GNU_SOURCE     /* ppoll */
#endif

// grbl headers first: glibc's <sys/stat.h> (via fcntl.h) defines st_mtime
// as a macro, which must not be in scope when the core's vfs.h declares
// its struct field of the same name.
#include "serial.h"
#include "ctlport.h"
#include "driver.h"
#include "glowforge_laser.h"
#include "platform.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/gcode.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static int listen_fd = -1;
static int client_fd = -1;
static unsigned client_generation = 0;  /* bumps on every connect/disconnect */
static char client_peer[48];            /* the sender's address, "" when none */
static double client_since;             /* CLOCK_MONOTONIC at the accept */
static bool stdin_eof = false;  /* stdio mode: stop polling stdin at EOF */
static bool banner_pending = false; /* a sender connected: welcome it from a top-level poll */
static bool tx_blocked = false;     /* serialPutC is waiting on the ring: no nested writes */
static bool rx_discarding = false;  /* dropping the rest of an overrun line */
static bool rx_overrun = false;     /* an overrun happened, not yet taken */

/* Line multiplexing (serial.h). An injected line goes into an empty ring
 * only, so it is the next thing the core reads: inj_left is how many of
 * its bytes the core has yet to read, and any byte behind them is the
 * sender's. */
static bool sender_midline = false;     /* the sender's last stored byte was not an end of line */
static bool read_midline = false;       /* the last byte the core read was not an end of line */
static uint8_t sender_eol = '\n';       /* the sender's last end-of-line byte */
static double sender_last_line = 0;     /* CLOCK_MONOTONIC of it */
static bool inj_active = false;         /* an injected line is queued or executing */
static uint_fast16_t inj_left = 0;
static bool inj_first = false;          /* none of its bytes has been read yet */
static bool inj_status_due = false;     /* its last byte was read: the next status is its own */
static status_code_t inj_saved_error;   /* the sender's held parser error, put back after it */
static bool hold_sender = false;
static serial_inject_done_ptr inj_done = NULL;
static status_message_ptr status_message_chain = NULL;
static on_report_handlers_init_ptr report_handlers_init_chain = NULL;

static inline bool is_eol (uint8_t c)
{
    return c == '\n' || c == '\r';
}

/* Bytes rx_poll() reads from the client per call; serial_wait() only arms
 * client RX while the ring has room for a full read, so a sender that
 * ignores flow control gets paced by the timeout instead of spinning the
 * protocol loop. */
#define RX_CHUNK 64

void serial_set_listen_fd (int fd)
{
    listen_fd = fd;
}

static void serialRxFlush (void);

/* A displaced or dropped sender takes its partial line with it: the next
 * sender's first line must never be glued to a fragment of the last. */
static void drop_client (void)
{
    if(client_fd >= 0) {
        close(client_fd);
        client_fd = -1;
        client_peer[0] = '\0';
        client_generation++;
        serialRxFlush();
    }
}

unsigned serial_client_generation (void)
{
    return client_generation;
}

bool serial_client_connected (void)
{
    return client_fd >= 0;
}

const char *serial_client_peer (void)
{
    return client_peer;
}

double serial_client_since (void)
{
    return client_since;
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    /* The read gate holds the sender's bytes in the ring. An injected line
     * and a cancel pass it, and so does an empty line: an end-of-line byte
     * the core meets between lines. LightBurn polls '?' with an end of
     * line behind it, about twice a second; the core answers an empty line
     * ok in every state, the jog state included, and held back, those oks
     * would arrive in a burst when the jog ends. */
    if(hold_sender && !inj_left && rxbuffer.data[bptr] != ASCII_CAN &&
        !(is_eol(rxbuffer.data[bptr]) && !read_midline))
        return -1;

    data = (int32_t)rxbuffer.data[bptr++];          // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer
    read_midline = !is_eol((uint8_t)data);

    if(inj_left) {
        /* The core runs a line at its end-of-line byte, before it reads
         * on. So at the injected line's first byte the sender's last line
         * has run and its status stands in gc_state, and the status that
         * follows the injected line's last byte is the injected line's. */
        if(inj_first) {
            inj_first = false;
            inj_saved_error = gc_state.last_error;
        }
        if(--inj_left == 0)
            inj_status_due = true;
    }

    return data;
}

static inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
    rx_discarding = false;
    sender_midline = false;
    read_midline = false;               // the core flushes its line buffer on the same occasions

    /* An injected line goes with the rest, and its source is told. A line
     * whose run was aborted gets no status from the core, and the core
     * flushes on its way back up, so this also clears a status that will
     * never come. */
    if(inj_active) {
        inj_active = inj_status_due = false;
        inj_left = 0;
        if(inj_done)
            inj_done(-1);
    }
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

/* Zero-drain-progress bound before a stalled client is dropped. Output
 * must NEVER block the caller indefinitely: status reports are written
 * mid-motion, and a blocked write here would stall the protocol thread
 * for the length of a network stall while the machine keeps moving. On
 * a healthy link the ring drains in microseconds; a second with no
 * progress means the peer is gone or wedged (a busy access point or a
 * settings dump into a slow reader pauses for less). The drop bumps the
 * client generation, which disarms and holds a running laser job, so
 * the bound is a safety action and stays a whole second, not tighter. */
#define TX_STALL_MS 1000

static bool serialPutC (const uint8_t c)
{
    uint_fast16_t next_head;

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    if(txbuffer.tail == next_head) {                            // Buffer full...
        struct timespec t0, t;
        uint_fast16_t seen_tail = txbuffer.tail;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        tx_blocked = true;

        while(txbuffer.tail == next_head) {
            // hal.stream_blocking_callback -> protocol_execute_realtime ->
            // our realtime hook -> serial_poll() drains TX on this thread.
            if(!hal.stream_blocking_callback())
                return false;
            if(txbuffer.tail != seen_tail) {                    // progress: restart the clock
                seen_tail = txbuffer.tail;
                clock_gettime(CLOCK_MONOTONIC, &t0);
                continue;
            }
            clock_gettime(CLOCK_MONOTONIC, &t);
            if((t.tv_sec - t0.tv_sec) * 1000 +
               (t.tv_nsec - t0.tv_nsec) / 1000000 > TX_STALL_MS) {
                drop_client();                                  // stalled peer
                txbuffer.tail = txbuffer.head;                  // flush ring
                break;
            }
        }
        tx_blocked = false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    return true;
}

static void serialWriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount (void)
{
    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    return &stream;
}

/* --- fd transport pump --------------------------------------------------- */

static void rx_byte (uint8_t data)
{
    if(data == 0x06) {          // ^F: request clean shutdown
        driver_request_exit();
        return;
    }

    if((data == CMD_CYCLE_START_LEGACY || data == CMD_CYCLE_START) && gflaser_resume_gate())
        return;                             // a held laser job re-arms first; the gate issues the start

    if(enqueue_realtime_command(data))
        return;                             // real-time: never queued, never dropped

    /* A sender that ignores flow control (Bf: is the contract) has
     * overrun the ring. The line that overran is dropped WHOLE: what
     * the ring already holds of it is unwritten back to the last
     * newline, and the rest is discarded through its own newline, so
     * the parser never sees a fragment glued to the next line. The
     * overrun is latched for the driver, which aborts the job: lines
     * are missing, and a job with lines missing is not the job the
     * sender wrote. */
    if(rx_discarding) {
        if(data == '\n' || data == '\r')
            rx_discarding = false;
        return;
    }

    uint_fast16_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);
    if(bptr == rxbuffer.tail) {
        rxbuffer.overflow = 1;
        rx_overrun = true;
        rx_discarding = data != '\n' && data != '\r';
        while(rxbuffer.head != rxbuffer.tail) {
            uint_fast16_t last = (rxbuffer.head - 1) & (RX_BUFFER_SIZE - 1);
            if(rxbuffer.data[last] == '\n' || rxbuffer.data[last] == '\r')
                break;
            rxbuffer.head = last;           // unwrite the partial line
        }
        sender_midline = false;             // the ring ends on a line boundary again
        return;
    }

    rxbuffer.data[rxbuffer.head] = data;
    rxbuffer.head = bptr;

    if(is_eol(data)) {
        /* Only a line with something in it is the sender speaking. An end
         * of line with nothing before it is a keep-alive, the second half
         * of a CR LF pair, or the end of line behind a '?' poll (the '?'
         * itself never reaches the ring). */
        if(sender_midline) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            sender_last_line = (double)ts.tv_sec + ts.tv_nsec / 1e9;
        }
        sender_eol = data;
        sender_midline = false;
    } else
        sender_midline = true;
}

/* --- line multiplexing (serial.h) ---------------------------------------- */

static status_code_t inject_status_message (status_code_t status)
{
    if(inj_status_due) {
        inj_active = inj_status_due = false;
        /* The sender never saw this line, so it does not inherit its
         * error: with COMPATIBILITY_LEVEL 0 a held error would refuse the
         * sender's next g-code line and report this status to it. */
        gc_state.last_error = inj_saved_error;
        if(inj_done)
            inj_done((int)status);
        return status;
    }

    return status_message_chain(status);
}

/* The core puts its own report handlers back at every soft reset and then
 * calls this, which is where a status hook belongs. */
static void inject_report_handlers_init (void)
{
    if(report_handlers_init_chain)
        report_handlers_init_chain();

    status_message_chain = grbl.report.status_message;
    grbl.report.status_message = inject_status_message;
}

void serial_inject_init (serial_inject_done_ptr done)
{
    inj_done = done;
    if(status_message_chain == NULL) {
        report_handlers_init_chain = grbl.on_report_handlers_init;
        grbl.on_report_handlers_init = inject_report_handlers_init;
        status_message_chain = grbl.report.status_message;
        grbl.report.status_message = inject_status_message;
    }
}

bool serial_inject_busy (void)
{
    return inj_active;
}

bool serial_inject_line (const char *line)
{
    size_t len = strlen(line);

    /* An empty ring, and the sender between lines: then the core reads the
     * injected line next and whole. It ends with the sender's own
     * end-of-line byte, which leaves the core's CR/LF pairing where the
     * sender left it, so an empty line that follows is answered (or
     * skipped as the second half of a pair) exactly as it would have been. */
    if(inj_active || sender_midline || rx_discarding || len == 0 ||
        rxbuffer.head != rxbuffer.tail || len + 1 > (size_t)serialRxFree())
        return false;

    for(size_t i = 0; i <= len; i++) {
        rxbuffer.data[rxbuffer.head] = i < len ? (uint8_t)line[i] : sender_eol;
        rxbuffer.head = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);
    }
    inj_left = (uint_fast16_t)len + 1;
    inj_active = inj_first = true;
    inj_status_due = false;

    return true;
}

void serial_hold_sender (bool hold)
{
    hold_sender = hold;
}

bool serial_sender_pending (void)
{
    /* The sender's bytes are the ones behind an injected line. End-of-line
     * bytes alone are empty lines, and no claim on the machine. */
    uint_fast16_t p = (rxbuffer.tail + inj_left) & (RX_BUFFER_SIZE - 1);

    while(p != rxbuffer.head) {
        if(!is_eol(rxbuffer.data[p]))
            return true;
        p = (p + 1) & (RX_BUFFER_SIZE - 1);
    }

    return false;
}

bool serial_sender_empty_lines_only (void)
{
    return rxbuffer.head != rxbuffer.tail && !inj_left && !serial_sender_pending();
}

double serial_sender_last_line (void)
{
    return sender_last_line;
}

bool serial_rx_overflow_take (void)
{
    bool was = rx_overrun;
    rx_overrun = false;
    return was;
}

static void rx_poll (void)
{
    if(listen_fd >= 0) {

        // Always accept: a new connection displaces any current session
        // (last connection wins - single-operator machine, and a client
        // that died without FIN would otherwise hold the port forever).
        struct sockaddr_storage sa;
        socklen_t sl = sizeof(sa);
        int fd = accept4(listen_fd, (struct sockaddr *)&sa, &sl, SOCK_CLOEXEC);
        if(fd >= 0) {
            drop_client();
            client_peer[0] = '\0';
            if(sa.ss_family == AF_INET)
                inet_ntop(AF_INET, &((struct sockaddr_in *)&sa)->sin_addr,
                          client_peer, sizeof(client_peer));
            else if(sa.ss_family == AF_INET6)
                inet_ntop(AF_INET6, &((struct sockaddr_in6 *)&sa)->sin6_addr,
                          client_peer, sizeof(client_peer));
            {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                client_since = (double)ts.tv_sec + ts.tv_nsec / 1e9;
            }
            int flags = fcntl(fd, F_GETFL, 0);
            if(flags != -1)
                fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            // Disable Nagle: senders poll with single-byte '?' and
            // responses are small; Nagle + delayed ACK adds latency.
            int nodelay = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
            client_fd = fd;
            client_generation++;
            banner_pending = true;
        }

        if(client_fd >= 0) {
            uint8_t buf[RX_CHUNK];
            ssize_t n = read(client_fd, buf, sizeof(buf));
            if(n > 0) {
                for(ssize_t i = 0; i < n; i++)
                    rx_byte(buf[i]);
            } else if(n == 0)
                drop_client();              // client hung up; keep running
            else if(errno != EAGAIN && errno != EWOULDBLOCK &&
                    errno != EINTR)
                drop_client();              // hard error (reset etc.)
        }

    } else {
        uint8_t c = platform_poll_stdin();
        if(c && c != 0xFF)
            rx_byte(c);
        else if(c == 0xFF)
            stdin_eof = true;   /* stop arming stdin in serial_wait() */
    }
}

static void tx_drain (void)
{
    while(txbuffer.tail != txbuffer.head) {

        int fd = client_fd >= 0 ? client_fd : (listen_fd < 0 ? STDOUT_FILENO : -1);

        if(fd < 0) {
            // Port mode with no client connected: discard output so the
            // ring can never wedge the protocol thread.
            txbuffer.tail = txbuffer.head;
            break;
        }

        uint_fast16_t tail = txbuffer.tail;
        uint_fast16_t n = txbuffer.head >= tail ? txbuffer.head - tail : TX_BUFFER_SIZE - tail;

        ssize_t w = write(fd, &txbuffer.data[tail], n);
        if(w > 0)
            txbuffer.tail = (tail + (uint_fast16_t)w) & (TX_BUFFER_SIZE - 1);
        else if(w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            break;                          // socket full; retry next poll
        else {
            drop_client();                  // write error: drop, keep running
            if(listen_fd < 0)
                break;                      // stdout failed; nothing to do
        }
    }
}

/* The welcome banner goes to every sender that connects, the way a UART
 * build prints it at power-up: a sender that waits for "Grbl" before it
 * speaks would otherwise sit on a running controller in silence. It is
 * written from a top-level poll only. A poll can run inside a blocked
 * serialPutC (through the blocking callback), and a banner written from
 * there would land in the middle of the line that is waiting. */
static void welcome (void)
{
    if(banner_pending && !tx_blocked && client_fd >= 0) {
        banner_pending = false;
        grbl.report.init_message(serialWriteS);
        /* A new sender starts with a clean parser error state. With
         * COMPATIBILITY_LEVEL 0 the core holds every g-code line after a
         * line that errored, re-reporting that error until a blank line
         * acknowledges it (the streaming contract). A jog past the bed
         * now errors (soft limits are armed after a home), and the hold
         * outlives the connection - the core's last_error is not tied to
         * the client - so the next sender to connect would get error:15
         * on its first g-code line for a jog it never sent. This is that
         * sender's implicit acknowledgment: it did not send the erroring
         * line, so it does not inherit the hold. Written here, from a
         * top-level poll between lines (never mid-execution, like the
         * banner itself), so it cannot clobber a line in flight. */
        gc_state.last_error = Status_OK;
    }
}

void serial_poll (void)
{
    rx_poll();
    ctlport_poll();
    welcome();
    tx_drain();
}

void serial_wait (long timeout_us)
{
    if(timeout_us < 0)
        timeout_us = 0;
    struct timespec ts = { .tv_sec = timeout_us / 1000000,
                           .tv_nsec = (timeout_us % 1000000) * 1000 };
    struct pollfd fds[4];   /* the sender's listener and client, the controller port's */
    nfds_t n = 0;

    tx_drain();     /* flush this iteration's output before blocking */

    if(listen_fd >= 0) {
        fds[n].fd = listen_fd;
        fds[n].events = POLLIN;
        n++;
        if(client_fd >= 0) {
            fds[n].fd = client_fd;
            fds[n].events =
                (serialRxFree() > RX_CHUNK ? POLLIN : 0) |
                (txbuffer.tail != txbuffer.head ? POLLOUT : 0);
            n++;
        }
    } else if(!stdin_eof) {
        fds[n].fd = STDIN_FILENO;
        fds[n].events = POLLIN;
        n++;
    }
    n += ctlport_pollfds(&fds[n]);

    /* No fd to wait on (stdio mode at EOF), or a ppoll failure that is
     * not a signal wakeup: plain sleep so pacing can never become a
     * busy spin. */
    if(n == 0)
        nanosleep(&ts, NULL);
    else if(ppoll(fds, n, &ts, NULL) < 0 && errno != EINTR)
        nanosleep(&ts, NULL);
}
