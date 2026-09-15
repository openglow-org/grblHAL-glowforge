/*
  serial_test.c - host unit test for the Grbl stream: the RX ring under a
  sender that ignores flow control, the welcome banner on a connect, and
  the TX ring's stall bound

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  A sender that writes more than the RX ring holds (Bf: reports 1023
  characters free) overruns it. The ring must never hand the parser a
  mangled line: the line that overran is dropped whole (the part already
  in the ring is unwritten, the rest is discarded through its newline),
  every line before it is delivered intact, real-time characters keep
  passing while the ring is full, and the overrun is reported once
  through serial_rx_overflow_take() so the driver can abort the job.

  The RX cases push bytes into rx_byte() the way rx_poll() does and read
  them back through the stream's read function. The TX cases run the
  pump: a sender that connects to the listening socket is welcomed with
  the banner (from a top-level poll, never from inside a blocked write),
  a settings dump queues in the ring without waiting on the sender, a
  sender that pauses under a second keeps its connection, and one that
  pauses longer is dropped with the ring flushed.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* --- stubs the serial source links against -------------------------- */

static int realtime_seen;           /* bytes taken by the real-time path */

bool protocol_enqueue_realtime_command(uint8_t c)
{
    if(c == '?' || c == '!' || c == '~' || c == 0x18) {
        realtime_seen++;
        return true;
    }
    return false;
}

void driver_request_exit(void) {}
uint8_t platform_poll_stdin(void) { return 0; }
bool gflaser_resume_gate(void) { return false; }   /* no held laser job here */

/* --- driver source under test ---------------------------------------- */
#include "../src/serial.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
grbl_hal_t hal;
grbl_t grbl;
bool stream_rx_suspend(stream_rx_buffer_t *rx, bool suspend)
{ (void)rx; (void)suspend; return false; }
bool stream_connected(void) { return true; }
parser_state_t gc_state;        /* welcome() clears its last_error on a connect */

/* The core's banner, as the report module writes it through the stream. */
#define BANNER "\r\nGrblHAL test ['$' for help]\r\n"
static int banners;
static void fake_banner(stream_write_ptr write) { banners++; write(BANNER); }

/* The blocking callback the core gives a full ring: the real one runs
   the protocol loop's real-time pass, which polls the stream. The stand-in
   polls, and plays the sender: a peer that starts reading `peer_after_s`
   after the write blocked (never, when peer_reads is false). */
static int peer_rd = -1;
static bool peer_reads;
static double peer_after_s, blocked_at;
static int cb_calls;

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec / 1e9;
}

static bool blocking_cb(void)
{
    if(cb_calls++ == 0)
        blocked_at = now_s();
    if(peer_reads && now_s() - blocked_at >= peer_after_s) {
        char buf[512];
        (void)!read(peer_rd, buf, sizeof(buf));
    }
    serial_poll();
    struct timespec ts = { 0, 1000000 };
    nanosleep(&ts, NULL);
    return true;
}

static int listen_loopback(uint16_t *port)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = 0,
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    socklen_t sl = sizeof(sa);
    if(fd < 0 || bind(fd, (struct sockaddr *)&sa, sl) != 0 || listen(fd, 4) != 0 ||
       getsockname(fd, (struct sockaddr *)&sa, &sl) != 0)
        return -1;
    *port = ntohs(sa.sin_port);
    return fd;
}

static int connect_loopback(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(port),
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    if(fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
        return -1;
    return fd;
}

/* What a sender receives within `ms`; "" when nothing comes. */
static const char *received(int fd, int ms)
{
    static char buf[256];
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    size_t len = 0;
    while(len < sizeof(buf) - 1 && poll(&pfd, 1, ms) > 0) {
        ssize_t n = read(fd, buf + len, sizeof(buf) - 1 - len);
        if(n <= 0)
            break;
        len += (size_t)n;
        ms = 50;                         /* the rest of the same burst */
    }
    buf[len] = '\0';
    return buf;
}

/* Bytes serialPutC accepted; a drop ends the count. */
static int put_bytes(int n)
{
    int i = 0;
    while(i < n && serialPutC('x'))
        i++;
    return i;
}

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void push(const char *s)
{
    while(*s)
        rx_byte((uint8_t)*s++);
}

/* Read every complete line out of the ring. Returns the count; the
   lines land in `out`, and `bad` counts lines that are not one of the
   two shapes the test sent. */
static int drain(char out[][32], int max, int *bad, const char *a, const char *b)
{
    int n = 0, len = 0;
    char line[32];
    int16_t c;
    *bad = 0;
    while((c = serialGetC()) != SERIAL_NO_DATA) {
        if(c == '\n') {
            line[len] = '\0';
            if(n < max)
                snprintf(out[n], 32, "%s", line);
            if(strcmp(line, a) != 0 && strcmp(line, b) != 0)
                (*bad)++;
            n++;
            len = 0;
        } else if(len < 31)
            line[len++] = (char)c;
    }
    if(len)                          /* a partial line left in the ring */
        (*bad)++;
    return n;
}

int main(void)
{
    static char got[200][32];
    const char *line = "G1 X-30 F1500\n";      /* 14 bytes, as a fill sends them */
    const char *after = "G1 Y0.3 F1500\n";
    int bad;

    printf("RX ring under a sender that ignores flow control:\n");

    /* The ring takes RX_BUFFER_SIZE - 1 = 1023 bytes. 73 lines of 14
       bytes is 1022, one byte short: the 74th line's first byte fits and
       its second overruns, so the byte already in the ring has to be
       unwritten. Then the sender keeps going for two more lines, with a
       '?' in the middle of the overrun. */
    serialRxFlush();
    for(int i = 0; i < 73; i++)
        push(line);
    CHECK(serialRxFree() == 1, "73 lines of 14 bytes leave one byte free");
    CHECK(!serial_rx_overflow_take(), "no overrun reported while the ring merely fills");

    push("G1 X-3");                             /* the 74th line begins... */
    push("?");                                  /* a status poll during the overrun */
    push("0 F1500\n");                          /* ...and ends */
    push(after);                                /* the 75th line, whole */
    CHECK(realtime_seen == 1, "a real-time character passes while the ring is full");
    CHECK(serial_rx_overflow_take(), "the overrun is reported once");
    CHECK(!serial_rx_overflow_take(), "and only once");

    /* Make room and let the sender's next line in, so the read side sees
       what a sender feeding against Bf would deliver after the overrun. */
    int n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 73, "every line that fit is delivered");
    CHECK(bad == 0, "no line is mangled and no partial line is left behind");
    CHECK(n > 0 && strcmp(got[n - 1], "G1 X-30 F1500") == 0,
          "the last delivered line is the last one that fit, not a fragment");

    /* The lines after the overrun are dropped through the newline of the
       line that overran, and the ring is back to whole lines after it. */
    push(after);
    n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 1 && bad == 0 && strcmp(got[0], "G1 Y0.3 F1500") == 0,
          "a line sent after the overrun arrives whole");

    /* An overrun in the middle of a line whose start is already in the
       ring: that start is unwritten, so nothing can glue onto the next
       line. */
    serialRxFlush();
    for(int i = 0; i < 72; i++)
        push(line);                             /* 1008 bytes, 14 free */
    push("G1 X-30 F15");                        /* 11 bytes in, 3 free */
    push("00\n");                               /* the 3 fit: 1022, full */
    push("G1 Y0.3");                            /* 7 more: overrun on the first */
    push(" F1500\n");
    (void)serial_rx_overflow_take();
    n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 73 && bad == 0, "the overrun line's bytes never reach the parser");
    push(after);
    n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 1 && bad == 0, "the ring recovers to whole lines");

    /* --- the banner on a connect --------------------------------------- */
    printf("The welcome banner on a connect:\n");
    grbl.report.init_message = fake_banner;
    hal.stream_blocking_callback = blocking_cb;
    uint16_t port;
    int lfd = listen_loopback(&port);
    CHECK(lfd >= 0, "a listening socket on the loopback");
    serial_set_listen_fd(lfd);

    int sender = connect_loopback(port);
    CHECK(sender >= 0, "a sender connects");
    serial_poll();                                  /* accept, welcome, drain */
    CHECK(banners == 1, "the connect writes the banner once");
    CHECK(strcmp(received(sender, 500), BANNER) == 0, "the sender receives the banner unasked");
    serial_poll();
    CHECK(banners == 1 && received(sender, 100)[0] == '\0', "a later poll writes nothing more");

    /* A second connect displaces the first and is welcomed in turn - but
       not from a poll that runs inside a blocked write. */
    int sender2 = connect_loopback(port);
    tx_blocked = true;                              /* as serialPutC leaves it while it waits */
    serial_poll();
    CHECK(client_fd >= 0 && banners == 1 && received(sender2, 100)[0] == '\0',
          "a poll inside a blocked write accepts the sender but holds the banner");
    tx_blocked = false;
    serial_poll();
    CHECK(banners == 2 && strcmp(received(sender2, 500), BANNER) == 0,
          "the next top-level poll writes the banner to the new sender");
    CHECK(received(sender, 100)[0] == '\0', "the displaced sender gets nothing");
    close(sender);
    close(sender2);
    drop_client();

    /* --- a new sender starts with a clean parser error state ------------- */
    printf("A connect clears a held g-code error:\n");
    /* The core holds every g-code line after one that errored, until a
       blank line acknowledges it; the hold outlives the connection. A
       soft-limit-rejected jog leaves it. The sender that connects next
       never sent that line, so the banner clears it. */
    gc_state.last_error = 15;                       /* Status_TravelExceeded, from a prior jog */
    int sender3 = connect_loopback(port);
    CHECK(sender3 >= 0, "a fresh sender connects");
    serial_poll();                                  /* accept + welcome */
    CHECK(gc_state.last_error == Status_OK,
          "the connect cleared the held error so the new sender's first line is not refused");
    close(sender3);
    drop_client();
    serial_set_listen_fd(-1);
    close(lfd);

    /* --- the TX ring and its stall bound --------------------------------- */
    printf("The TX ring under a sender that stops reading:\n");
    CHECK(TX_BUFFER_SIZE >= 2048, "the ring holds 2048 bytes");

    /* The sender is a pipe with a 4 KiB capacity, already full: every
       byte from here on waits in the ring for the reader. */
    int pfd[2];
    CHECK(pipe2(pfd, O_NONBLOCK | O_CLOEXEC) == 0 && fcntl(pfd[1], F_SETPIPE_SZ, 4096) >= 0,
          "a pipe stands in for the sender's socket");
    {
        char fill[4096];
        memset(fill, 'f', sizeof(fill));
        while(write(pfd[1], fill, sizeof(fill)) > 0)
            ;
    }
    peer_rd = pfd[0];
    client_fd = pfd[1];
    txbuffer.head = txbuffer.tail = 0;

    /* A $$ dump is about 750 bytes on the machine; twice that queues
       without a single wait on the sender. */
    cb_calls = 0;
    peer_reads = false;
    CHECK(put_bytes(1500) == 1500 && cb_calls == 0 && serialTxCount() == 1500,
          "a 1500-byte report queues in the ring without waiting on the sender");

    /* The ring fills; a sender that resumes reading 900 ms later keeps
       its connection and every byte. */
    cb_calls = 0;
    peer_reads = true;
    peer_after_s = 0.9;
    int accepted = put_bytes(1200);
    double waited = now_s() - blocked_at;
    CHECK(accepted == 1200 && client_fd == pfd[1], "a 900 ms pause in the sender's reading drops nothing");
    CHECK(cb_calls > 0 && waited >= 0.9 && waited < 1.5,
          "the write waited for the sender, in the blocking callback");

    /* Drain everything the reader can take, then fill the ring again
       with a sender that reads only after 1100 ms: the bound drops it at
       one second, with the ring flushed. */
    {
        char buf[512];
        while(read(pfd[0], buf, sizeof(buf)) > 0)
            serial_poll();
        while(write(pfd[1], buf, sizeof(buf)) > 0)
            ;
    }
    cb_calls = 0;
    peer_after_s = 1.1;
    put_bytes(TX_BUFFER_SIZE);                      /* fills, then blocks on the last byte */
    waited = now_s() - blocked_at;
    CHECK(client_fd < 0, "a sender that pauses past the bound is dropped");
    CHECK(waited >= 1.0 && waited < 1.1, "the drop comes at the one-second bound");
    CHECK(serialTxCount() == 1, "the ring is flushed with the drop; only the byte that waited remains");
    close(pfd[0]);

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: an overrun drops the overrunning line whole, keeps every earlier "
                      "line, passes real-time characters and is reported once; a connect is "
                      "welcomed from a top-level poll; the ring holds a settings dump and "
                      "drops a sender only after a second without progress\n",
           failures);
    return failures ? 1 : 0;
}
