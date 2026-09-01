/*
  serial_test.c - host unit test for the RX ring under a sender that
  ignores flow control

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later

  A sender that writes more than the RX ring holds (Bf: reports 1023
  characters free) overruns it. The ring must never hand the parser a
  mangled line: the line that overran is dropped whole (the part already
  in the ring is unwritten, the rest is discarded through its newline),
  every line before it is delivered intact, real-time characters keep
  passing while the ring is full, and the overrun is reported once
  through serial_rx_overflow_take() so the driver can abort the job.

  The transport pump is not exercised: the test pushes bytes into
  rx_byte() the way rx_poll() does and reads them back through the
  stream's read function.
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
bool stream_rx_suspend(stream_rx_buffer_t *rx, bool suspend)
{ (void)rx; (void)suspend; return false; }
bool stream_connected(void) { return true; }

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

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: an overrun drops the overrunning line whole, keeps every "
                      "earlier line, passes real-time characters and is reported once\n",
           failures);
    return failures ? 1 : 0;
}
