/*
  latch_test.c - host unit test for the laser latch writer and its owner

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  Every cnc/laser_latch write in the process goes through the stream
  engine's one writer. This test includes the stream source and stubs
  the sysfs layer to prove what the writer promises:

  - a lock that fails is retried, and a lock that still fails after the
    retries faults the stream, once per episode (the reset's
    acknowledgment opens a new one);
  - an unlock that fails returns false, faults nothing, and leaves the
    record locked;
  - an unlock of a latch this process already believes unlocked writes
    nothing: it would undo a lock someone else made;
  - the run-start relight undoes only a lock this process wrote inside
    an open window with the fire gate open: an armed window with the
    latch believed unlocked, a closed window, or a closed gate write no
    unlock at a run start;
  - the sideband (GFSINK_LATCH_LOG) carries one line per write.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

static char wr_log[64][40];         /* "attr=val" per sysfs write */
static int  wr_n;
static int  latch_fail_left;        /* latch writes that fail before one takes */
static const char *rd_state = "idle";

int gfio_wr_attr (const char *attr, const char *val)
{
    if(wr_n < 64)
        snprintf(wr_log[wr_n], sizeof(wr_log[0]), "%s=%s", attr, val);
    wr_n++;
    if(!strcmp(attr, "cnc/laser_latch") && latch_fail_left > 0) {
        latch_fail_left--;
        return -1;
    }
    return 0;
}

int gfio_rd_attr (const char *attr, char *buf, size_t len)
{
    if(!strcmp(attr, "cnc/state")) {
        snprintf(buf, len, "%s", rd_state);
        return 0;
    }
    if(len)
        buf[0] = '\0';
    return -1;
}

void gfio_set_hw (bool active) { (void)active; }
bool gfio_pulse_inherited (void) { return true; }
int  gfio_open_pulse_dev (const char *path) { (void)path; return -1; }
int  gfio_open_pulse_dev_nb (const char *path) { (void)path; return -1; }
void gfio_analog_config (void) {}
void gfio_currents_run (void) {}
void gfio_currents_hold (void) {}
unsigned gfio_xy_microsteps (void) { return 8; }
float gfio_conf_read_float (const char *k, float fb) { (void)k; return fb; }
int  gfio_conf_read (const char *k, char *v, size_t n) { (void)k; (void)v; (void)n; return -1; }
void gfhome_invalidate (void) {}
void gf_core_lock (void) {}
void gf_core_unlock (void) {}
void fflog (int prio, const char *fmt, ...) { (void)prio; (void)fmt; }

/* --- driver source under test ---------------------------------------- */
#include "../src/stepper_stream.c"

grbl_hal_t hal;

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static int latch_writes (void)
{
    int n = 0;
    for(int i = 0; i < wr_n && i < 64; i++)
        if(!strncmp(wr_log[i], "cnc/laser_latch=", 16))
            n++;
    return n;
}

static int find_write (const char *what)
{
    for(int i = 0; i < wr_n && i < 64; i++)
        if(!strcmp(wr_log[i], what))
            return i;
    return -1;
}

static void reset_state (void)
{
    wr_n = 0;
    latch_fail_left = 0;
    rd_state = "idle";
    atomic_store(&fault_flag, false);
    atomic_store(&laser_armed, false);
    atomic_store(&fire_gate, false);
    gf.failed = false;
    gf.active = true;                   /* sysfs writes reach the stub */
    latch_locked_by_us = true;
    latch_fail_reported = false;
}

int main (void)
{
    char path[] = "/tmp/latch_test.XXXXXX";
    int fd = mkstemp(path);
    if(fd < 0) {
        printf("FAIL: cannot create the sideband file\n");
        return 1;
    }
    latch_log_fd = fd;

    printf("the lock write and its retries:\n");

    reset_state();
    latch_fail_left = 2;
    CHECK(gf_stream_laser_latch(true), "a lock that takes on a retry succeeds");
    CHECK(latch_writes() == 3, "the failed writes were retried");
    CHECK(latch_locked_by_us, "the record says locked after the retry");
    CHECK(!atomic_load(&fault_flag) && !gf.failed, "a lock that took on a retry faults nothing");

    reset_state();
    latch_locked_by_us = false;
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    CHECK(!gf_stream_laser_latch(true), "a lock that fails after every retry reports failure");
    CHECK(latch_writes() == 1 + LATCH_RETRIES, "the lock was tried once plus the retries");
    CHECK(!latch_locked_by_us, "the record does not say locked after a failed lock");
    CHECK(atomic_exchange(&fault_flag, false) && gf.failed, "a failed lock faults the stream");
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    CHECK(!gf_stream_laser_latch(true) && !atomic_load(&fault_flag),
          "a second failure inside the episode is not reported again");
    gf_stream_reset();                  /* the operator acknowledges the fault */
    CHECK(!gf.failed, "the reset acknowledges the fault");
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    CHECK(!gf_stream_laser_latch(true) && atomic_exchange(&fault_flag, false),
          "a failure after the acknowledgment is a new episode and is reported");

    printf("the unlock write:\n");

    reset_state();
    CHECK(gf_stream_laser_latch(false), "an unlock of a latch this process locked writes");
    CHECK(latch_writes() == 1 && find_write("cnc/laser_latch=0") == 0, "the unlock wrote 0");
    CHECK(!latch_locked_by_us, "the record says unlocked");
    wr_n = 0;
    CHECK(gf_stream_laser_latch(false), "an unlock of a latch believed unlocked reports success");
    CHECK(latch_writes() == 0, "and writes nothing: it would undo someone else's lock");

    reset_state();
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    CHECK(!gf_stream_laser_latch(false), "an unlock that fails reports failure");
    CHECK(latch_locked_by_us, "the record still says locked after a failed unlock");
    CHECK(!atomic_load(&fault_flag) && !gf.failed, "a failed unlock faults nothing");

    printf("the run-start relight:\n");

    reset_state();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    CHECK(issue_run() == 1, "a run starts inside an open window");
    CHECK(find_write("cnc/laser_latch=0") == 0 && find_write("cnc/run=1") == 1,
          "a lock this process wrote inside the window is undone before the run");
    CHECK(!latch_locked_by_us, "the relight is recorded as an unlock");
    wr_n = 0;
    CHECK(issue_run() == 1 && latch_writes() == 0,
          "a run start with the latch believed unlocked writes no unlock");

    reset_state();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, false);
    CHECK(issue_run() == 1 && latch_writes() == 0,
          "a run start under a closed fire gate leaves the lock in place");

    reset_state();
    atomic_store(&laser_armed, false);
    atomic_store(&fire_gate, true);
    CHECK(issue_run() == 1 && latch_writes() == 0,
          "a run start outside the window writes no unlock");

    reset_state();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    CHECK(issue_run() == -1, "a run whose relight fails is refused");
    CHECK(find_write("cnc/run=1") < 0, "the refused run never reached the kernel");
    CHECK(atomic_exchange(&fault_flag, false) && gf.failed, "the refused run faults the stream");

    printf("the sideband:\n");

    close(fd);
    FILE *f = fopen(path, "r");
    char line[64];
    int locks = 0, unlocks = 0, skipped = 0, failed = 0;
    while(f && fgets(line, sizeof(line), f)) {
        if(!strcmp(line, "lock\n")) locks++;
        else if(!strcmp(line, "unlock\n")) unlocks++;
        else if(!strcmp(line, "unlock-skipped\n")) skipped++;
        else if(strstr(line, "failed")) failed++;
    }
    if(f)
        fclose(f);
    unlink(path);
    CHECK(locks >= 1 && unlocks >= 2 && skipped >= 1 && failed >= 4,
          "the sideband carries the locks, the unlocks, the skipped unlock and the failures");

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the latch writer retries, faults a lost lock, never re-opens a "
                      "latch it did not lock, and relights only its own lock under an open gate\n",
           failures);
    return failures ? 1 : 0;
}
