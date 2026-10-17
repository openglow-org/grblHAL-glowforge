/*
  latch_test.c - host unit test for the laser latch writer and the kernel
  run requests

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

  The stub also models the kernel's answer to a run request (refused
  while it plays, in underrun or in fault; refused at idle with another
  errno, as a head sync that fails is refused with EIO; refused on an
  empty ring; a stop acknowledges an underrun), a stop write that does
  not take (issue_run ignores its return), a state read that fails, and
  another thread's fault landing during one, so the test proves what the
  shipper's run requests and its mid-run check promise:

  - the run-start retry's answer: an underrun met at a run start outside
    the armed window is acknowledged and the run asked for again, once,
    and the answer to that second request is the answer, read with the
    state after it: a retry refused on an empty ring reports nothing
    queued, never a run in flight, and an underrun that outlasts its
    acknowledgment, or a state that cannot be read, is a fault;
  - a run refused at idle other than on an empty ring faults the stream,
    and the reset clears the ring;
  - the deferred run and the underrun under it: a request the busy
    kernel refused waits while the kernel plays and starts once it reads
    idle; an underrun under it while the kernel owns bytes of the live
    stream faults the stream with no run asked for, armed or not, and
    raises no second alarm for a stream another fault failed meanwhile; a
    stream that fails under the request (the armed clamp's stop) drops it
    and asks for no run; a request left over from an ended stream takes
    the run start's retry; a deferred run refused on an empty ring ends
    the kernel run, so the next run leads with a power byte and a fresh
    dither;
  - the mid-run check: an underrun while the kernel owns bytes of the
    live stream faults the stream;
  - one alarm per stream fault: a fault that another thread lands during
    a state read raised its own alarm, so the run request and the mid-run
    check raise no second one.
*/
#include <errno.h>
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
static bool ring_empty;             /* the kernel's ring at a run request */
static int  idle_refusal;           /* errno of a run refused at idle, 0 = none */
static int  stops_ignored;          /* stop writes in underrun that do not take */
static int  state_reads, state_fail_at;     /* the cnc/state read that fails, 0 = none */
static void (*on_state_read)(void); /* another thread's work, landing during a read */

int gfio_wr_attr (const char *attr, const char *val)
{
    if(wr_n < 64)
        snprintf(wr_log[wr_n], sizeof(wr_log[0]), "%s=%s", attr, val);
    wr_n++;
    if(!strcmp(attr, "cnc/laser_latch") && latch_fail_left > 0) {
        latch_fail_left--;
        return -1;
    }
    /* The kernel's run request: refused while running, in underrun or
     * in fault (EPERM), refused at idle with idle_refusal when a case
     * sets it (the kernel refuses a head sync that fails with EIO),
     * refused on an empty ring (ENODATA), otherwise started. A stop
     * acknowledges an underrun; the first stops_ignored stop writes in
     * underrun do not take, and leave it in place. */
    if(!strcmp(attr, "cnc/run")) {
        if(!strcmp(rd_state, "running") || !strcmp(rd_state, "underrun") ||
           !strcmp(rd_state, "fault")) {
            errno = EPERM;
            return -1;
        }
        if(idle_refusal && !strcmp(rd_state, "idle")) {
            errno = idle_refusal;
            return -1;
        }
        if(ring_empty) {
            errno = ENODATA;
            return -1;
        }
        return 0;
    }
    if(!strcmp(attr, "cnc/stop") && !strcmp(rd_state, "underrun")) {
        if(stops_ignored > 0) {
            stops_ignored--;
            return -1;
        }
        rd_state = "idle";
    }
    return 0;
}

int gfio_rd_attr (const char *attr, char *buf, size_t len)
{
    if(!strcmp(attr, "cnc/state")) {
        /* A read that fails leaves the buffer as it was, as the real
         * reader does when the attribute does not open. */
        if(++state_reads == state_fail_at)
            return -1;
        snprintf(buf, len, "%s", rd_state);
        if(on_state_read)
            on_state_read();
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

static int count_write (const char *what)
{
    int n = 0;
    for(int i = 0; i < wr_n && i < 64; i++)
        if(!strcmp(wr_log[i], what))
            n++;
    return n;
}

/* A fault from another thread (the producer's armed clamp, a lock the
 * protocol thread lost) that lands while the shipper reads the kernel
 * state. It raised its own alarm, which the hook has taken already. */
static void fault_lands (void)
{
    gf.failed = true;
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
    ring_empty = false;
    idle_refusal = 0;
    stops_ignored = 0;
    state_reads = state_fail_at = 0;
    on_state_read = NULL;
    gf.run_pending = gf.kernel_running = gf.clear_pending = false;
    gf.streaming = false;
    gf.power_sent = false;
    gf.dith_left = 0;
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

    printf("the deferred run and the underrun:\n");

    /* An underrun found at a run start, outside the window, is
     * acknowledged and the run asked for again; the answer to that
     * second request is the answer. */
    reset_state();
    rd_state = "underrun";
    CHECK(issue_run() == 1, "a run start that finds an underrun acks it, and the retry starts the run");
    CHECK(wr_n == 3 && find_write("cnc/stop=1") == 1 && count_write("cnc/run=1") == 2,
          "the request, the acknowledgment, the retry");
    CHECK(!gf.failed && !atomic_load(&fault_flag), "a retry that starts the run faults nothing");

    reset_state();
    rd_state = "underrun";
    ring_empty = true;
    CHECK(issue_run() == 2, "a retry refused on an empty ring reports nothing queued, not a run in flight");
    CHECK(!gf.failed && !atomic_load(&fault_flag), "and faults nothing: the next ship restarts the stream");

    /* The run is asked for again once, never more: an underrun that
     * outlasts its acknowledgment (the stop write does not take) meets
     * the retry refused, a fault. */
    reset_state();
    rd_state = "underrun";
    stops_ignored = 1;
    CHECK(issue_run() == -1 && gf.failed && atomic_exchange(&fault_flag, false),
          "an underrun that outlasts its acknowledgment faults the stream at the retry");
    CHECK(count_write("cnc/run=1") == 2, "the run is asked for twice, never more");

    /* The retry's answer is read with the state after it: a state that
     * cannot be read is a fault, never the underrun read before. */
    reset_state();
    rd_state = "underrun";
    ring_empty = true;
    state_fail_at = 2;
    CHECK(issue_run() == -1 && gf.failed && atomic_exchange(&fault_flag, false),
          "a retry whose state cannot be read faults the stream");
    CHECK(count_write("cnc/stop=1") == 1,
          "and nothing acts on the underrun read before it: the acknowledgment is the one stop");

    /* A run refused at idle for any reason but an empty ring (the
     * kernel's EIO for a head sync that fails) leaves the shipped bytes
     * in the ring with no run to play them: a fault, and the reset then
     * clears the ring, so the next run start never plays them. */
    reset_state();
    idle_refusal = EIO;
    CHECK(issue_run() == -1 && gf.failed && atomic_exchange(&fault_flag, false),
          "a run refused at idle other than on an empty ring faults the stream");
    gf_stream_reset();
    CHECK(gf.clear_pending, "and the reset clears what the ring holds");

    /* The bytes behind a pending request were queued behind what the
     * kernel was playing: an underrun with a run pending starved this
     * stream mid-run, and the ring is drained. A fault, with no run
     * asked for over it, armed or not. */
    reset_state();
    gf.run_pending = gf.kernel_running = gf.streaming = true;
    rd_state = "underrun";
    ring_empty = true;
    CHECK(pending_pass(), "an underrun with a run pending does not hold the shipper");
    CHECK(gf.failed && atomic_exchange(&fault_flag, false), "an underrun with a run pending faults the stream");
    CHECK(!gf.run_pending, "the pending request is dropped");
    CHECK(count_write("cnc/run=1") == 0, "no run is asked for over the underrun");

    reset_state();
    atomic_store(&laser_armed, true);
    gf.run_pending = gf.kernel_running = gf.streaming = true;
    rd_state = "underrun";
    ring_empty = true;
    CHECK(pending_pass() && gf.failed && atomic_exchange(&fault_flag, false) && !gf.run_pending,
          "an underrun with a run pending faults the stream inside the window too");
    CHECK(count_write("cnc/run=1") == 0, "and asks for no run either");
    atomic_store(&laser_armed, false);

    /* A fault that lands while the shipper reads the state raised its
     * own alarm: the underrun under the request drops it, and the stream
     * alarms once. */
    reset_state();
    gf.run_pending = gf.kernel_running = gf.streaming = true;
    rd_state = "underrun";
    on_state_read = fault_lands;
    CHECK(pending_pass() && gf.failed && !gf.run_pending && !atomic_load(&fault_flag),
          "an underrun under a request whose stream failed meanwhile raises no second alarm");

    /* The same at a run request's refusal, and at its armed underrun. */
    reset_state();
    rd_state = "fault";
    on_state_read = fault_lands;
    CHECK(issue_run() == -1 && gf.failed && !atomic_load(&fault_flag),
          "a run refused for a stream that failed meanwhile raises no second alarm");

    reset_state();
    atomic_store(&laser_armed, true);
    rd_state = "underrun";
    on_state_read = fault_lands;
    CHECK(issue_run() == -1 && gf.failed && !atomic_load(&fault_flag),
          "an armed underrun at a run start for a stream that failed meanwhile raises no second alarm");
    atomic_store(&laser_armed, false);

    /* A stream that fails under a pending request: the armed clamp fault
     * stops the kernel, and the stop's ramp ends idle with what it did
     * not reach still in the ring. The request goes with the fault, and
     * the reset clears the residue. */
    reset_state();
    atomic_store(&laser_armed, true);
    gf.run_pending = gf.kernel_running = gf.streaming = true;
    rd_state = "running";
    pthread_mutex_lock(&gf.lock);
    clamp_locked();                     /* late events inside the window */
    pthread_mutex_unlock(&gf.lock);
    (void)atomic_exchange(&fault_flag, false);   /* the hook takes the alarm */
    rd_state = "idle";                  /* the ramp ended with the residue queued */
    CHECK(pending_pass() && !gf.run_pending, "a failed stream drops its pending request");
    CHECK(count_write("cnc/run=1") == 0, "and asks the kernel for no run over what the ring still holds");
    atomic_store(&laser_armed, false);

    /* A request left over from a stream that ended, while the kernel
     * drained its tail: the tail's end-of-data under the next stream's
     * live flag reads as an underrun before the next bytes land. Nothing
     * is lost, so it is the run start's case: acknowledged, the anchor
     * dropped, the retry's empty-ring answer ending the stale request. */
    reset_state();
    gf.run_pending = gf.streaming = true;
    gf.kernel_running = false;
    rd_state = "underrun";
    ring_empty = true;
    CHECK(pending_pass() && !gf.failed && !atomic_load(&fault_flag),
          "an underrun under a request left over from an ended stream is no fault");
    CHECK(!gf.run_pending && !gf.kernel_running, "the stale request ends with the retry's answer");
    CHECK(find_write("cnc/stop=1") == 1 && count_write("cnc/run=1") == 2,
          "the underrun was acknowledged and the run asked for again");

    /* A deferred run that finds the ring drained: the bytes played as a
     * continuation of the tail. That kernel run is over, so the next
     * run leads with a power byte and a fresh dither, like every run. */
    reset_state();
    gf.run_pending = gf.kernel_running = true;
    gf.power_sent = true;
    gf.dith_left = 5;
    rd_state = "idle";
    ring_empty = true;
    CHECK(pending_pass() && !gf.run_pending && !gf.kernel_running,
          "a deferred run refused on an empty ring ends the kernel run: the bytes were played");
    CHECK(!gf.power_sent && gf.dith_left == 0, "the next run leads with a power byte and a fresh dither");
    CHECK(!gf.failed && !atomic_load(&fault_flag), "it is no fault");

    /* A deferred run that starts: its leading power byte is already in
     * the ring, so the run state carries on. */
    reset_state();
    gf.run_pending = gf.kernel_running = true;
    gf.power_sent = true;
    rd_state = "idle";
    CHECK(pending_pass() && !gf.run_pending && gf.kernel_running && find_write("cnc/run=1") == 0,
          "a deferred run starts once the kernel reads idle");
    CHECK(gf.power_sent && !gf.failed, "its leading power byte is in the ring already; no fault");

    /* Still playing: the request waits and nothing is written. */
    reset_state();
    gf.run_pending = gf.kernel_running = true;
    rd_state = "running";
    CHECK(pending_pass() && gf.run_pending && gf.kernel_running && wr_n == 0,
          "a kernel still playing keeps the request pending and is asked nothing");

    printf("the mid-run check:\n");

    /* An underrun while the kernel owns bytes of the live stream is the
     * stream's, mid-run: a fault, and one alarm. */
    reset_state();
    gf.kernel_running = gf.streaming = true;
    rd_state = "underrun";
    check_kernel_state();
    CHECK(gf.failed && atomic_exchange(&fault_flag, false), "an underrun mid-run faults the stream");

    reset_state();
    gf.kernel_running = gf.streaming = true;
    rd_state = "underrun";
    on_state_read = fault_lands;
    check_kernel_state();
    CHECK(gf.failed && !atomic_load(&fault_flag),
          "an underrun mid-run for a stream that failed meanwhile raises no second alarm");

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
                      "latch it did not lock, and relights only its own lock under an open gate; "
                      "a run request takes the retry's answer, an underrun under a pending "
                      "run or mid-run faults the stream, a failed stream asks for no run, "
                      "and a stream fault alarms once\n",
           failures);
    return failures ? 1 : 0;
}
