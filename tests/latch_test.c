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
  another thread's fault landing during one, and the kernel's byte
  counts: a pipe stands in for the ring, and the position read answers
  with what reached the pipe, counted apart from the engine, and with a
  processed count that the case sets. So the test proves what the
  shipper's run requests and its mid-run check promise:

  - the run-start retry's answer: an underrun met at a run start outside
    the armed window is acknowledged and the run asked for again, once,
    and the answer to that second request is the answer, read with the
    state after it: a retry refused on an empty ring reports nothing
    queued, never a run in flight, and at a stream's own run start,
    right after its first write, that answer faults the stream; an
    underrun that outlasts its acknowledgment, or a state that cannot be
    read, is a fault;
  - a run refused at idle other than on an empty ring faults the stream,
    and the reset clears the ring;
  - the deferred run and what ends it: a request waits while the kernel
    plays; a kernel that stops running under it while it owns bytes of
    the live stream (an underrun, or a stop another writer made) faults
    the stream with no run asked for, armed or not, and raises no second
    alarm for a stream another fault failed meanwhile; a stream that
    fails under the request (the armed clamp's stop) drops it and asks
    for no run; a request whose stream no longer owns the kernel asks
    for nothing;
  - the run gate: a stream's run start at a kernel that plays asks for
    no run, and its request waits and ends with the stream; at any other
    state but an underrun or a fault, a run starts only over exactly the
    bytes the stream wrote for it, the dark pad behind a lit end
    counted: a stop's residue at an idle or a disabled kernel (one whose
    ramp ended after the stream's end included), or bytes another writer
    added, fault the stream with no run asked for and no relight, and so
    does a count or a state that cannot be read; an underrun at a run
    start goes to the retry;
  - the mid-run check: a kernel that stops running while it owns bytes
    of the live stream (an underrun, or a stop another writer made)
    faults the stream;
  - one alarm per stream fault: a fault that another thread lands during
    a state read raised its own alarm, so the run request and the mid-run
    check raise no second one;
  - the dropped backlog: a reset, mid-motion or after a stream fault,
    leaves no step byte of the backlog it drops in the ring;
  - the reset during motion, in mc_reset's order (the core goes idle
    under the reset, the shipper can pass, then the reset): the kernel
    gets its stop and the residue its clear, whether or not the stream's
    run is still the kernel's, the backlog drops, and a stream behind a
    lit byte still ends dark; a reset that lands after the core's own
    idle ended the stream, before the cycle complete, gets the same stop,
    clear and read-back, and a reset while the device is handed over
    gets neither, and a second idle under the reset keeps the step the
    first one marked; once the ramp has ended, the core is owed
    the steps that never played (the step it counted for a pulse start
    the reset stopped, the backlog, and the residue from the kernel's
    step counters), and a count that cannot be read, or a counter zero
    that lands under the clear's read, loses the position; the null sink
    clears with no seek and owes the backlog alone.
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
static int  ring_rd = -1;           /* the read end of the pipe that stands in for the ring */
static uint32_t ring_took;          /* bytes that reached the stand-in */
static unsigned char ring_tail[2];  /* the last two of them */
static uint32_t k_processed;        /* bytes the kernel has played */
static uint32_t k_foreign;          /* bytes another writer added to the ring */
static int32_t k_pos[3];            /* the kernel's X, Y and Z step counters */
static bool pos_fail;               /* the position record cannot be read */
static void (*on_pos_read)(void);   /* another thread's work, landing after a position read */

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

static void ring_drain (void)
{
    unsigned char b[512];
    ssize_t n;
    while(ring_rd >= 0 && (n = read(ring_rd, b, sizeof(b))) > 0) {
        ring_took += (uint32_t)n;
        ring_tail[0] = n > 1 ? b[n - 2] : ring_tail[1];
        ring_tail[1] = b[n - 1];
    }
}

/* The kernel's position record: the step counters and the processed
 * count the case sets, and the tally of what the ring took, which is what
 * reached the stand-in and what another writer added. */
int gfio_rd_bin (const char *attr, void *buf, size_t len)
{
    unsigned char *p = buf;
    if(strcmp(attr, "cnc/position") || pos_fail || len < 32)
        return -1;
    ring_drain();
    uint32_t took = ring_took + k_foreign;
    memset(p, 0, 32);
    for(int i = 0; i < 4; i++) {
        for(int a = 0; a < 3; a++)
            p[4 * a + i] = (unsigned char)((uint32_t)k_pos[a] >> (8 * i));
        p[12 + i] = (unsigned char)(k_processed >> (8 * i));
        p[16 + i] = (unsigned char)(took >> (8 * i));
    }
    if(on_pos_read)
        on_pos_read();
    return 32;
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

/* The writes since the log held `from` of them. */
static int count_write_from (const char *what, int from)
{
    int n = 0;
    for(int i = from; i < wr_n && i < 64; i++)
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

/* A counter zero (a home, on the protocol thread) that lands right after
 * the shipper has read the kernel's step counters. */
static void zero_lands (void)
{
    gf_stream_clear_position();
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
    gf.hold_pending = false;
    gf.power_sent = false;
    gf.last_lit = false;
    gf.want_fire = false;
    gf.dith_left = 0;
    gf.written = gf.run_base = 0;
    gf.killed = false;
    gf.readback = RB_NONE;
    memset(gf.steps, 0, sizeof(gf.steps));
    memset(gf.base_steps, 0, sizeof(gf.base_steps));
    memset(gf.base_counts, 0, sizeof(gf.base_counts));
    memset(gf.unplayed, 0, sizeof(gf.unplayed));
    gf.base_valid = true;               /* the ring was empty at the start */
    k_processed = k_foreign = 0;
    memset(k_pos, 0, sizeof(k_pos));
    pos_fail = false;
    on_pos_read = NULL;
    latch_locked_by_us = true;
    latch_fail_reported = false;
}

/* The ring stand-in: a pipe whose write end is the pulse device, so what
 * the ring took is what reached it, counted apart from the engine. */
static void ring_open (void)
{
    int p[2];
    if(pipe(p) != 0 || fcntl(p[0], F_SETFL, O_NONBLOCK) != 0) {
        printf("FAIL: cannot open the ring stand-in\n");
        exit(1);
    }
    ring_rd = p[0];
    gf.fd = p[1];
    ring_took = 0;
    ring_tail[0] = ring_tail[1] = 0;
    gf.rate = 1000;
    gf.depth = 100;
}

static void ring_close (void)
{
    close(ring_rd);
    close(gf.fd);
    ring_rd = gf.fd = -1;
    gf.shipped = gf.produced = 0;
}

static uint32_t ring_total (void)
{
    ring_drain();
    return ring_took;
}

/* A stream that starts its run at an idle kernel and ends lit: the
 * preload, a stretch that the wakeup's re-asserted fire lights, and the
 * dark pad that the end writes behind it. The kernel then plays all but
 * `unplayed` of what the ring took. The caller opens the window and the
 * gate and asks for fire. */
static void stream_ended (uint32_t unplayed)
{
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    gf.ship_t0 -= 0.05;                 /* 50 more ticks are due */
    ship_pass();                        /* lit from the wakeup's base on */
    gf_stream_go_idle(false, 0, 0);
    ship_pass();                        /* the end: a dark pad behind the lit tick */
    k_processed = ring_total() - unplayed;
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
    CHECK(!gf.failed && !atomic_load(&fault_flag), "and faults nothing itself: its caller reads the answer");

    /* That answer at a stream's own run start, right after its first
     * write: the bytes just shipped are no longer in the kernel (played
     * as a continuation and run out, or cleared under the stream), so
     * the motion the core counted no longer matches the machine. A
     * fault; the stream ships and asks for nothing more until the
     * reset. */
    reset_state();
    int sink = open("/dev/null", O_WRONLY);
    gf.fd = sink;
    gf.rate = 1000;
    gf.depth = 100;
    gf_stream_wakeup();                 /* the live flag and one depth of pad slots */
    rd_state = "underrun";
    ring_empty = true;
    ship_pass();
    CHECK(gf.shipped >= gf.depth && gf.failed && atomic_exchange(&fault_flag, false),
          "a stream's run start whose retry finds the ring empty faults the stream");
    uint64_t at = gf.shipped;
    gf.produced = at + gf.depth + 30;   /* the producer goes on */
    ring_empty = false;
    ship_pass();
    CHECK(gf.shipped == at && count_write("cnc/run=1") == 2,
          "and the stream ships and asks for nothing more until the reset");
    close(sink);
    gf.fd = -1;
    gf.shipped = gf.produced = 0;

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

    /* A request whose stream no longer owns bytes in the kernel asks for
     * nothing, whatever the ring holds: the request dies with its
     * stream, and nothing here starts a run. */
    reset_state();
    gf.run_pending = true;
    rd_state = "idle";
    CHECK(pending_pass() && wr_n == 0 && !gf.run_pending && !gf.failed && !atomic_load(&fault_flag),
          "a request whose stream no longer owns the kernel is dropped and asks for nothing");

    /* A kernel stopped under a live stream's pending request: another
     * writer's stop (the cooling engine's safing stop) ramps to idle
     * with the rest of the stream in the ring, since the live flag turns
     * a run out of bytes into an underrun, never into idle. The stream
     * faults, with no run asked for over the stop's residue and no
     * relight of the lock this process wrote; the reset clears it. */
    reset_state();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf.run_pending = gf.kernel_running = gf.streaming = true;
    rd_state = "idle";
    CHECK(pending_pass() && gf.failed && atomic_exchange(&fault_flag, false) && !gf.run_pending,
          "a kernel stopped under a live stream's pending request faults the stream");
    CHECK(count_write("cnc/run=1") == 0 && latch_writes() == 0,
          "and asks for no run and no relight over the stop's residue");
    atomic_store(&laser_armed, false);
    atomic_store(&fire_gate, false);

    /* Still playing: the request waits and nothing is written. */
    reset_state();
    gf.run_pending = gf.kernel_running = true;
    rd_state = "running";
    CHECK(pending_pass() && gf.run_pending && gf.kernel_running && wr_n == 0,
          "a kernel still playing keeps the request pending and is asked nothing");

    printf("the run gate:\n");

    /* A run starts over exactly the bytes that the stream wrote for it:
     * the ring holds nothing older, the dark pad behind a lit end
     * included, and the kernel has played none of the stream's. */
    reset_state();
    ring_open();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf.want_fire = true;
    gf.want_power = 100;
    stream_ended(0);
    CHECK(count_write("cnc/run=1") == 1 && (ring_tail[0] & 0x10) && ring_tail[1] == 0x00 && !gf.failed,
          "a run start at an idle kernel with an empty ring starts, and its lit end gets a dark pad");
    gf_stream_wakeup();
    ship_pass();
    CHECK(count_write("cnc/run=1") == 2 && gf.kernel_running && !gf.failed && !atomic_load(&fault_flag),
          "the next run start over exactly its own bytes starts: the dark pad is counted");
    ring_close();

    /* A stop another writer made after the previous stream ended leaves
     * the rest of that stream in the ring. The next run start asks for no
     * run over it and does not relight a lock that this process wrote:
     * the stream faults, and the reset clears the ring. */
    const char *stopped[] = { "idle", "disabled" };
    for(int i = 0; i < 2; i++) {
        reset_state();
        ring_open();
        atomic_store(&laser_armed, true);
        atomic_store(&fire_gate, true);
        gf.want_fire = true;
        gf.want_power = 100;
        stream_ended(50);
        latch_locked_by_us = true;      /* a lock this process wrote inside the window */
        rd_state = stopped[i];
        int latches = latch_writes();
        gf_stream_wakeup();
        ship_pass();
        CHECK(gf.failed && atomic_exchange(&fault_flag, false) && count_write("cnc/run=1") == 1 &&
              latch_writes() == latches,
              i == 0 ? "a run start over a stop's residue faults the stream, with no run and no relight"
                     : "and the same at a disabled kernel");
        ring_close();
    }

    /* A count or a state that cannot be read, and bytes another writer
     * added, refuse the run too. */
    for(int i = 0; i < 3; i++) {
        reset_state();
        ring_open();
        if(i == 0)
            pos_fail = true;
        else if(i == 1)
            state_fail_at = 1;
        else
            k_foreign = 7;
        gf_stream_wakeup();
        ship_pass();
        CHECK(gf.failed && atomic_exchange(&fault_flag, false) && count_write("cnc/run=1") == 0,
              i == 0 ? "a run start whose byte counts cannot be read faults the stream, with no run"
              : i == 1 ? "a run start whose kernel state cannot be read faults the stream, with no run"
                       : "a run start over bytes another writer added faults the stream, with no run");
        ring_close();
    }

    /* An underrun at a run start is the retry's to answer: the previous
     * stream's tail ran out under this stream's live flag. */
    reset_state();
    ring_open();
    rd_state = "underrun";
    gf_stream_wakeup();
    ship_pass();
    CHECK(count_write("cnc/run=1") == 2 && find_write("cnc/stop=1") >= 0 && !gf.failed &&
          !atomic_load(&fault_flag), "an underrun at a run start goes to the retry, which starts the run");
    ring_close();

    /* A stop another writer made under a live stream, whose ramp ends
     * after the stream's end. The stream's run start met the kernel still
     * playing, so it asked for no run, and its request waited. The stream
     * ends inside the ramp, and its request with it. The ramp ends short,
     * with the residue in the ring, and nothing starts it; the next run
     * start finds it. */
    reset_state();
    ring_open();
    rd_state = "running";               /* the kernel plays the previous tail */
    gf_stream_wakeup();
    ship_pass();
    CHECK(count_write("cnc/run=1") == 0 && gf.run_pending && gf.kernel_running && !gf.failed,
          "a run start at a kernel that plays asks for no run, and its request waits");
    gf_stream_go_idle(false, 0, 0);     /* the stop ramps: the kernel still reads running */
    ship_pass();
    CHECK(!gf.run_pending && !gf.kernel_running && find_write("cnc/streaming=0") >= 0,
          "the stream's end ends its request");
    rd_state = "idle";                  /* the ramp ends short of the stream's end */
    k_processed = ring_total() - 40;
    ship_pass();
    CHECK(count_write("cnc/run=1") == 0 && !gf.failed && !atomic_load(&fault_flag),
          "the ramp's idle starts no run over the stop's residue");
    gf_stream_wakeup();
    ship_pass();
    CHECK(gf.failed && atomic_exchange(&fault_flag, false) && count_write("cnc/run=1") == 0,
          "and the next run start finds the residue: the stream faults, with no run");
    ring_close();

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

    /* With no request pending the same rule holds: a kernel idle while
     * it owns bytes of the live stream was stopped by another writer. */
    reset_state();
    gf.kernel_running = gf.streaming = true;
    rd_state = "idle";
    check_kernel_state();
    CHECK(gf.failed && atomic_exchange(&fault_flag, false),
          "a kernel stopped under a live stream with no request pending faults the stream");

    printf("the dropped backlog:\n");

    /* A reset drops the unshipped backlog, and the step bytes the
     * producer mapped ahead of the ship cursor go with it: their slots
     * hold nothing for a later stream that reaches those indices. */
    reset_state();
    gf.rate = 1000;
    gf.depth = 100;
    gf_stream_wakeup();                 /* a fresh stream: one depth of pad slots */
    gf.vticks = 20 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);        /* one X step ahead of the cursor */
    uint64_t slot = gf.produced - 1;
    bool mapped = gf.ring[slot & RING_MASK] != 0;
    gf_stream_reset();                  /* mid-motion */
    CHECK(mapped && gf.ring[slot & RING_MASK] == 0 && gf.produced == gf.shipped,
          "a reset mid-motion drops the backlog's step bytes with it");

    reset_state();
    gf_stream_wakeup();
    gf.vticks = 20 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x02, 0x02);        /* one Y step ahead of the cursor */
    slot = gf.produced - 1;
    mapped = gf.ring[slot & RING_MASK] != 0;
    gf.failed = true;                   /* a stream fault */
    gf_stream_go_idle(false, 0, 0);     /* and the core goes idle on its alarm */
    gf_stream_reset();                  /* the operator acknowledges it */
    CHECK(mapped && gf.ring[slot & RING_MASK] == 0 && gf.produced == gf.shipped,
          "a failed stream's reset drops the backlog's step bytes with it");

    printf("the reset during motion:\n");

    /* mc_reset's order: the core goes idle under the reset (EXEC_RESET is
     * set), the shipper can pass before the reset runs, then the reset.
     * The kernel plays the stream's run, so the reset stops it with its
     * ramp, clears the residue once the ramp has ended, and drops the
     * backlog with its step bytes. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    gf.vticks = 60 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);        /* one X step ahead of the cursor */
    slot = gf.produced - 1;
    mapped = gf.ring[slot & RING_MASK] != 0;
    rd_state = "running";               /* the kernel plays the stream */
    gf_stream_go_idle(true, 0, 0);      /* mc_reset's st_go_idle, under the reset */
    gf.ship_t0 = wall_s();              /* nothing more is due yet */
    ship_pass();                        /* the shipper passes before the reset */
    int before = wr_n;
    gf_stream_reset();
    CHECK(count_write_from("cnc/stop=1", before) == 1 && count_write_from("cnc/streaming=0", before) == 1,
          "a reset under which the core went idle mid-run stops the kernel with its ramp");
    CHECK(gf.clear_pending && gf.hold_pending && !gf.kernel_running && !gf.streaming,
          "and clears the residue once the ramp has ended");
    CHECK(mapped && gf.ring[slot & RING_MASK] == 0 && gf.produced == gf.shipped,
          "and drops the backlog with its step bytes");
    CHECK(!pending_pass() && gf.clear_pending && gf.readback == RB_WAIT,
          "while the kernel ramps, the clear waits and holds the shipper");
    ring_close();

    /* A reset that lands behind a lit byte: the stream still ends dark,
     * one pad behind it, before the clear removes the residue. */
    reset_state();
    ring_open();
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf.want_fire = true;
    gf.want_power = 100;
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    gf.ship_t0 -= 0.05;                 /* 50 more ticks are due */
    ship_pass();                        /* lit from the wakeup's base on */
    ring_total();
    bool lit = (ring_tail[1] & 0x10) != 0;
    rd_state = "running";
    gf_stream_go_idle(true, 0, 0);
    gf_stream_reset();
    bool held = !pending_pass();        /* the kernel ramps: the clear waits */
    ring_total();
    CHECK(lit && held && ring_tail[1] == 0x00 && gf.clear_pending,
          "a reset behind a lit byte ends the stream dark, one pad behind it, before the clear");
    atomic_store(&laser_armed, false);
    atomic_store(&fire_gate, false);
    ring_close();

    /* The same when the shipper ended the stream between the core's idle
     * and the reset: the stream's run is no longer the engine's, but the
     * kernel still plays it. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    gf_stream_go_idle(true, 0, 0);
    ship_pass();                        /* the stream's end: the run is no longer the engine's */
    before = wr_n;
    bool ended = !gf.kernel_running;
    gf_stream_reset();
    CHECK(ended && count_write_from("cnc/stop=1", before) == 1 && gf.clear_pending,
          "a stream that ended between the core's idle and the reset still gets the stop and the clear");
    ring_close();

    /* The core's own end-of-buffer idle ends the stream, and a reset
     * lands before the protocol thread takes the cycle complete: mc_reset
     * still counts it as a reset during motion and goes idle under it,
     * while the kernel plays the stream's tail. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    gf_stream_go_idle(false, 0, 0);     /* the core's own idle: the buffer ran dry */
    ship_pass();                        /* the stream's end */
    gf_stream_go_idle(true, 0, 0);      /* mc_reset's st_go_idle, under the reset */
    before = wr_n;
    gf_stream_reset();
    CHECK(count_write_from("cnc/stop=1", before) == 1 && gf.clear_pending &&
          gf.readback == RB_WAIT,
          "a reset that lands after the core's own idle ended the stream gets the stop, the clear "
          "and the read-back");
    ring_close();

    /* A reset during the web-service homing session: the device is the
     * session's, so the core's idle under the reset marks nothing. */
    reset_state();
    gf.suspended = true;
    gf_stream_go_idle(true, 0, 0);
    before = wr_n;
    gf_stream_reset();
    CHECK(count_write_from("cnc/stop=1", before) == 0 && !gf.clear_pending &&
          gf.readback == RB_NONE,
          "a reset while the device is handed over writes no stop and owes no read-back");
    gf.suspended = false;

    /* The read-back. The stream shipped 10 steps of +X, 5 of -Y and 3 of
     * +Z, and the backlog held 4 steps of -X. The core counted all of
     * them, and one more step of +Z for the pulse start that the reset
     * stopped. The ramp played 7 of +X, 5 of -Y and 1 of +Z: the kernel's
     * counters say so. The core is owed that step (1 Z), the backlog (-4
     * X) and the residue (3 X, 2 Z), so it stands where the machine
     * stopped: X 6 + 1, Y -5, Z 4 - 3. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    for(int i = 0; i < 10; i++) {
        gf.vticks = (uint64_t)i * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x00);
    }
    for(int i = 0; i < 5; i++) {
        gf.vticks = (uint64_t)(10 + i) * VTICKS_PER_BYTE;
        gf_stream_pulse(0x02, 0x02);
    }
    for(int i = 0; i < 3; i++) {
        gf.vticks = (uint64_t)(15 + i) * VTICKS_PER_BYTE;
        gf_stream_pulse(0x04, 0x00);
    }
    for(int i = 0; i < 4; i++) {
        gf.vticks = (uint64_t)(150 + i) * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x01);
    }
    ship_pass();                        /* the preload, and the run start */
    gf.ship_t0 -= 0.05;                 /* 50 more ticks are due: the steps */
    ship_pass();
    rd_state = "running";
    gf_stream_go_idle(true, 0x04, 0x00);    /* with the core's +Z step that never went out */
    gf_stream_reset();
    rd_state = "idle";                  /* the ramp has ended */
    k_pos[0] = 7;
    k_pos[1] = -5;
    k_pos[2] = 1;
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);   /* a ring the clear can seek */
    bool cleared = pending_pass() && !gf.clear_pending;
    int32_t owed[3] = { 0, 0, 0 };
    int rb = gf_stream_readback(owed, 0);
    CHECK(cleared && rb == 1 && owed[0] == -1 && owed[1] == 0 && owed[2] == 3,
          "once the ramp has ended, the clear runs, and the core is owed the step it never sent, "
          "the backlog it dropped, and the residue the ramp did not play");
    CHECK(gf.base_valid && gf.base_steps[0] == 10 && gf.base_counts[0] == 7 &&
          gf.base_steps[1] == (uint32_t)-5 && gf.base_counts[2] == 1 &&
          gf_stream_readback(owed, 0) == 0,
          "the cleared ring is the new base, and the read-back is taken once");
    ring_close();

    /* A count that cannot be read once the ramp has ended: the position
     * cannot be read back, and no base is left for the next one. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();
    rd_state = "running";
    gf_stream_go_idle(true, 0, 0);
    gf_stream_reset();
    rd_state = "idle";
    pos_fail = true;
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    CHECK(pending_pass() && gf_stream_readback(owed, 0) == -1 && !gf.base_valid,
          "a step count that cannot be read loses the position");
    ring_close();

    /* A counter zero that lands between the clear's read of the counters
     * and their use: the counts are stale. The position is lost, and the
     * zero's base stands. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();
    rd_state = "running";
    gf_stream_go_idle(true, 0, 0);
    gf_stream_reset();
    rd_state = "idle";
    k_pos[0] = 500;
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    on_pos_read = zero_lands;
    bool passed = pending_pass();
    on_pos_read = NULL;
    CHECK(passed && gf_stream_readback(owed, 0) == -1 && gf.base_valid && gf.base_counts[0] == 0,
          "a counter zero under the clear's read loses the position, and the zero's base stands");
    ring_close();

    /* The null sink takes every byte it is given: there is no residue
     * and no seek, so the clear lands at once and the shipper is not
     * held. The core is owed the backlog alone. */
    reset_state();
    gf.active = false;
    gf.fd = -1;
    gf.rate = 1000;
    gf.depth = 100;
    gf_stream_wakeup();
    for(int i = 0; i < 4; i++) {
        gf.vticks = (uint64_t)i * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x00);
    }
    for(int i = 0; i < 2; i++) {
        gf.vticks = (uint64_t)(150 + i) * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x00);
    }
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();
    gf_stream_go_idle(true, 0, 0);
    before = wr_n;
    gf_stream_reset();
    CHECK(count_write_from("cnc/stop=1", before) == 1,
          "the null sink's reset writes the stop the hardware would get");
    bool owed_clear = gf.clear_pending;
    CHECK(owed_clear && pending_pass() && !gf.clear_pending,
          "the null sink clears with no seek, so the shipper is not held");
    rb = gf_stream_readback(owed, 0);
    CHECK(rb == 1 && owed[0] == 2 && owed[1] == 0 && owed[2] == 0,
          "and the core is owed the backlog alone");
    gf.shipped = gf.produced = 0;

    /* Two idles under one reset: the first, with the stream live, marks
     * the step the core never sent, and the second, with the stream
     * ended, keeps it. */
    reset_state();
    gf.active = false;
    gf.fd = -1;
    gf.rate = 1000;
    gf.depth = 100;
    gf_stream_wakeup();
    gf_stream_go_idle(true, 0x01, 0x00);
    gf_stream_go_idle(true, 0, 0);
    gf_stream_reset();
    rb = pending_pass() ? gf_stream_readback(owed, 0) : 0;
    CHECK(rb == 1 && owed[0] == 1 && owed[1] == 0 && owed[2] == 0,
          "a second idle under the reset keeps the step the first one marked");
    gf.shipped = gf.produced = 0;

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
                      "a run starts only over the stream's own bytes, a run request takes the "
                      "retry's answer and dies with its stream, a kernel that stops under a "
                      "live stream faults it, a failed stream asks for no run, a stream "
                      "fault alarms once, and a reset during motion stops the kernel with its "
                      "ramp and owes the core the steps that never played\n",
           failures);
    return failures ? 1 : 0;
}
