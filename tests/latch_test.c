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
  - an unlock the kernel refuses because the cooling engine holds the
    latch is tried once, faults nothing, leaves the record locked, and
    is told apart from a failed write;
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
  - the preload and the live flag: a stream that starts while the kernel
    still plays the last one's tail preloads a depth less what the kernel
    still holds, to the byte, so the queue stays near a depth over any
    number of restarts, and never less than one pad; counts that cannot
    be read ship the whole depth; the count is read with the stream's
    lock released, and a fault or a reset that lands under the read ships
    nothing; the live flag is the shipper's, written once a chunk of a
    stream that has more to give is in the ring and before the run
    request, never at the wakeup, never for a stream with no more bytes
    to give or one that failed under that chunk's write, and once for a
    stream that takes over bytes shipped with no flag;
  - the mid-run check: a kernel that stops running while it owns bytes
    of the live stream (an underrun, or a stop another writer made)
    faults the stream;
  - the run-start re-assert: a laser state the core pushed while nothing
    streamed relights the next run at its first byte only inside the
    armed window, and a run that restarts while the last one's lit tail
    still ships relights at its first byte all the same; a run the core
    stops from off the producer forgets the wanted fire, so only a push
    made after it relights, and a run that ran out of segments on the
    producer keeps it for the rest of its block;
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
    clears with no seek and owes the backlog alone; a failed stream's
    reset during motion owes no read-back, and its residue clear still
    runs;
  - the reset at rest: a kernel that still plays an ended stream's tail
    gets the stop, the clear and the read-back, and so does what the
    shipper had not shipped of it; a read-back that succeeds raises no
    fault, and a tail that cannot be read back is a stream fault, where a
    reset during motion raises none; a reset between a stream's first
    chunk and its run request is followed by no run request; a kernel
    state that cannot be read is taken as playing; a kernel that reads
    underrun or fault under a stream the engine owns (a backlog, its run
    alone, or a stream a reset during motion ended) is a stream fault,
    with the stop and the live flag's 0 written and a read-back that
    answers that the position cannot be read back; the reset that
    acknowledges the shipper's own underrun alarm raises no second one; a
    reset's stop is written after a run request that is in the kernel,
    never inside it; a clear that has waited past the read-back's bound
    for a state that reads faults a new stream before a byte of it ships;
    a kernel with nothing playing and nothing owned, a device handed
    over, a live stream and a failed stream get what the cases say.

  And the motor currents and the clamp: the shipper only finds the hold
  currents owed (the null sink too), and the protocol thread takes the
  drop once, never over a stream started since; a handover writes an
  owed drop itself; a run current that does not take faults the stream
  before a byte ships, with one alarm; a stepper driver configuration
  that does not take refuses the resume, and every start after it until
  a write takes; an unarmed clamp faults nothing, and owes the homing
  anchor's drop once a stream, when one axis's late steps reach a full
  step (the mode's microsteps of X or Y, two lens half-steps) and not a
  step before, at x8, x16 and x32; a laser event's clamp counts no step.
*/
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

static char wr_log[64][40];         /* "attr=val" per sysfs write */
static int  wr_n;
static int  latch_fail_left;        /* latch writes that fail before one takes */
static bool tick_fails;             /* the machine tick (step_freq) write does not take */
static bool latch_engine_holds;     /* the kernel refuses an unlock: the cooling engine holds */
static const char *rd_state = "idle";
static bool ring_empty;             /* the kernel's ring at a run request */
static int  idle_refusal;           /* errno of a run refused at idle, 0 = none */
static int  stops_ignored;          /* stop writes in underrun that do not take */
static int  state_reads, state_fail_at;     /* the cnc/state read that fails, 0 = none */
static bool state_gone;             /* every cnc/state read fails */
static void (*on_state_read)(void); /* another thread's work, landing during a read */
static int  ring_rd = -1;           /* the read end of the pipe that stands in for the ring */
static uint32_t ring_took;          /* bytes that reached the stand-in */
static unsigned char ring_tail[2];  /* the last two of them */
static uint32_t k_processed;        /* bytes the kernel has played */
static uint32_t k_foreign;          /* bytes another writer added to the ring */
static int32_t k_pos[3];            /* the kernel's X, Y and Z step counters */
static bool pos_fail;               /* the position record cannot be read */
static void (*on_pos_read)(void);   /* another thread's work, landing after a position read */
static uint32_t live_at;            /* bytes the ring had taken when the live flag was last set */
static void (*on_run_write)(void);  /* another thread's work, landing while a run request is in the kernel */
static void ring_drain (void);

int gfio_wr_attr (const char *attr, const char *val)
{
    if(wr_n < 64)
        snprintf(wr_log[wr_n], sizeof(wr_log[0]), "%s=%s", attr, val);
    wr_n++;
    if(!strcmp(attr, "cnc/laser_latch") && latch_fail_left > 0) {
        latch_fail_left--;
        errno = EIO;
        return -1;
    }
    if(!strcmp(attr, "cnc/laser_latch") && !strcmp(val, "0") && latch_engine_holds) {
        errno = EBUSY;
        return -1;
    }
    if(!strcmp(attr, "cnc/streaming") && !strcmp(val, "1")) {
        ring_drain();
        live_at = ring_took;
    }
    if(!strcmp(attr, "cnc/step_freq") && tick_fails) {
        errno = EINVAL;
        return -1;
    }
    /* The kernel's run request: refused while running, in underrun or
     * in fault (EPERM), refused at idle with idle_refusal when a case
     * sets it (the kernel refuses a head sync that fails with EIO),
     * refused on an empty ring (ENODATA), otherwise started. A stop
     * acknowledges an underrun; the first stops_ignored stop writes in
     * underrun do not take, and leave it in place. */
    if(!strcmp(attr, "cnc/run")) {
        if(on_run_write) {
            void (*work)(void) = on_run_write;
            on_run_write = NULL;
            work();
        }
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
        if(++state_reads == state_fail_at || state_gone)
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

static unsigned char ring_buf[4096];   /* the first bytes the stand-in took, in order */

static void ring_drain (void)
{
    unsigned char b[512];
    ssize_t n;
    while(ring_rd >= 0 && (n = read(ring_rd, b, sizeof(b))) > 0) {
        for(ssize_t i = 0; i < n; i++)
            if(ring_took + (uint32_t)i < sizeof(ring_buf))
                ring_buf[ring_took + (uint32_t)i] = b[i];
        ring_took += (uint32_t)n;
        ring_tail[0] = n > 1 ? b[n - 2] : ring_tail[1];
        ring_tail[1] = b[n - 1];
    }
}

/* The tick bytes the ring took from stream tick `from` up to `to`, and
 * how many of them carry FIRE. A power byte (bit 7) is no tick. */
static int lit_ticks (uint32_t from, uint32_t to, int *ticks)
{
    uint32_t tick = 0;
    int lit = 0, n = 0;
    ring_drain();
    for(uint32_t i = 0; i < ring_took && i < sizeof(ring_buf); i++) {
        if(ring_buf[i] & 0x80)
            continue;
        if(tick >= from && tick < to) {
            n++;
            if(ring_buf[i] & 0x10)
                lit++;
        }
        tick++;
    }
    *ticks = n;
    return lit;
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
/* The stepper drivers' configuration and the motor currents: whether a
 * write takes, and how many were made. */
static bool config_fails;           /* a driver configuration write does not take */
static bool run_current_fails;      /* the X or Y run current does not take */
static int  run_currents, hold_currents, config_writes;
bool gfio_analog_config (void) { config_writes++; return !config_fails; }
bool gfio_currents_run (void) { run_currents++; return !run_current_fails; }
bool gfio_currents_hold (void) { hold_currents++; return true; }
static unsigned stub_mode = 32;     /* the microstep mode the controller's start read */
unsigned gfio_xy_microsteps (void) { return stub_mode; }
float gfio_conf_read_float (const char *k, float fb) { (void)k; return fb; }
int  gfio_conf_read (const char *k, char *v, size_t n) { (void)k; (void)v; (void)n; return -1; }
void gfhome_invalidate (void) {}
void gf_core_lock (void) {}
void gf_core_unlock (void) {}
void fflog (int prio, const char *fmt, ...) { (void)prio; (void)fmt; }
int  fflog_level (void) { return LOG_DEBUG; }

/* --- driver source under test ---------------------------------------- */
#define ship_pass ship_pass_under_test
#include "../src/stepper_stream.c"
#undef ship_pass

grbl_hal_t hal;

/* The producer as these cases model it: no producer thread runs, and the
 * case maps the events itself, so the producer counts as ahead of every
 * slot the shipper reaches (its covered cursor past them all), unless a
 * case stalls it and sets the cursor itself. */
static bool producer_stalled;

static void ship_pass (void)
{
    if(!producer_stalled)
        atomic_store(&gf.covered, UINT64_MAX);
    ship_pass_under_test();
}

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

/* A fault from another thread that lands while the shipper reads the
 * kernel's byte counts for a preload. */
static void fault_lands_under_read (void)
{
    on_pos_read = NULL;
    gf.failed = true;
}

/* A soft reset from the protocol thread that lands while the shipper
 * reads the kernel's byte counts for a preload. */
static void reset_lands_under_read (void)
{
    on_pos_read = NULL;
    gf_stream_reset();
}

/* A soft reset from the protocol thread that lands while the shipper
 * reads the kernel state for its run gate: after its first chunk, before
 * its run request. */
static void reset_lands_at_the_gate (void)
{
    on_state_read = NULL;
    gf_stream_reset();
}

/* A soft reset on the protocol thread that lands while the shipper's run
 * request is in the kernel: the reset runs on a thread of its own, and
 * the stub looks 10 ms later for a stop written inside the request. */
static pthread_t reset_tid;
static int stops_inside_the_run = -1;

static void *reset_thread (void *arg)
{
    (void)arg;
    gf_stream_reset();
    return NULL;
}

static void reset_lands_in_the_run (void)
{
    pthread_create(&reset_tid, NULL, reset_thread, NULL);
    struct timespec ts = { 0, 10000000L };
    nanosleep(&ts, NULL);
    stops_inside_the_run = count_write("cnc/stop=1");
}

/* A fault from another thread that lands while the shipper's write waits
 * on a full ring; the ring then takes the write. */
static void fault_lands_under_write (void)
{
    on_state_read = NULL;
    gf.failed = true;
    ring_drain();
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
    latch_engine_holds = false;
    latch_refused = false;
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
    state_gone = false;
    on_state_read = NULL;
    on_run_write = NULL;
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
    gf.readback_at_rest = false;
    gf.live_flag = false;
    atomic_store(&reset_stop, false);
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
    config_fails = run_current_fails = tick_fails = false;
    run_currents = hold_currents = config_writes = 0;
    gf.hold_due = false;
    gf.late_noted = 0;
    gf.suspended = false;
    atomic_store(&late_flags, 0);
    stub_mode = 32;
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
    CHECK(!gf_stream_latch_refused(), "a failed unlock is not a refusal");

    reset_state();
    latch_engine_holds = true;
    CHECK(!gf_stream_laser_latch(false), "an unlock the kernel refuses (the cooling engine holds) reports failure");
    CHECK(latch_writes() == 1, "and is tried once: the engine releases on its own clock");
    CHECK(gf_stream_latch_refused(), "the refusal is told apart from a failed write");
    CHECK(latch_locked_by_us && !atomic_load(&fault_flag) && !gf.failed,
          "the record still says locked, and the refusal faults nothing");

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
    gf_stream_wakeup();                 /* one depth of pad slots */
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
    CHECK(count_write("cnc/streaming=1") == 2 && count_write("cnc/streaming=0") == 1,
          "and each of the two streams wrote its own live flag, the first one's cleared at its end");
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

    /* An underrun at a run start is the retry's to answer: the kernel
     * still holds an underrun nothing has acknowledged. */
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

    printf("the preload and the live flag:\n");

    /* Twelve short streams, each started while the kernel still plays
     * the tail of the one before: the kernel plays 30 bytes between one
     * stream's end and the next one's start. Each preload tops the
     * kernel's queue up to one depth, so the queue stays near a depth
     * however many restarts there are, and a stop has no more than that
     * to play through. */
    reset_state();
    ring_open();
    uint32_t deepest = 0;
    for(int i = 0; i < 12; i++) {
        gf_stream_wakeup();
        ship_pass();                    /* the preload: what the queue is short of */
        rd_state = "running";
        gf.ship_t0 -= 0.02;             /* 20 more ticks are due */
        ship_pass();
        gf_stream_go_idle(false, 0, 0);
        ship_pass();                    /* the stream's end */
        uint32_t queued = ring_total() - k_processed;
        if(queued > deepest)
            deepest = queued;
        k_processed += queued < 30 ? queued : 30;
    }
    CHECK(deepest < 2 * gf.depth && !gf.failed && count_write("cnc/run=1") == 1,
          "a stream that starts while the kernel plays the last one's tail tops the queue up to "
          "one depth: twelve restarts leave it no deeper");
    ring_close();

    /* The preload's size, for a kernel that still holds 0, 1, a depth
     * less one, a depth, a depth and one, and five depths of the last
     * stream: the pads shipped at the first pass bring the queue to one
     * depth exactly, and a queue already a depth deep or more gets one
     * pad, so the stream owns bytes in the kernel from its first pass. */
    int pads = 0;
    bool sized = true;
    static const uint32_t tails[] = { 0, 1, 99, 100, 101, 500 };
    for(unsigned i = 0; i < sizeof(tails) / sizeof(tails[0]); i++) {
        reset_state();
        ring_open();
        gf.written = tails[i];          /* the ring took that many bytes, none played */
        k_foreign = tails[i];
        rd_state = tails[i] ? "running" : "idle";
        gf_stream_wakeup();
        ship_pass();
        (void)lit_ticks(0, UINT32_MAX, &pads);
        uint32_t want = tails[i] < gf.depth ? gf.depth - tails[i] : 1;
        if((uint32_t)pads != want || !gf.kernel_running || gf.failed ||
           gf.run_pending != (tails[i] != 0)) {
            printf("  a tail of %u: %d pads shipped, %u wanted\n", (unsigned)tails[i], pads,
                   (unsigned)want);
            sized = false;
        }
        ring_close();
    }
    CHECK(sized, "a preload is a depth less what the kernel still holds, to the byte, and one pad "
                 "for a queue a depth deep or more");

    /* Byte counts that cannot be read ship the whole depth. */
    reset_state();
    ring_open();
    gf.written = 40;
    k_foreign = 40;
    k_processed = 0;
    rd_state = "running";
    pos_fail = true;
    gf_stream_wakeup();
    ship_pass();
    pos_fail = false;
    (void)lit_ticks(0, UINT32_MAX, &pads);
    CHECK(pads == (int)gf.depth && gf.kernel_running && !gf.failed,
          "a preload whose byte counts cannot be read ships the whole depth");
    ring_close();

    /* The live flag is the shipper's, written once the run's first chunk
     * is in the ring and before the run is asked for: a last stream's
     * tail that ends between the wakeup and that chunk ends as a normal
     * end, never as an underrun. */
    reset_state();
    ring_open();
    live_at = 0;
    gf_stream_wakeup();
    bool early = find_write("cnc/streaming=1") >= 0;
    ship_pass();
    CHECK(!early && live_at > 0 && find_write("cnc/streaming=1") >= 0 &&
          find_write("cnc/streaming=1") < find_write("cnc/run=1"),
          "the live flag is written by the shipper once the run's first chunk is in the ring, "
          "never at the wakeup");
    ring_close();

    /* A stream that ended with nothing more to ship writes no flag: its
     * end of data is a normal end. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf_stream_go_idle(false, 0, 0);     /* ended before the shipper's first pass */
    ship_pass();
    CHECK(count_write("cnc/streaming=1") == 0 && count_write("cnc/run=1") == 1 && !gf.failed,
          "a stream with no more bytes to give writes no live flag");
    /* A second stream that starts before the shipper has ended that one
     * takes its bytes over as a continuation, where no chunk is a run's
     * first: it gets the flag at its own first chunk, once. */
    gf_stream_wakeup();
    bool taken_over = gf.kernel_running && gf.streaming;
    rd_state = "running";
    gf.ship_t0 -= 0.02;
    ship_pass();
    gf.ship_t0 -= 0.02;
    ship_pass();
    CHECK(taken_over && count_write("cnc/streaming=1") == 1 && !gf.failed,
          "a stream that takes over bytes shipped with no live flag writes the flag at its own "
          "first chunk, once");
    ring_close();

    /* A stream that ended before the shipper's first pass with a step
     * still to ship past its preload is live until that step ships: the
     * flag is written. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf.vticks = 5 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    CHECK(gf.shipped < gf.produced && count_write("cnc/streaming=1") == 1 && !gf.failed,
          "an ended stream with bytes still to ship writes the live flag");
    ring_close();

    /* A reset that lands under the preload's count read, with a tail in
     * the kernel: the pass ends there, and the stream the reset dropped
     * keeps nothing of the count (no pad skipped, nothing shipped). */
    reset_state();
    ring_open();
    gf.written = 40;
    k_foreign = 40;
    rd_state = "running";
    gf_stream_wakeup();
    on_pos_read = reset_lands_under_read;
    ship_pass();
    CHECK(on_pos_read == NULL && gf.shipped == 0 && gf.produced == 0 && ring_total() == 0 &&
          !gf.kernel_running && count_write("cnc/run=1") == 0,
          "a reset that lands under the preload's count read ends the pass with nothing shipped");
    ring_close();

    /* The preload's count is read with the stream's lock released: a
     * fault that lands under the read ends the pass, and nothing of the
     * failed stream ships. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    on_pos_read = fault_lands_under_read;
    ship_pass();
    CHECK(on_pos_read == NULL && gf.failed && ring_total() == 0 && !gf.kernel_running &&
          count_write("cnc/run=1") == 0,
          "a fault that lands under the preload's count read ships nothing of the stream");
    ring_close();

    /* A stream that fails under its first chunk's write (as one a reset
     * ends there) gets no live flag: the reset that follows writes a 0,
     * and a 1 behind it would stand at an idle kernel. The ring stand-in
     * is full, so the write waits, and the fault lands at its state
     * read. */
    reset_state();
    ring_open();
    if(fcntl(gf.fd, F_SETFL, O_NONBLOCK) == 0) {
        static const unsigned char fill[4096];
        while(write(gf.fd, fill, sizeof(fill)) > 0) { /* fill the ring */ }
    }
    pos_fail = true;                    /* the count read would empty the stand-in */
    gf_stream_wakeup();
    on_state_read = fault_lands_under_write;
    ship_pass();
    CHECK(on_state_read == NULL && gf.failed && gf.kernel_running &&
          count_write("cnc/streaming=1") == 0 && count_write("cnc/run=1") == 0,
          "a stream that fails under its first chunk's write gets no live flag and no run");
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

    printf("the run-start re-assert:\n");

    /* The core's laser state pushed while nothing streams (an S word, an
     * M3 at Idle) has no event to ride: the next run re-asserts it at its
     * first byte, and lights it only inside the armed window. With the
     * window closed and the per-tick gate open, so that the re-assert's
     * own mask is the only thing left to hold it, the run ships dark. */
    int ticks;
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&fire_gate, true);
    gf_stream_laser(100, true);         /* pushed at Idle: the wanted state only */
    gf_stream_wakeup();                 /* a fresh stream: its first byte is the depth */
    ship_pass();
    gf.ship_t0 -= 0.05;                 /* 50 more ticks are due */
    ship_pass();
    int relit = lit_ticks(100, 150, &ticks);
    CHECK(ticks == 50 && relit == 0, "a run start outside the window relights nothing the core wanted at Idle");
    ring_close();

    /* A run that restarts while the last one's tail still plays (a
     * continuation inside the producer's lead) relights at its first
     * byte: the last run's laser-off waits in the queue for that byte
     * while the shipper still plays the lit tail, and the re-assert goes
     * behind it, whatever the fire state at the cursor. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf_stream_laser(100, true);
    gf_stream_wakeup();
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();                        /* lit from the run's first byte, 100, to the cursor */
    gf.produced = gf.shipped + 30;      /* the producer leads the cursor */
    uint64_t next_base = gf.produced;
    gf_stream_go_idle(false, 0, 0);     /* the run ends: its laser-off at its last byte */
    gf_stream_wakeup();                 /* and the next starts inside the lead, at that byte */
    CHECK(gf.base == next_base, "the restart is a continuation at the ended run's last byte");
    gf.ship_t0 -= 0.05;
    ship_pass();
    int lit_tail = lit_ticks(100, (uint32_t)next_base, &ticks);
    CHECK(lit_tail == ticks && ticks > 0, "the ended run's tail ships lit to its last byte");
    relit = lit_ticks((uint32_t)next_base, (uint32_t)next_base + 20, &ticks);
    CHECK(ticks == 20 && relit == 20, "and the restarted run is lit from its first byte");
    uint32_t lit_now;
    CHECK(!gf_stream_lit_ticks(&lit_now), "a stream still in motion has not shipped all it will make");
    ring_close();

    /* A run the core stops from off the producer (st_reset's go-idle after
     * a canceled jog; this test's thread is not the producer) forgets the
     * wanted fire: the next run lights nothing the core wanted before the
     * end, and a laser state the core pushes after it lights the run after. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    uint32_t lit_base;
    (void)gf_stream_lit_ticks(&lit_base);
    gf_stream_laser(100, true);
    stream_ended(0);                    /* lit from its first byte, and its end */
    /* The count the laser module asks at an M5 (a window that fired
     * nothing closes there): every FIRE tick the ring took and only those,
     * final once the stream has shipped all it made. */
    int all_lit = lit_ticks(0, UINT32_MAX, &ticks);
    CHECK(gf_stream_lit_ticks(&lit_now) && all_lit > 0 && ticks > all_lit &&
          lit_now - lit_base == (uint32_t)all_lit,
          "the stream counts the FIRE ticks it shipped, final once it has shipped all it made");
    gf_stream_run_end();                /* the driver's go-idle, no reset, off the producer */
    lit_ticks(0, UINT32_MAX, &ticks);
    uint32_t t0 = (uint32_t)ticks;      /* the next stream's ticks follow these */
    gf_stream_wakeup();
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();
    relit = lit_ticks(t0 + 100, t0 + 150, &ticks);
    CHECK(ticks == 50 && relit == 0, "after a run end the next run relights nothing wanted before it");
    gf_stream_go_idle(false, 0, 0);
    gf_stream_run_end();
    ship_pass();
    k_processed = ring_total();
    gf_stream_laser(100, true);         /* the core pushes a laser-on at Idle */
    lit_ticks(0, UINT32_MAX, &ticks);
    t0 = (uint32_t)ticks;
    gf_stream_wakeup();
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();
    relit = lit_ticks(t0 + 100, t0 + 150, &ticks);
    CHECK(ticks == 50 && relit == 50, "and a push after the end lights the next run from its first byte");
    ring_close();

    /* A run that ran out of prepared segments on the producer (the planner
     * ran dry in the middle of a block) keeps the wanted fire: the core
     * sends the rest of that block no update of its own, and the next
     * run's first byte relights it. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf_stream_laser(100, true);
    stream_ended(0);
    pthread_t real_producer = gf.producer_tid;
    gf.producer_tid = pthread_self();   /* the go-idle comes from the producer */
    gf.threads_started = true;
    gf_stream_run_end();
    gf.threads_started = false;
    gf.producer_tid = real_producer;
    lit_ticks(0, UINT32_MAX, &ticks);
    t0 = (uint32_t)ticks;
    gf_stream_wakeup();
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();
    relit = lit_ticks(t0 + 100, t0 + 150, &ticks);
    CHECK(ticks == 50 && relit == 50, "a run that ran out on the producer keeps the wanted fire for the rest of its block");
    ring_close();

    /* A producer kept off the CPU past the lead and the depth: the shipper
     * reaches slots the producer has not covered. FIRE rides only the
     * slots below its covered cursor; the ones past it ship dark (the head
     * may have stopped there), with the fire request and the gate open. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf_stream_laser(100, true);
    gf_stream_wakeup();
    ship_pass();                        /* the preload: the depth's pads, dark */
    producer_stalled = true;
    /* The producer's cursor as its advance publishes it, stalled just
     * short of its next fire at slot 20: the slot that fire's step will
     * land in is not covered. */
    gf.vticks = 20 * VTICKS_PER_BYTE;
    gf.period_latched = VTICKS_PER_BYTE - 1;
    producer_advance();
    gf.period_latched = VTICKS_PER_BYTE;    /* the default, for the cases after */
    gf.ship_t0 -= 0.05;                 /* 50 ticks due past the base */
    ship_pass();
    producer_stalled = false;
    int past_ticks;
    int covered_lit = lit_ticks(100, 120, &ticks);
    int past_lit = lit_ticks(120, 150, &past_ticks);
    CHECK(ticks == 20 && covered_lit == 20 && past_ticks == 30 && past_lit == 0,
          "a stalled producer: FIRE on the 20 slots it covered, none on the next fire's slot or past it");
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    ring_close();

    /* A new stream's base below the last stream's cursor (a fresh stream's
     * index space starts over): the wakeup takes the cursor back to the new
     * base, so a producer that stalls at the run's start leaves every slot
     * past the base dark. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    producer_stalled = true;
    atomic_store(&gf.covered, 100000);  /* the last stream's producer, far along */
    gf_stream_laser(100, true);
    gf_stream_wakeup();
    ship_pass();
    gf.ship_t0 -= 0.05;
    ship_pass();
    producer_stalled = false;
    past_lit = lit_ticks(100, 150, &past_ticks);
    CHECK(past_ticks == 50 && past_lit == 0, "a new stream's stalled start: no FIRE past its base");
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    ring_close();

    printf("the stream's fire masks, each alone:\n");

    /* The enqueue mask: a laser-on the core sends while a stream runs is
     * queued dark with the window closed (or under a jog), however open
     * the per-tick gate is. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&fire_gate, true);
    gf_stream_wakeup();
    ship_pass();
    gf_stream_laser(100, true);         /* in the stream, at its base */
    gf.ship_t0 -= 0.05;
    ship_pass();
    relit = lit_ticks(100, 150, &ticks);
    CHECK(ticks == 50 && relit == 0, "a laser-on with the window closed and the gate open is queued dark");
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    ring_close();

    /* The per-tick gate: a fire the core asked for inside the armed window
     * ships dark on every tick the gate stands closed. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    gf.gate_seen = false;               /* closed from the start: no falling edge */
    gf_stream_wakeup();
    ship_pass();
    gf_stream_laser(100, true);
    gf.ship_t0 -= 0.05;
    ship_pass();
    relit = lit_ticks(100, 150, &ticks);
    CHECK(ticks == 50 && relit == 0, "an armed fire under a closed gate ships dark");
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    ring_close();

    /* The gate's falling edge: it clears the fire in flight, so a gate
     * that opens again lights nothing until the core asserts fire again. */
    reset_state();
    ring_open();
    gf.dith_period = 0;
    atomic_store(&laser_armed, true);
    atomic_store(&fire_gate, true);
    gf.gate_seen = true;
    gf_stream_wakeup();
    ship_pass();
    gf_stream_laser(100, true);
    gf.ship_t0 -= 0.02;
    ship_pass();                        /* 20 ticks lit */
    atomic_store(&fire_gate, false);
    ship_pass();                        /* the edge, on a pass that ships nothing */
    atomic_store(&fire_gate, true);
    gf.ship_t0 -= 0.03;
    ship_pass();
    int before_edge = lit_ticks(100, 120, &ticks);
    relit = lit_ticks(120, 150, &past_ticks);
    CHECK(before_edge == 20 && past_ticks == 30 && relit == 0,
          "a gate that closed and opened again: dark until the core asserts fire again");
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    ring_close();

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

    /* A stream fault that the hook has not taken, then a reset during
     * motion: the fault can have dropped steps the core counted, which
     * the read-back cannot see, so the reset owes none, and the ring is
     * still cleared once the kernel is idle. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();
    rd_state = "running";
    gf.failed = true;
    gf_stream_go_idle(true, 0, 0);      /* mc_reset's idle, under the reset */
    gf_stream_reset();
    rd_state = "idle";
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    CHECK(pending_pass() && !gf.clear_pending && gf_stream_readback(owed, 0) == 0,
          "a failed stream's reset owes no read-back, and the residue clear still runs");
    ring_close();

    printf("the reset at rest:\n");

    /* A reset after the stream ended, with the kernel still playing its
     * tail: the stream shipped 6 steps of +X and ended, and the core
     * went idle with all of them counted. The reset stops the kernel
     * with its ramp, which plays 2 of them, and clears the rest: the
     * core is owed the 4 that never played. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    for(int i = 0; i < 6; i++) {
        gf.vticks = (uint64_t)i * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x00);
    }
    ship_pass();                        /* the preload, and the run start */
    gf.ship_t0 -= 0.05;                 /* 50 more ticks are due: the steps */
    ship_pass();
    gf_stream_go_idle(false, 0, 0);     /* the core's own idle: the buffer ran dry */
    ship_pass();                        /* the stream's end */
    rd_state = "running";               /* the kernel plays the tail */
    before = wr_n;
    ended = !gf.kernel_running && !gf.streaming;
    gf_stream_reset();
    CHECK(ended && count_write_from("cnc/stop=1", before) == 1 && gf.clear_pending &&
          gf.readback == RB_WAIT,
          "a reset at rest stops a kernel that still plays the ended stream's tail");
    rd_state = "idle";                  /* the ramp has ended */
    k_pos[0] = 2;
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    cleared = pending_pass() && !gf.clear_pending;
    rb = gf_stream_readback(owed, 0);
    CHECK(cleared && rb == 1 && owed[0] == 4 && owed[1] == 0 && owed[2] == 0 &&
          !atomic_load(&fault_flag) && !gf.failed,
          "and the core is owed the steps of the tail that never played, with no fault raised");
    ring_close();

    /* The core was at rest and holds its position for true: a stopped
     * tail whose steps cannot be read back is a stream fault, so the hook
     * alarms and drops the reference. A reset during motion that cannot
     * be read back raises none: the core counts that position lost
     * itself. */
    for(int i = 0; i < 2; i++) {
        reset_state();
        ring_open();
        gf_stream_wakeup();
        ship_pass();                    /* the preload, and the run start */
        rd_state = "running";
        if(i == 0) {
            gf_stream_go_idle(false, 0, 0);
            ship_pass();                /* the stream's end; the kernel plays the tail */
        } else
            gf_stream_go_idle(true, 0, 0);
        gf_stream_reset();
        rd_state = "idle";
        pos_fail = true;
        close(gf.fd);
        gf.fd = open("/dev/null", O_WRONLY);
        rb = pending_pass() ? gf_stream_readback(owed, 0) : 0;
        CHECK(rb == -1 && atomic_exchange(&fault_flag, false) == (i == 0),
              i == 0 ? "a tail stopped at rest that cannot be read back is a stream fault"
                     : "and a reset during motion that cannot be read back raises none");
        ring_close();
    }

    /* The same for the last of an ended stream that the shipper has not
     * shipped yet: the reset drops it, and nothing of it plays after the
     * reset. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf.vticks = 150 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);        /* one X step */
    slot = gf.produced - 1;
    gf_stream_go_idle(false, 0, 0);     /* the stream ends before the shipper's first pass */
    gf_stream_reset();
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    cleared = pending_pass() && !gf.clear_pending;
    rb = gf_stream_readback(owed, 0);
    CHECK(gf.ring[slot & RING_MASK] == 0 && gf.produced == gf.shipped && cleared && rb == 1 &&
          owed[0] == 1,
          "a reset at rest drops what an ended stream had not shipped, and the core is owed it");
    ring_close();

    /* And for an ended stream whose bytes are in the ring with the
     * shipper's run request still to come: the kernel reads idle and
     * nothing is unshipped, but the engine still owns bytes there, which
     * a run would play after the reset. The reset lands in the run gate's
     * state read, between the stream's first chunk and its run request:
     * it stops and clears, and the shipper asks for no run behind the
     * stop. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf_stream_go_idle(false, 0, 0);     /* ended before the shipper's first pass */
    on_state_read = reset_lands_at_the_gate;
    ship_pass();
    CHECK(on_state_read == NULL && find_write("cnc/stop=1") >= 0 && gf.clear_pending &&
          !gf.kernel_running && count_write("cnc/run=1") == 0 && !gf.run_pending && !gf.failed &&
          !atomic_load(&fault_flag),
          "a reset at rest between a stream's first chunk and its run request stops and clears, "
          "and no run is asked for behind the stop");
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    cleared = pending_pass() && !gf.clear_pending && !atomic_load(&reset_stop);
    CHECK(cleared, "and the clear lifts the order, so the next stream's run is asked for");
    ring_close();

    /* A kernel state that cannot be read is taken as playing: the stop,
     * the clear and the read-back, which is made once the state reads
     * again and is a stream fault while it does not. */
    for(int i = 0; i < 2; i++) {
        reset_state();
        ring_open();
        gf_stream_wakeup();
        for(int s = 0; s < 6; s++) {
            gf.vticks = (uint64_t)s * VTICKS_PER_BYTE;
            gf_stream_pulse(0x01, 0x00);
        }
        ship_pass();                    /* the preload, and the run start */
        gf.ship_t0 -= 0.05;
        ship_pass();                    /* the steps */
        gf_stream_go_idle(false, 0, 0);
        ship_pass();                    /* the stream's end */
        rd_state = "running";
        state_fail_at = state_reads + 1;    /* the reset's read fails */
        before = wr_n;
        gf_stream_reset();
        bool taken = count_write_from("cnc/stop=1", before) == 1 && gf.clear_pending &&
                     gf.readback == RB_WAIT;
        close(gf.fd);
        gf.fd = open("/dev/null", O_WRONLY);
        if(i == 0) {
            rd_state = "idle";          /* the state reads again: the ramp has ended */
            k_pos[0] = 2;
            rb = pending_pass() ? gf_stream_readback(owed, 0) : 0;
            CHECK(taken && rb == 1 && owed[0] == 4 && !atomic_load(&fault_flag),
                  "a reset at rest that cannot read the kernel state stops, clears and reads back "
                  "as for a kernel that plays");
        } else {
            state_fail_at = state_reads + 1;    /* and the clear's read fails too */
            bool held = !pending_pass();
            rb = gf_stream_readback(owed, 0);
            CHECK(taken && held && rb == -1 && atomic_exchange(&fault_flag, false),
                  "and a state that stays unreadable ends in the stream fault at the reset");
        }
        gf.clear_pending = false;
        ring_close();
    }

    /* A kernel that reads underrun or fault under a stream the engine
     * still owns stopped dead or faulted under bytes the core counted,
     * and the shipper has not seen it. The reset raises the stream fault
     * itself and reads nothing back from counters that are not to be
     * trusted (the read-back answers that the position cannot be read
     * back, so the sender is told): its stop, which would acknowledge the
     * underrun, never passes for a stop of a kernel that played. Here the
     * engine owns the stream by its backlog. */
    static const char *const broken[] = { "underrun", "fault" };
    for(int i = 0; i < 2; i++) {
        reset_state();
        ring_open();
        gf_stream_wakeup();
        for(int s = 0; s < 6; s++) {
            gf.vticks = (uint64_t)s * VTICKS_PER_BYTE;
            gf_stream_pulse(0x01, 0x00);
        }
        ship_pass();                    /* the preload, the live flag and the run */
        rd_state = "running";
        gf.ship_t0 -= 0.003;
        ship_pass();                    /* three of the six steps ship */
        rd_state = broken[i];           /* the kernel ran dry, or faulted, unseen */
        k_pos[0] = 3;
        gf_stream_go_idle(false, 0, 0); /* the core's move ends */
        gf_stream_reset();              /* the soft reset, at rest */
        bool cleared_after = gf.clear_pending && !gf.kernel_running && !gf.failed;
        close(gf.fd);
        gf.fd = open("/dev/null", O_WRONLY);
        rd_state = "idle";
        (void)pending_pass();
        rb = gf_stream_readback(owed, 0);
        CHECK(cleared_after && rb == -1 && atomic_exchange(&fault_flag, false),
              i == 0 ? "a reset at rest over an underrun the shipper has not seen raises the stream "
                       "fault and reads nothing back"
                     : "and the same over a kernel fault");
        ring_close();
    }

    /* The same for a stream the engine owns by its backlog alone: it
     * ended before the shipper's first pass, under an underrun no stream
     * of the engine's has seen. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf.vticks = 150 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);        /* one X step */
    gf_stream_go_idle(false, 0, 0);
    ended = !gf.kernel_running && gf.produced > gf.shipped && !gf.streaming;
    rd_state = "underrun";
    gf_stream_reset();
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    (void)pending_pass();
    rb = gf_stream_readback(owed, 0);
    CHECK(ended && rb == -1 && atomic_exchange(&fault_flag, false),
          "and the same for a stream owned by its backlog alone");
    ring_close();

    /* The same for a stream the engine owns by its run alone: it ended,
     * every byte of it shipped, and the shipper's end pass has not run.
     * The reset writes the stop and the live flag's 0. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    for(int s = 0; s < 6; s++) {
        gf.vticks = (uint64_t)s * VTICKS_PER_BYTE;
        gf_stream_pulse(0x01, 0x00);
    }
    ship_pass();                        /* the preload, the live flag and the run */
    rd_state = "running";
    gf.ship_t0 -= 0.2;                  /* everything is due */
    ship_pass();
    gf_stream_go_idle(false, 0, 0);     /* ended, before the shipper's end pass */
    ended = gf.kernel_running && gf.produced <= gf.shipped && !gf.streaming;
    rd_state = "underrun";
    k_pos[0] = 3;
    before = wr_n;
    gf_stream_reset();
    bool stop_and_flag = count_write_from("cnc/stop=1", before) == 1 &&
                   count_write_from("cnc/streaming=0", before) == 1;
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    (void)pending_pass();
    rb = gf_stream_readback(owed, 0);
    CHECK(ended && stop_and_flag && rb == -1 && atomic_exchange(&fault_flag, false),
          "and the same for a stream owned by its run alone, with the stop and the live flag's 0 "
          "written");
    ring_close();

    /* And for a stream a reset during motion ended after the shipper's
     * end pass, which wrote the flag's 0 and let the run go without
     * reading the state, passed over an underrun on its last bytes. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf.vticks = 0;
    gf_stream_pulse(0x01, 0x00);
    ship_pass();
    rd_state = "running";
    gf.ship_t0 -= 0.2;
    ship_pass();
    gf_stream_go_idle(false, 0, 0);
    rd_state = "underrun";              /* it ran dry before the end pass */
    ship_pass();                        /* the end pass */
    ended = !gf.kernel_running;
    gf_stream_go_idle(true, 0, 0);      /* mc_reset: the core still counted a cycle */
    gf_stream_reset();
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    (void)pending_pass();
    rb = gf_stream_readback(owed, 0);
    CHECK(ended && rb == -1 && atomic_exchange(&fault_flag, false),
          "and the same for a stream a reset during motion ended");
    ring_close();

    /* The reset that acknowledges the shipper's own underrun alarm, with
     * the kernel still in the underrun, raises no second alarm. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "underrun";
    check_kernel_state();               /* the shipper's own alarm */
    bool alarmed = gf.failed && atomic_exchange(&fault_flag, false);
    gf_stream_go_idle(true, 0, 0);
    gf_stream_reset();
    CHECK(alarmed && !atomic_load(&fault_flag) && !gf.failed,
          "the reset that acknowledges an underrun alarm raises no second one");
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    (void)pending_pass();
    ring_close();

    /* A reset that lands while the shipper's run request is in the
     * kernel: its stop waits for the request (both are written under
     * latch_mx), so it ends the run the request started and never lands
     * inside the run start. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf_stream_go_idle(false, 0, 0);
    on_run_write = reset_lands_in_the_run;
    ship_pass();
    pthread_join(reset_tid, NULL);
    CHECK(stops_inside_the_run == 0 && count_write("cnc/run=1") == 1 &&
          find_write("cnc/stop=1") > find_write("cnc/run=1"),
          "a reset that lands while a run request is in the kernel writes its stop after it");
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    (void)pending_pass();
    ring_close();

    /* A clear that cannot run (the state cannot be read since the reset)
     * holds a new stream's bytes. Inside the read-back's bound they wait;
     * past it the stream is the stream fault, and nothing of it ships at
     * whatever later moment the state reads again. With no stream behind
     * it, the clear just waits. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    gf_stream_go_idle(false, 0, 0);
    ship_pass();                        /* the stream's end; the kernel plays the tail */
    state_gone = true;                  /* no state from here on */
    gf_stream_reset();                  /* taken as playing: the stop and the clear */
    uint32_t took = ring_total();
    gf.clear_at -= CLEAR_WAIT_S;
    ship_pass();                        /* past the bound, no stream */
    bool no_fault = !gf.failed && !atomic_load(&fault_flag);
    gf.clear_at += CLEAR_WAIT_S;        /* back inside the bound */
    gf_stream_wakeup();                 /* a new job */
    ship_pass();
    bool waited = gf.clear_pending && !gf.failed && !atomic_load(&fault_flag);
    gf.clear_at -= CLEAR_WAIT_S;        /* the bound passes */
    ship_pass();
    CHECK(no_fault && waited && gf.failed && atomic_exchange(&fault_flag, false) &&
          ring_total() == took && gf.shipped == 0,
          "a clear that cannot run past the read-back's bound faults a new stream before a byte "
          "of it ships");
    state_gone = false;
    gf.clear_pending = false;
    ring_close();

    /* The same for a kernel that still reads running past the bound: the
     * new stream is the stream fault, and nothing of it ships. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    gf_stream_go_idle(false, 0, 0);
    ship_pass();                        /* the stream's end; the kernel plays the tail */
    gf_stream_reset();                  /* the stop and the clear; the kernel keeps running */
    uint32_t took_running = ring_total();
    gf_stream_wakeup();                 /* a new job */
    ship_pass();
    bool waited_running = gf.clear_pending && !gf.failed && !atomic_load(&fault_flag);
    gf.clear_at -= CLEAR_WAIT_S;        /* the bound passes */
    ship_pass();
    CHECK(waited_running && gf.failed && atomic_exchange(&fault_flag, false) &&
          ring_total() == took_running && gf.shipped == 0,
          "a kernel still running past the read-back's bound faults a new stream before a byte "
          "of it ships");
    gf.clear_pending = false;
    rd_state = "idle";
    ring_close();

    /* A job whose producer ended inside the bound (its bytes all made,
     * none shipped) is the stream fault past it. The fault is raised once,
     * and a later reset keeps the clear's first time, so the next job in
     * the same state faults on its first pass. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    gf_stream_go_idle(false, 0, 0);
    ship_pass();                        /* the stream's end; the kernel plays the tail */
    state_gone = true;                  /* no state from here on */
    gf_stream_reset();                  /* taken as playing: the stop and the clear */
    uint32_t took_ended = ring_total();
    gf_stream_wakeup();                 /* a short job, ended inside the bound */
    gf.vticks = 0;
    gf_stream_pulse(0x01, 0x00);
    gf_stream_go_idle(false, 0, 0);
    ship_pass();
    bool held_ended = !gf.streaming && gf.produced > gf.shipped && !gf.failed &&
                      !atomic_load(&fault_flag);
    gf.clear_at -= CLEAR_WAIT_S;        /* the bound passes */
    ship_pass();
    bool faulted_ended = gf.failed && atomic_exchange(&fault_flag, false);
    CHECK(held_ended && faulted_ended && ring_total() == took_ended,
          "a job whose producer ended inside the bound is the stream fault past it");
    ship_pass();                        /* the next pass reports nothing */
    CHECK(!atomic_load(&fault_flag), "the overdue clear raises the stream fault once");
    double first_at = gf.clear_at;
    gf_stream_reset();                  /* the reset that acknowledges the fault */
    bool kept_first = gf.clear_at == first_at && gf.clear_pending;
    gf_stream_wakeup();                 /* the next job */
    ship_pass();
    CHECK(kept_first && gf.failed && atomic_exchange(&fault_flag, false),
          "a later reset keeps the clear's first time, so the next job faults on its first pass");
    state_gone = false;
    gf.clear_pending = false;
    ring_close();

    /* The states with nothing playing and nothing owned get nothing: an
     * idle kernel, a disabled one, and an underrun or a fault that no
     * stream of the engine's stands under (the next run start answers
     * those). */
    static const char *const quiet[] = { "idle", "disabled", "underrun", "fault" };
    bool nothing = true;
    for(int i = 0; i < 4; i++) {
        reset_state();
        rd_state = quiet[i];
        gf_stream_reset();
        if(wr_n != 0 || gf.clear_pending || gf.readback != RB_NONE || atomic_load(&fault_flag))
            nothing = false;
    }
    CHECK(nothing, "a reset at rest with nothing playing and nothing owned writes nothing, in "
                   "every kernel state that reads");

    /* A device handed to another process is not the engine's to stop:
     * with that process's run playing, or with a state that cannot be
     * read, the reset writes nothing and orders no clear. */
    for(int i = 0; i < 3; i++) {
        reset_state();
        gf.suspended = true;
        gf.kernel_running = true;       /* whatever the engine's own record says */
        rd_state = i == 2 ? "underrun" : "running";
        if(i == 1)
            state_fail_at = 1;
        gf_stream_reset();
        CHECK(wr_n == 0 && !gf.clear_pending && gf.readback == RB_NONE && !gf.failed &&
              !atomic_load(&fault_flag),
              i == 0 ? "a reset with the device handed over and its kernel running writes nothing"
              : i == 1 ? "and the same when the kernel state cannot be read"
                       : "and the same when it reads underrun");
        gf.suspended = false;
        gf.kernel_running = false;
    }

    /* A stream still live with no idle under the reset gets the stop
     * while its run is the kernel's, and no read-back: its producer can
     * still be armed. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    rd_state = "running";
    before = wr_n;
    gf_stream_reset();
    CHECK(count_write_from("cnc/stop=1", before) == 1 && gf.clear_pending &&
          gf.readback == RB_NONE,
          "a reset over a live stream with no idle under it stops the kernel and owes no read-back");
    /* The reset wrote the flag's 0: the next stream writes its own 1. */
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    cleared = pending_pass() && !gf.clear_pending;
    before = wr_n;
    gf_stream_wakeup();
    ship_pass();
    CHECK(cleared && count_write_from("cnc/streaming=1", before) == 1,
          "and the stream after that reset writes its live flag again");
    ring_close();

    /* A failed stream that stopped shipping has a backlog, so the reset
     * that acknowledges it is a reset at rest over a tail: it owes no
     * read-back (the fault can have dropped steps the read-back cannot
     * see) and raises no second alarm. */
    reset_state();
    ring_open();
    gf_stream_wakeup();
    ship_pass();                        /* the preload, and the run start */
    gf.vticks = 150 * VTICKS_PER_BYTE;
    gf_stream_pulse(0x01, 0x00);        /* one X step, not yet due */
    gf.failed = true;                   /* a stream fault, its alarm taken by the hook */
    gf_stream_go_idle(false, 0, 0);
    rd_state = "running";
    gf_stream_reset();
    close(gf.fd);
    gf.fd = open("/dev/null", O_WRONLY);
    rd_state = "idle";
    pos_fail = true;                    /* a read-back, were one owed, would fail and alarm */
    (void)pending_pass();
    rb = gf_stream_readback(owed, 0);
    CHECK(rb == 0 && !atomic_load(&fault_flag) && !gf.failed && !gf.clear_pending,
          "a failed stream's reset at rest owes no read-back and raises no second alarm");
    ring_close();

    printf("the motor currents:\n");

    /* The shipper finds the hold currents owed once the kernel has played
     * the stream out, and writes no current itself: the protocol thread
     * takes the drop (gf_stream_hold_take), once, and a stream started
     * before it does keeps its run currents. */
    reset_state();
    ring_open();
    stream_ended(0);
    gf.hold_poll_at = 0.0;              /* the kernel's idle is read now */
    ship_pass();
    CHECK(run_currents == 1 && hold_currents == 0 && gf.hold_due,
          "the shipper owes the hold drop and writes no current");
    CHECK(gf_stream_hold_take() && !gf_stream_hold_take(), "the protocol thread takes the drop, once");
    gf.hold_due = true;                 /* owed again, and a start comes first */
    gf_stream_wakeup();
    CHECK(!gf_stream_hold_take() && run_currents == 2,
          "a stream started before the drop is taken keeps its run currents");
    gf_stream_go_idle(false, 0, 0);
    ring_close();

    /* A handover with the drop owed writes it before the device changes
     * hands, and leaves none for the protocol pass to write into the other
     * holder's session. */
    reset_state();
    ring_open();
    gf.hold_due = true;
    CHECK(gf_stream_suspend() && hold_currents == 1 && !gf_stream_hold_take(),
          "a handover writes the owed drop itself and leaves none owed");
    gf.suspended = false;
    ring_close();

    /* A run current that does not take is no run: the stream faults at
     * its start, one alarm, and nothing reaches the kernel. */
    reset_state();
    ring_open();
    run_current_fails = true;
    gf_stream_wakeup();
    ship_pass();
    CHECK(gf.failed && !gf.streaming && atomic_exchange(&fault_flag, false) && ring_total() == 0,
          "a run current that does not take faults the stream before a byte ships");
    gf_stream_wakeup();
    CHECK(!atomic_load(&fault_flag), "and raises one alarm for it");
    ring_close();

    /* The controller's start: a latch lock that does not take refuses it
     * before the stepper drivers are configured; a configuration that does
     * not take refuses it, and so does a machine tick that does not take;
     * all three taking start it. */
    reset_state();
    latch_fail_left = 1 + LATCH_RETRIES + 1;
    {
        const char *why = start_config();
        CHECK(why != NULL && strstr(why, "latch") && config_writes == 0,
              "a latch lock that does not take refuses the controller's start, the drivers left unconfigured");
    }
    reset_state();
    config_fails = true;
    CHECK(start_config() != NULL, "a driver configuration that does not take refuses the controller's start");
    reset_state();
    tick_fails = true;
    CHECK(start_config() != NULL, "a machine tick that does not take refuses it");
    reset_state();
    CHECK(start_config() == NULL && config_writes == 1, "all three taking start it");

    /* It refuses the resume after a homing session too. */
    reset_state();
    ring_open();
    gf.suspended = true;
    config_fails = true;
    CHECK(!gf_stream_resume(), "a driver configuration that does not take refuses the resume");
    /* The session's alarm cleared ($X), the refusal stands: the next start
     * writes the configuration again and does not stream while it does not
     * take, and the reset that acknowledges that start's fault leaves the
     * refusal standing; once a write takes, the start streams. */
    gf_stream_wakeup();
    CHECK(gf.failed && !gf.streaming && atomic_exchange(&fault_flag, false),
          "after the refused resume a start does not stream");
    gf_stream_reset();
    gf_stream_wakeup();
    CHECK(gf.failed && !gf.streaming && atomic_exchange(&fault_flag, false),
          "after the fault's reset, a start whose configuration still does not take does not stream");
    gf_stream_reset();
    config_fails = false;
    gf_stream_wakeup();
    CHECK(gf.streaming && !gf.failed && !gf.config_pending,
          "the configuration written again and taken: the start streams");
    gf_stream_go_idle(false, 0, 0);
    config_fails = false;
    gf.suspended = true;
    CHECK(gf_stream_resume(), "one that takes resumes");
    ring_close();

    /* The null sink has no kernel queue to drain and owes the hold drop all
     * the same, so its attribute log shows the run currents and then the
     * hold currents after a move. */
    reset_state();
    ring_open();
    gf.active = false;
    stream_ended(0);
    gf.hold_poll_at = 0.0;
    ship_pass();
    CHECK(run_currents == 1 && gf_stream_hold_take(), "the null sink owes the hold drop after a move");
    gf.active = true;
    ring_close();

    printf("late events outside the window:\n");

    /* Unarmed, a clamp faults nothing and the move goes on. The steps it
     * pushes forward are counted per axis for the stream, from the step
     * bits alone: X and Y's reference is owed its drop once X or Y has
     * owed a full step (the mode the controller's start read, which the
     * stub plays), the lens's once it has owed two half-steps, each once.
     * gf.shipped forged ahead of the producer's clock makes every pulse a
     * late one. A laser event's clamp counts no step, and the X and Y
     * pulses carry every axis's direction bit: a direction is no step. */
    static const uint32_t modes[] = { 8, 16, 32 };
    for(int i = 0; i < 3; i++) {
        char what[112];
        reset_state();
        ring_open();
        stub_mode = modes[i];
        gf_stream_wakeup();
        gf.shipped = gf.base + gf.vticks / VTICKS_PER_BYTE + 1000;
        pthread_mutex_lock(&gf.lock);
        for(int n = 0; n < 50; n++)
            clamp_locked();             /* a laser event's clamp */
        pthread_mutex_unlock(&gf.lock);
        for(uint32_t n = 0; n + 1 < modes[i]; n++)
            gf_stream_pulse(0x01, 0x07);    /* X: one microstep short of a full step */
        for(uint32_t n = 0; n + 1 < modes[i]; n++)
            gf_stream_pulse(0x02, 0x07);    /* and Y the same, apart */
        snprintf(what, sizeof(what), "x%u: %u late X and %u late Y steps owe no drop, and fault nothing",
                 modes[i], modes[i] - 1, modes[i] - 1);
        CHECK(!gf_stream_clamp_take() && !gf.failed && !atomic_load(&fault_flag), what);
        gf_stream_pulse(0x01, 0x07);
        snprintf(what, sizeof(what), "x%u: %u late X steps owe X and Y's drop alone, once",
                 modes[i], modes[i]);
        CHECK(gf_stream_clamp_take() == GF_LATE_XY && !gf_stream_clamp_take(), what);
        gf_stream_go_idle(false, 0, 0);
        ring_close();
    }
    reset_state();
    ring_open();
    gf_stream_wakeup();
    gf.shipped = gf.base + gf.vticks / VTICKS_PER_BYTE + 1000;
    for(int n = 0; n < 31; n++)
        gf_stream_pulse(0x02, 0);
    CHECK(!gf_stream_clamp_take(), "31 late Y steps at x32 owe no drop");
    gf_stream_pulse(0x02, 0);
    CHECK(gf_stream_clamp_take() == GF_LATE_XY, "the 32nd owes X and Y's");
    gf_stream_go_idle(false, 0, 0);
    gf_stream_wakeup();                 /* a new stream starts its count over */
    gf.shipped = gf.base + gf.vticks / VTICKS_PER_BYTE + 1000;
    gf_stream_pulse(0x04, 0);
    CHECK(!gf_stream_clamp_take(), "a new stream: one late lens half-step owes no drop");
    gf_stream_pulse(0x04, 0);
    CHECK(gf_stream_clamp_take() == GF_LATE_Z, "two late lens half-steps owe the lens's alone");
    for(int n = 0; n < 32; n++)
        gf_stream_pulse(0x01, 0);
    CHECK(gf_stream_clamp_take() == GF_LATE_XY, "in the same stream, a full step of X then owes X and Y's");
    gf_stream_go_idle(false, 0, 0);
    ring_close();

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
