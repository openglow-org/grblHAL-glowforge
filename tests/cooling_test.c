/*
  cooling_test.c - host unit test for the cooling client's verdict tiers

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  The cooling client reads the engine's verdict file and enforces it in
  process. This test includes the client source, publishes verdicts into
  a temporary file the way the engine does, and drives gfcool_poll():

  - inside an open armed window, a verdict named FIRE, CRASH, EMISSION, AIRFLOW or
    CRITICAL is the fail tier (the laser module is told to end the job),
    and every other fire_ok=false verdict, and a stale one, is the pause
    tier (the job is held, said once, and the latch is never written);
  - a verdict stamped ahead of the reader's clock is no verdict, as a
    stale one is: fire stays blocked and the job is held;
  - the freshness deadline never wraps: one 2^31 ms in the past is stale,
    and before the first verdict nothing is fresh at any uptime, one in
    [2^31, 2^32) ms included (the client's clock is moved on there);
  - a clean verdict with resume_ok resumes the hold the client took, once
    the hold has completed;
  - the pause tier's lit deceleration is bounded: a hold the core did not
    take ends it and stops the job with a reset, and a hold that has not
    completed inside the longest stop the settings allow ends it too (at
    the defaults, at the floors, and at the 60 s cap), and it runs only
    under a fresh verdict: a stale one's hold is dark, and a verdict gone
    stale ends a lit deceleration for good;
  - outside the window a blocking verdict is neither tier;
  - every change of the armed bit is reported at once, whatever the
    sender's M8/M9 state;
  - the per-tick gate's input is the verdict's own fire_ok, without the
    engine's acknowledgment of the window, which gfcool_fire_ok() still
    waits on;
  - GF_VERDICT_FILE moves the verdict file only in a null-sink run: with
    GFSINK set the client reads the engine's file;
  - the acknowledgment is the window's own: a change of the window clears
    it at once, and a verdict read gives it again only when it began after
    the engine answered that window's report (its 200; a refused report
    answers only a harness that publishes its own verdicts);
  - no report answered for 3 s reads as a stale reporter.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

static int   fail_calls;
static char  fail_name[16];
static bool  armed_val;                 /* gflaser_armed() */
static uint8_t cmds[32];
static int   ncmds;
static char  last_message[160];
static char  all_messages[4096];
static char  wr_log[32][48];
static int   wr_n;

void gflaser_verdict_fail (const char *name)
{ fail_calls++; snprintf(fail_name, sizeof(fail_name), "%s", name); }
bool gflaser_armed (void) { return armed_val; }
bool gflaser_resume_gate (void) { return false; }
bool gflaser_density (void) { return true; }
bool protocol_enqueue_realtime_command (uint8_t c) { if(ncmds < 32) cmds[ncmds] = c; ncmds++; return true; }
int  gfio_wr_attr (const char *attr, const char *val)
{
    if(wr_n < 32)
        snprintf(wr_log[wr_n], sizeof(wr_log[0]), "%s=%s", attr, val);
    wr_n++;
    return 0;
}
void fflog (int prio, const char *fmt, ...) { (void)prio; (void)fmt; }

/* The client's clock, and the test's: CLOCK_MONOTONIC moved on by
   clock_offset_ms, so a case can stand at an uptime the host does not
   have. Zero for every case but the one that sets it. A case that sets
   clock_frozen stands the monotonic clock at clock_base: only the case's
   own steps move it, however long the host takes between them. */
#include <time.h>
static long long clock_offset_ms;
static int clock_frozen;
static struct timespec clock_base;
static int offset_clock_gettime (clockid_t id, struct timespec *ts)
{
    int r = clock_gettime(id, ts);
    if(clock_frozen && id == CLOCK_MONOTONIC)
        *ts = clock_base;
    long long ns = (long long)ts->tv_sec * 1000000000LL + ts->tv_nsec + clock_offset_ms * 1000000LL;
    ts->tv_sec = (time_t)(ns / 1000000000LL);
    ts->tv_nsec = (long)(ns % 1000000000LL);
    return r;
}
#define clock_gettime offset_clock_gettime

/* A read of the verdict file that takes its time: when a case sets
   slow_read, the next fopen moves the clock on 20 ms and records what the
   stepper producer would see from gfcool_decel_lit() while the read is
   under way. */
#include <stdio.h>
static int slow_read;
static int seen_during_read = -1;
static FILE *slow_fopen (const char *path, const char *mode);
#define fopen slow_fopen

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_cooling.c"

#undef fopen
static FILE *slow_fopen (const char *path, const char *mode)
{
    if(slow_read) {
        slow_read = 0;
        clock_offset_ms += 20;
        seen_during_read = gfcool_decel_lit();
    }
    return fopen(path, mode);
}

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
system_t sys;
settings_t settings;
static sys_state_t cur_state = STATE_IDLE;
sys_state_t state_get (void) { return cur_state; }
void report_message (const char *msg, message_type_t type)
{
    (void)type;
    snprintf(last_message, sizeof(last_message), "%s", msg ? msg : "");
    size_t n = strlen(all_messages);
    snprintf(all_messages + n, sizeof(all_messages) - n, "%s\n", msg ? msg : "");
}

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static char verdict_path[] = "/tmp/cooling_test.XXXXXX";
/* Fields a case adds to the published document (the ack_wait field). */
static const char *publish_extra = "";

/* Publish a verdict the way the engine does: the whole document, then a
   rename into place. age_s backdates ts_mono (a stale publisher); a
   negative age_s stamps it ahead of the reader's clock. */
static void publish (bool fire_ok, const char *name, bool hold, bool resume_ok,
                     bool armed, double age_s)
{
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "%s.tmp", verdict_path);
    FILE *f = fopen(tmp, "w");
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    /* One millisecond back: the engine prints ts_mono to the
       millisecond, and a reader that follows within that millisecond
       must not see a timestamp from its future. */
    double now = (double)ts.tv_sec + ts.tv_nsec / 1e9 - 0.001;
    fprintf(f, "{\"seq\":1,\"ts_mono\":%.3f,\"fire_ok\":%s,\"verdict\":\"%s\",\"hold\":%s,"
               "\"resume_ok\":%s,\"armed\":%s,\"reason\":\"%s\"%s,\"down_c\":20.0,\"up_c\":20.0}\n",
            now - age_s, fire_ok ? "true" : "false", name, hold ? "true" : "false",
            resume_ok ? "true" : "false", armed ? "true" : "false",
            fire_ok ? "" : "test: fire blocked", publish_extra);
    fclose(f);
    rename(tmp, verdict_path);
    next_verdict_ms = mono_ms();        /* the next poll reads it */
    /* A stale file does not touch the cache; the cache expires on its
       own clock. A backdated publish models the engine gone for that
       long, so the cache's deadline is moved back the same way. */
    if(age_s > 0.0)
        atomic_store(&v_fresh_until_ms, mono_ms64() - 1);
}

/* A cut in progress: the job in its cycle, seen once under a clean
   verdict, as every running cut has been before a verdict turns on it. */
static void cut_in_progress (void)
{
    cur_state = STATE_CYCLE;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
}

static bool enqueued (uint8_t c)
{
    for(int i = 0; i < ncmds && i < 32; i++)
        if(cmds[i] == c)
            return true;
    return false;
}

static bool wrote (const char *what)
{
    for(int i = 0; i < wr_n && i < 32; i++)
        if(!strcmp(wr_log[i], what))
            return true;
    return false;
}

static void reset_state (void)
{
    fail_calls = 0;
    fail_name[0] = '\0';
    ncmds = 0;
    wr_n = 0;
    last_message[0] = all_messages[0] = '\0';
    cur_state = STATE_IDLE;
    memset(&sys, 0, sizeof(sys));
    armed_val = false;
    hold_ours = hold_asked = hold_refused = cycle_ok = op_held = false;
    hold_misses = 0;
    rehold_said = stale_warned = fallback_done = pause_said = false;
    atomic_store(&decel_lit, false);
    laser_on_window = false;
    coolant_reported.flood = Off;
    atomic_store(&v_fresh_until_ms, 0);
    v_reason_shown[0] = '\0';
    publish_extra = "";
}

int main (void)
{
    int fd = mkstemp(verdict_path);
    if(fd < 0) {
        printf("FAIL: cannot create the verdict file\n");
        return 1;
    }
    close(fd);
    setenv("GF_VERDICT_FILE", verdict_path, 1);
    setenv("FORGECTRL_PORT", "1", 1);   /* the reporter's POSTs fail fast */

    printf("the verdict file that GF_VERDICT_FILE names, honored only in a null-sink run:\n");

    /* On a machine the supervisor sets GFSINK, and the client reads the
       engine's file whatever GF_VERDICT_FILE names. Each init starts a
       reporter; both send to a port that refuses at once. */
    setenv("GFSINK", "/dev/glowforge", 1);
    gfcool_init();
    CHECK(!strcmp(verdict_file, VERDICT_FILE), "with GFSINK set, the client reads the engine's verdict file");
    CHECK(!harness_engine && !report_answered(true, NULL, 0),
          "with GFSINK set, a refused report answers nothing");
    unsetenv("GFSINK");
    gfcool_init();                      /* a null-sink run: the cases below publish to verdict_path */

    printf("the armed bit is reported on every change:\n");

    reset_state();
    coolant_reported.flood = On;        /* the sender's M8 is already on */
    unsigned seq0 = rep_seq;
    gfcool_laser_armed(true);
    CHECK(rep_seq == seq0 + 1, "arming under M8 reports at once");
    CHECK(rep_armed, "the report carries armed");
    gfcool_laser_armed(true);
    CHECK(rep_seq == seq0 + 1, "an unchanged armed bit reports nothing");
    gfcool_laser_armed(false);
    CHECK(rep_seq == seq0 + 2 && !rep_armed, "disarming under M8 reports at once");

    printf("the per-tick gate's input against the arm-time gate:\n");

    reset_state();
    laser_on_window = true;
    publish(true, "OK", false, true, false, 0.0);
    verdict_read();
    CHECK(gfcool_verdict_fire_ok(), "the verdict's own fire_ok is the gate's input");
    CHECK(!gfcool_fire_ok(), "the arm-time gate still waits on the engine's acknowledgment");
    publish(true, "OK", false, true, true, 0.0);
    verdict_read();
    CHECK(gfcool_fire_ok(), "the acknowledged verdict passes the arm-time gate");
    publish(true, "OK", false, true, true, 3.0);
    verdict_read();
    CHECK(!gfcool_verdict_fire_ok(), "a stale verdict is no verdict");

    printf("the tiers inside an open window:\n");

    /* The pause tier: OVERTEMP holds, says so once, writes no latch. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(fail_calls == 0, "OVERTEMP is not the fail tier");
    CHECK(enqueued(CMD_FEED_HOLD) && hold_ours, "the pause tier holds the job");
    CHECK(gfcool_decel_lit(), "the first hold of the episode decelerates lit");
    CHECK(pause_said, "the pause tier says what it did");
    CHECK(!wrote("cnc/laser_latch=1"), "the pause tier writes no latch");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;   /* still decelerating */
    gfcool_poll();
    CHECK(gfcool_decel_lit(), "the gate stays open while the head decelerates");
    sys.holding_state = Hold_Complete;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(!gfcool_decel_lit(), "the gate closes once the head has stopped");
    CHECK(pause_said, "a standing pause is said once, and not again");
    /* A ~ under the standing verdict: held again, dark this time. */
    cur_state = STATE_CYCLE;
    ncmds = 0;
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && !gfcool_decel_lit(),
          "a resume under the standing verdict is held again with the gate closed");
    CHECK(hold_ours, "the hold taken again is the client's");
    cur_state = STATE_HOLD;
    /* The clean verdict resumes the hold the client took. */
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START) && !hold_ours, "the clean verdict resumes the hold the client took");
    CHECK(!pause_said, "the clean verdict ends the episode");
    CHECK(fail_calls == 0 && !wrote("cnc/laser_latch=1") && !wrote("cnc/laser_latch=0"),
          "the whole pause episode wrote no latch");
    CHECK(!gfcool_decel_lit(), "the clean verdict leaves no lit-deceleration grace behind");

    /* A cycle that starts under a verdict that already blocks fire (the
     * job sat at Idle, in a dwell or between two lines, when the verdict
     * turned): it is held, and no lit deceleration comes with the hold.
     * There is no cut in progress to finish, and its first move is dark. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_IDLE;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    cur_state = STATE_CYCLE;            /* the job's next cut starts */
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && hold_ours, "a cycle that starts under the verdict is held");
    CHECK(!gfcool_decel_lit(), "and its hold is dark: the gate stays closed");

    /* A hold the core does not take (the sender's M53 P0 disables feed
     * hold): two requests in a row have been executed and the job is
     * still in its cycle. The lit deceleration ends there and the job is
     * stopped. A request the core has not executed yet is waited for, and
     * one executed request is not yet a refusal: the poll can run between
     * the core's taking a request and its acting on it. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && gfcool_decel_lit(), "the pause tier asks for its hold, lit");
    sys.rt_exec_state = EXEC_FEED_HOLD; /* the request is still pending */
    ncmds = 0;
    gfcool_poll();
    CHECK(gfcool_decel_lit() && !enqueued(CMD_RESET), "a hold the core has not executed yet is waited for");
    sys.rt_exec_state = 0;              /* taken by the core, and the job is still in its cycle */
    ncmds = 0;
    gfcool_poll();
    CHECK(gfcool_decel_lit() && !enqueued(CMD_RESET) && enqueued(CMD_FEED_HOLD),
          "one request taken with the job still in its cycle is asked for again, not yet a refusal");
    ncmds = 0;
    gfcool_poll();                      /* the second one taken too, and still no hold */
    CHECK(!gfcool_decel_lit(), "a hold the core did not take ends the lit deceleration");
    CHECK(enqueued(CMD_RESET) && !enqueued(CMD_FEED_HOLD), "and the job is stopped with a reset");
    ncmds = 0;
    gfcool_poll();
    CHECK(ncmds == 0, "the stop is asked for once");

    /* The first request acted on between two polls: the job is in its
     * hold, and nothing is stopped. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    gfcool_poll();                      /* taken, not yet acted on: asked for again */
    cur_state = STATE_HOLD;             /* the core acts on the first */
    sys.holding_state = Hold_Pending;
    ncmds = 0;
    gfcool_poll();
    cur_state = STATE_CYCLE;            /* a resume under the standing verdict */
    gfcool_poll();
    gfcool_poll();
    CHECK(!enqueued(CMD_RESET), "a hold the core took between two polls is no refusal, and the count starts over at the next cycle");

    /* The lit deceleration's time bound: the longest stop the settings
     * allow, the largest of the axes' rate over acceleration, plus 0.5 s.
     * A hold that has not completed by then no longer holds the gate
     * open. At the board's defaults Y's stop is the longest (200 mm/s over
     * 590 mm/s^2, 339 ms; X 286 ms, Z 100 ms): 839 ms. At the floors, half
     * the defaults, Y again (200 over 295, 678 ms): 1178 ms. And a
     * nonsense acceleration, which the settings dispatch never leaves in
     * force, is held to the 60 s cap: 60.5 s. */
    static const struct {
        const char *what;
        float ax, ay, az;               /* $120-$122, mm/s^2 */
        long long bound_ms;
    } bounds[] = {
        { "the defaults", 700.0f, 590.0f, 50.0f, 839 },
        { "the floors", 350.0f, 295.0f, 25.0f, 1178 },
        { "a nonsense X acceleration", 1e-6f, 590.0f, 50.0f, 60500 },
    };
    for(int i = 0; i < 3; i++) {
        reset_state();
        clock_gettime(CLOCK_MONOTONIC, &clock_base);
        clock_frozen = 1;               /* the bound is judged on the case's clock alone */
        memset(&settings, 0, sizeof(settings));
        settings.axis[X_AXIS].max_rate = 12000.0f;
        settings.axis[Y_AXIS].max_rate = 12000.0f;
        settings.axis[Z_AXIS].max_rate = 300.0f;
        settings.axis[X_AXIS].acceleration = bounds[i].ax * 3600.0f;
        settings.axis[Y_AXIS].acceleration = bounds[i].ay * 3600.0f;
        settings.axis[Z_AXIS].acceleration = bounds[i].az * 3600.0f;
        armed_val = true;
        laser_on_window = true;
        cut_in_progress();
        publish(false, "OVERTEMP", true, false, true, 0.0);
        gfcool_poll();
        cur_state = STATE_HOLD;
        sys.holding_state = Hold_Pending;   /* decelerating, and it stays that way */
        /* The engine publishes every second and the client polls far more
           often, so the verdict stays fresh all the way to the bound. */
        for(long long left = bounds[i].bound_ms - 50; left > 0; left -= 1000) {
            clock_offset_ms += left < 1000 ? left : 1000;
            publish(false, "OVERTEMP", true, false, true, 0.0);
            gfcool_poll();
        }
        char msg[128];
        snprintf(msg, sizeof(msg), "%s: the gate stays open 50 ms inside the %lld ms bound",
                 bounds[i].what, bounds[i].bound_ms);
        CHECK(gfcool_decel_lit(), msg);
        clock_offset_ms += 100;
        publish(false, "OVERTEMP", true, false, true, 0.0);
        gfcool_poll();
        snprintf(msg, sizeof(msg), "%s: 50 ms past it, a hold that has not completed no longer holds it open",
                 bounds[i].what);
        CHECK(!gfcool_decel_lit(), msg);
        clock_frozen = 0;
    }

    /* A verdict that clears while the hold still decelerates: the core
     * drops a cycle start until the hold completes, so the client keeps
     * the hold and resumes it once the head has stopped. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(!enqueued(CMD_CYCLE_START) && hold_ours && !strstr(all_messages, "resuming"),
          "a verdict that clears while the hold decelerates resumes nothing yet, the hold kept");
    sys.holding_state = Hold_Complete;
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START) && !hold_ours && strstr(all_messages, "resuming"),
          "once the hold completes, the client resumes it");
    /* The operator's own feed hold inside the client's hold: the hold is
     * the operator's, and the verdict that clears resumes nothing. */
    hold_ours = true;
    ncmds = 0;
    all_messages[0] = '\0';
    gfcool_operator_hold();
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(!enqueued(CMD_CYCLE_START) && !hold_ours && !strstr(all_messages, "resuming") &&
          strstr(all_messages, "resume with ~"),
          "after the operator's own feed hold the clean verdict resumes nothing");
    /* The same while the client's hold still decelerates, and in the pass
     * where the client asked for its hold and the core has not taken it
     * yet (the job still in its cycle). */
    for (int when = 0; when < 2; when++) {
        cur_state = when ? STATE_CYCLE : STATE_HOLD;
        sys.holding_state = when ? Hold_NotHolding : Hold_Pending;
        hold_ours = true;
        sys.rt_exec_state = when ? EXEC_FEED_HOLD : 0;     /* the client's request, not yet taken */
        gfcool_operator_hold();
        sys.rt_exec_state = 0;
        cur_state = STATE_HOLD;
        sys.holding_state = Hold_Complete;
        ncmds = 0;
        publish(true, "OK", false, true, true, 0.0);
        gfcool_poll();
        CHECK(!enqueued(CMD_CYCLE_START) && !hold_ours,
              when ? "a feed hold before the core took the client's hold is the operator's: no resume"
                   : "a feed hold during the client's lit deceleration is the operator's: no resume");
    }

    /* The operator's ! in the client's pass, then a poll that runs again
     * before the core acts on the hold (a status report written to a full
     * transmit ring runs the hook between the core's take and its act): the
     * hold stays the operator's, and the clear resumes nothing. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();                      /* the client's pass: hold_take, the claim */
    gfcool_operator_hold();             /* the operator's ! in the same pass */
    sys.rt_exec_state = 0;              /* the core took the request, the job still in its cycle */
    gfcool_poll();                      /* the pass run inside the core's status write */
    int reclaimed = hold_ours;
    cur_state = STATE_HOLD;             /* the core acts on the hold */
    sys.holding_state = Hold_Complete;
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(!reclaimed && !enqueued(CMD_CYCLE_START),
          "a poll before the core acts on the hold takes no claim back from the operator's !: no resume");
    /* The operator's hold ends with that hold: the cycle the operator
     * resumes is a new one, and a verdict's hold in it is the client's. */
    cur_state = STATE_CYCLE;
    sys.holding_state = Hold_NotHolding;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(hold_ours, "in the cycle the operator resumed, a verdict's hold is the client's again");
    /* The operator's ! first (the job in its cycle, no claim yet), then,
     * before the core has acted on it, a poll that reads a pause-tier
     * verdict (a hook pass run by a blocked write): no claim. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    gfcool_operator_hold();             /* the operator's !, the core not yet in its hold */
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();                      /* still Cycle: hold_take claims it */
    int claimed2 = hold_ours;
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Complete;
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(!claimed2 && !enqueued(CMD_CYCLE_START),
          "an operator's ! the core has not acted on, then the verdict: the client claims nothing, no resume");

    /* The same after a cut run clean (a lit deceleration would be due):
     * the client asks for the hold all the same, claims none, and starts
     * no lit deceleration of its own. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();                      /* a cut in progress under a clean verdict */
    gfcool_operator_hold();             /* the operator's !, the core not yet in its hold */
    ncmds = 0;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && !hold_ours && !gfcool_decel_lit(),
          "after the operator's !, the client asks for the hold, claims none, and lights no deceleration");

    /* Under the operator's hold too, a hold the core does not take ends
     * at the second miss with the reset. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    gfcool_operator_hold();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    ncmds = 0;
    gfcool_poll();                      /* asks */
    sys.rt_exec_state = 0;
    gfcool_poll();                      /* the core executed it, the job still in its cycle: a miss */
    sys.rt_exec_state = 0;
    gfcool_poll();                      /* the second: the reset */
    CHECK(enqueued(CMD_RESET), "a hold the core does not take ends with the reset under the operator's hold too");

    /* A fail-tier verdict (EMISSION) inside a cooling hold the operator
     * made theirs ends the job, as in a cycle, and nothing resumes it. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Complete;
    gfcool_operator_hold();
    publish(false, "EMISSION", true, false, true, 0.0);
    gfcool_poll();
    CHECK(fail_calls == 1 && !strcmp(fail_name, "EMISSION"),
          "EMISSION inside the operator's cooling hold is the fail tier");
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(!enqueued(CMD_CYCLE_START), "and nothing resumes it after");

    /* A fault that waits for its Acknowledge: the cause gone, the verdict
     * ACK (fire blocked, no hold, resume_ok): the held job stays held, and
     * the Acknowledge (a clean verdict) resumes it. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(false, "SENSOR", true, false, true, 0.0);
    gfcool_poll();
    CHECK(hold_ours, "SENSOR holds the job");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Complete;
    laser_on_window = false;            /* the window closed during the hold */
    armed_val = false;
    ncmds = 0;
    publish(false, "ACK", false, true, false, 0.0);
    gfcool_poll();
    gfcool_poll();
    CHECK(!enqueued(CMD_CYCLE_START) && hold_ours, "under ACK the held job is not resumed");
    publish(true, "OK", false, true, false, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START) && !hold_ours, "the Acknowledge resumes it");

    /* The acknowledgment is its own field: ack_wait says a fault waits
     * for its Acknowledge under any verdict name, the name ACK says it
     * without the field, a document without the field says nothing, and
     * a stale verdict says nothing at all. */
    reset_state();
    publish_extra = ",\"ack_wait\":true";
    publish(false, "OVERTEMP", true, false, false, 0.0);
    verdict_read();
    CHECK(gfcool_ack_waits(), "ack_wait true under another verdict's name: a fault waits for its Acknowledge");
    publish_extra = "";
    publish(false, "OVERTEMP", true, false, false, 0.0);
    verdict_read();
    CHECK(!gfcool_ack_waits(), "no ack_wait field reads as false");
    publish_extra = ",\"ack_wait\":false";
    publish(false, "ACK", false, true, false, 0.0);
    verdict_read();
    CHECK(gfcool_ack_waits(), "the name ACK still says it");
    publish_extra = ",\"ack_wait\":true";
    publish(false, "OVERTEMP", true, false, false, 0.0);
    verdict_read();
    atomic_store(&v_fresh_until_ms, mono_ms64() - 1);
    CHECK(!gfcool_ack_waits(), "a stale verdict's ack_wait says nothing");
    /* The control panel's prompt that has the machine's button is its own
     * field too: button_reserved says so under any name, a document
     * without it says nothing, and a stale verdict says nothing at all. */
    reset_state();
    publish_extra = ",\"button_reserved\":true";
    publish(false, "BUTTON", false, true, false, 0.0);
    verdict_read();
    CHECK(gfcool_button_reserved(), "button_reserved true: a prompt on the panel has the button");
    publish_extra = "";
    publish(false, "BUTTON", false, true, false, 0.0);
    verdict_read();
    CHECK(!gfcool_button_reserved(), "no button_reserved field reads as false");
    publish_extra = ",\"button_reserved\":true";
    publish(false, "COLD", true, false, false, 0.0);
    verdict_read();
    atomic_store(&v_fresh_until_ms, mono_ms64() - 1);
    CHECK(!gfcool_button_reserved(), "a stale verdict's button_reserved says nothing");
    publish_extra = "";
    /* And the held job is not resumed under it, whatever the name. */
    reset_state();
    hold_ours = true;
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Complete;
    publish_extra = ",\"ack_wait\":true";
    publish(false, "EXT", false, true, false, 0.0);
    gfcool_poll();
    CHECK(!enqueued(CMD_CYCLE_START) && hold_ours, "under ack_wait the held job is not resumed, whatever the name");
    publish_extra = "";

    /* A second episode says so again. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(false, "COLD", true, false, true, 0.0);
    gfcool_poll();
    CHECK(hold_ours && pause_said, "COLD is the pause tier");

    /* The fail tier, by name. */
    static const char *fail_names[] = { "FIRE", "CRASH", "EMISSION", "AIRFLOW", "CRITICAL" };
    for(int i = 0; i < 5; i++) {
        reset_state();
        armed_val = true;
        laser_on_window = true;
        cur_state = STATE_CYCLE;
        publish(false, fail_names[i], true, false, true, 0.0);
        gfcool_poll();
        char msg[64];
        snprintf(msg, sizeof(msg), "%s inside the window is the fail tier", fail_names[i]);
        CHECK(fail_calls == 1 && !strcmp(fail_name, fail_names[i]) && !pause_said, msg);
    }
    static const char *pause_names[] = { "FAULT", "SENSOR", "WARMUP", "BUMP", "FLAME" };
    for(int i = 0; i < 5; i++) {
        reset_state();
        armed_val = true;
        laser_on_window = true;
        cur_state = STATE_CYCLE;
        publish(false, pause_names[i], true, false, true, 0.0);
        gfcool_poll();
        char msg[64];
        snprintf(msg, sizeof(msg), "%s inside the window is the pause tier", pause_names[i]);
        CHECK(fail_calls == 0 && hold_ours && pause_said, msg);
    }

    /* The pause tier as a held job with its window closed meets it
       (gfcool_hold_stands): a fresh verdict that blocks firing short of
       the fail tier, a fault waiting for its Acknowledge included. The
       fail tier, a clean verdict and a stale one are not it. */
    reset_state();
    publish(false, "OVERTEMP", true, false, false, 0.0);
    verdict_read();
    CHECK(gfcool_hold_stands(), "OVERTEMP over a closed window is a standing hold");
    publish(false, "ACK", false, true, false, 0.0);
    verdict_read();
    CHECK(gfcool_hold_stands(), "so is a fault waiting for its Acknowledge");
    publish(false, "CRITICAL", true, false, false, 0.0);
    verdict_read();
    CHECK(!gfcool_hold_stands(), "the fail tier is not");
    publish(true, "OK", false, true, false, 0.0);
    verdict_read();
    CHECK(!gfcool_hold_stands(), "a verdict that lets the laser fire is not");
    publish(false, "OVERTEMP", true, false, false, 0.0);
    verdict_read();
    atomic_store(&v_fresh_until_ms, mono_ms64() - 1);     /* the same verdict, gone stale */
    CHECK(!gfcool_hold_stands(), "a stale verdict is not");

    /* A stale verdict inside the window is the pause tier, with the
       fallback airflow once and the heater off. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(true, "OK", false, true, true, 3.0);
    gfcool_poll();
    CHECK(fail_calls == 0 && pause_said, "a stale verdict inside the window is the pause tier");
    CHECK(enqueued(CMD_FEED_HOLD), "a stale verdict holds the job");
    CHECK(wrote("thermal/heater_pwm=0") && wrote("thermal/exhaust_pwm=" FB_EXHAUST),
          "a stale verdict writes the fallback airflow and the heater off");
    CHECK(!wrote("cnc/laser_latch=1"), "a stale verdict writes no latch");
    /* The engine returns clean: the job resumes. */
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Complete;
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START) && !hold_ours && !pause_said, "the restored engine resumes");

    /* A stale verdict vouches for nothing: its hold over a cut in
       progress runs dark, and a verdict that comes back fresh but
       blocking while the head still slows down does not relight it. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(true, "OK", false, true, true, 3.0);
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && !gfcool_decel_lit(), "a stale verdict's hold over a cut runs dark");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(!gfcool_decel_lit(), "a fresh pause verdict after it does not relight the deceleration");
    /* A lit deceleration whose verdict expired between two polls, with no
       reader in between (an M3 cut, whose segments send no update), stays
       dark when the next poll is the read that brings back a fresh, still
       blocking verdict: the poll ends it before it reads. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(gfcool_decel_lit(), "a fresh pause verdict over a cut decelerates lit");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;
    atomic_store(&v_fresh_until_ms, mono_ms64() - 1);  /* expired, and nothing reads it before the poll */
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(!gfcool_decel_lit(), "and the read poll that brings a blocking verdict back fresh does not relight it");
    /* The cache expires inside the read poll itself, after its freshness
       check and before the read stores the new deadline: the producer sees
       the deceleration dark during the read, and the read that brings the
       blocking verdict back fresh does not relight it. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cut_in_progress();
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(gfcool_decel_lit(), "a fresh pause verdict over a cut decelerates lit again");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;
    atomic_store(&v_fresh_until_ms, mono_ms64() + 10);     /* fresh for 10 ms more */
    publish(false, "OVERTEMP", true, false, true, 0.0);
    slow_read = 1;
    gfcool_poll();
    CHECK(seen_during_read == 0, "the verdict gone stale ends the lit deceleration at once, mid-read");
    CHECK(!gfcool_decel_lit(), "a deceleration the producer saw dark during the read is not relit by that read");
    clock_offset_ms = 0;

    /* A verdict stamped ahead of the reader's clock is no verdict, as a
       stale one is: fire stays blocked and the job is held. The cache
       has expired before the read, so only an accepted document opens
       the gate. */
    static const double ahead_s[] = { 0.25, 5.0 };
    for(int i = 0; i < 2; i++) {
        reset_state();
        armed_val = true;
        laser_on_window = true;
        cur_state = STATE_CYCLE;
        atomic_store(&v_fresh_until_ms, mono_ms64() - 1);
        publish(true, "OK", false, true, true, -ahead_s[i]);
        gfcool_poll();
        char msg[96];
        snprintf(msg, sizeof(msg), "a stamp %g s ahead of the clock is no verdict: fire blocked, the job held",
                 ahead_s[i]);
        CHECK(!gfcool_verdict_fire_ok() && enqueued(CMD_FEED_HOLD), msg);
    }

    /* The enforcement runs on every poll from the cache, not only when
       the file is read: a cache that expires between two reads holds
       the job at once, in the tick the fire gate closes. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();                      /* reads the file; the next read is 500 ms away */
    CHECK(!enqueued(CMD_FEED_HOLD) && !hold_ours, "a clean verdict holds nothing");
    atomic_store(&v_fresh_until_ms, mono_ms64() - 1);
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && hold_ours,
          "a cache that expires between reads holds the job before the next read");
    CHECK(pause_said, "and says so");

    /* Outside the window a blocking verdict is neither tier. */
    reset_state();
    armed_val = false;
    cur_state = STATE_IDLE;
    publish(false, "AIRFLOW", true, false, false, 0.0);
    gfcool_poll();
    CHECK(fail_calls == 0 && !pause_said && !hold_ours, "a blocking verdict outside the window is neither tier");

    printf("the freshness deadline at any distance and any uptime:\n");

    /* The client's clock stands at an uptime in [2^31, 2^32) ms (mod
       2^32), where a 32-bit deadline of 0 reads ahead of it. */
    struct timespec real;
    clock_gettime(CLOCK_MONOTONIC, &real);
    long long real_ms = (long long)real.tv_sec * 1000 + real.tv_nsec / 1000000 - clock_offset_ms;
    clock_offset_ms = 3000000000LL - (real_ms % 4294967296LL);

    /* A verdict that stopped refreshing 2^31 ms ago is as stale as one
       that stopped a second ago: the deadline never wraps into fresh. */
    reset_state();
    publish(true, "OK", false, true, true, 0.0);
    verdict_read();
    CHECK(gfcool_verdict_fire_ok(), "a fresh verdict permits fire");
    atomic_store(&v_fresh_until_ms, mono_ms64() - (1ull << 31));
    CHECK(!verdict_fresh() && !gfcool_verdict_fire_ok() && !gfcool_run_ack(),
          "a deadline 2^31 ms in the past is stale: fire blocked");

    /* Before any verdict there is no verdict at that uptime: the stale
       branch runs (the fallback for an open window). */
    reset_state();
    unlink(verdict_path);
    laser_on_window = true;
    armed_val = true;
    cur_state = STATE_CYCLE;
    uint64_t uptime = mono_ms64() % 4294967296ULL;
    gfcool_poll();
    CHECK(uptime >= 2147483648ULL && !verdict_fresh() && !gfcool_verdict_fire_ok() && stale_warned &&
          strstr(all_messages, "cooling service lost") != NULL,
          "at an uptime in [2^31, 2^32) ms no verdict is fresh before the first: fire blocked, the fallback runs");

    /* The clock itself is 64 bits wide: a verdict read a second before the
       millisecond count passes 2^32 is stale 4 s after it, never fresh
       again on a count that wrapped. */
    clock_offset_ms = 0;
    clock_gettime(CLOCK_MONOTONIC, &real);
    real_ms = (long long)real.tv_sec * 1000 + real.tv_nsec / 1000000;
    clock_offset_ms = (real_ms / 4294967296LL + 1) * 4294967296LL - 1000 - real_ms;
    reset_state();
    publish(true, "OK", false, true, true, 0.0);
    verdict_read();
    CHECK(gfcool_verdict_fire_ok(), "a verdict read just under 2^32 ms is fresh");
    clock_offset_ms += 5000;
    CHECK(!verdict_fresh() && !gfcool_verdict_fire_ok(), "past 2^32 ms, 5 s on, it is stale");
    clock_offset_ms = 0;

    printf("the acknowledgment is the window's own, and an unanswered reporter is seen:\n");

    /* The engine's answer is its 200; a refused connection answers only a
       harness that stands in for the engine. Off for the cases below, so
       the reporter thread's own sends (to a port that refuses) answer
       nothing while they run. */
    CHECK(report_answered(false, "HTTP/1.1 200 OK\r\n", 17) &&
          !report_answered(false, "HTTP/1.1 403 Forbidden\r\n", 24) &&
          !report_answered(false, "", -1), "only the engine's 200 answers a report");
    CHECK(report_answered(true, NULL, 0) == harness_engine && harness_engine,
          "a refused report answers the harness that publishes its own verdicts");
    harness_engine = false;
    CHECK(!report_answered(true, NULL, 0), "and nothing else");
    {
        struct timespec settle = { 0, 100 * 1000000L };
        nanosleep(&settle, NULL);       /* a send already under way ends */
    }

    /* A window's change clears the acknowledgment read for the last one at
       once, and a read gives it again only once it began after the engine
       answered this window's report. */
    reset_state();
    publish(true, "OK", false, true, true, 0.0);
    atomic_store(&rep_answered_gen, win_gen);
    verdict_read();
    gfcool_laser_armed(true);
    CHECK(!gfcool_run_ack(), "a window that opens has no acknowledgment of the last");
    verdict_read();
    CHECK(!gfcool_run_ack(), "a read begun before its report was answered gives none");
    atomic_store(&rep_answered_gen, win_gen);
    verdict_read();
    CHECK(gfcool_run_ack() && gfcool_fire_ok(), "a read begun after it was answered gives it");
    gfcool_laser_armed(false);
    CHECK(!gfcool_run_ack(), "the close takes it off at once");

    /* No report answered for 3 s: the reporter reads stale. */
    atomic_store(&rep_answered_ms, mono_ms64() - REPORT_STALE_MS - 1);
    CHECK(gfcool_report_stale(), "no report answered for 3 s reads stale");
    atomic_store(&rep_answered_ms, mono_ms64());
    CHECK(!gfcool_report_stale(), "an answered report reads fresh");
    harness_engine = true;

    unlink(verdict_path);
    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the verdict's fail tier ends the job, its pause tier holds without a "
                      "latch write and resumes on the clean verdict, and every armed change is reported\n",
           failures);
    return failures ? 1 : 0;
}
