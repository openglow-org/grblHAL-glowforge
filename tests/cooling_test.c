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

  - inside an open armed window, a verdict named FIRE, CRASH, AIRFLOW or
    CRITICAL is the fail tier (the laser module is told to end the job),
    and every other fire_ok=false verdict, and a stale one, is the pause
    tier (the job is held, said once, and the latch is never written);
  - a clean verdict with resume_ok resumes the hold the client took;
  - outside the window a blocking verdict is neither tier;
  - every change of the armed bit is reported at once, whatever the
    sender's M8/M9 state;
  - the per-tick gate's input is the verdict's own fire_ok, without the
    engine's acknowledgment of the window, which gfcool_fire_ok() still
    waits on.
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

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_cooling.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
system_t sys;
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

/* Publish a verdict the way the engine does: the whole document, then a
   rename into place. age_s backdates ts_mono (a stale publisher). */
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
               "\"resume_ok\":%s,\"armed\":%s,\"reason\":\"%s\",\"down_c\":20.0,\"up_c\":20.0}\n",
            now - age_s, fire_ok ? "true" : "false", name, hold ? "true" : "false",
            resume_ok ? "true" : "false", armed ? "true" : "false",
            fire_ok ? "" : "test: fire blocked");
    fclose(f);
    rename(tmp, verdict_path);
    next_verdict_ms = mono_ms();        /* the next poll reads it */
    /* A stale file does not touch the cache; the cache expires on its
       own clock. A backdated publish models the engine gone for that
       long, so the cache's deadline is moved back the same way. */
    if(age_s > 0.0)
        atomic_store(&v_fresh_until_ms, mono_ms() - 1);
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
    hold_ours = false;
    rehold_said = stale_warned = fallback_done = pause_said = false;
    atomic_store(&decel_lit, false);
    laser_on_window = false;
    coolant_reported.flood = Off;
    atomic_store(&v_fresh_until_ms, 0);
    v_reason_shown[0] = '\0';
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
    gfcool_init();

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
    cur_state = STATE_CYCLE;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(fail_calls == 0, "OVERTEMP is not the fail tier");
    CHECK(enqueued(CMD_FEED_HOLD) && hold_ours, "the pause tier holds the job");
    CHECK(gfcool_decel_lit(), "the first hold of the episode decelerates lit");
    CHECK(strstr(all_messages, "fire masked, job held") != NULL, "the pause tier says what it did");
    CHECK(!wrote("cnc/laser_latch=1"), "the pause tier writes no latch");
    cur_state = STATE_HOLD;
    sys.holding_state = Hold_Pending;   /* still decelerating */
    gfcool_poll();
    CHECK(gfcool_decel_lit(), "the gate stays open while the head decelerates");
    sys.holding_state = Hold_Complete;
    publish(false, "OVERTEMP", true, false, true, 0.0);
    gfcool_poll();
    CHECK(!gfcool_decel_lit(), "the gate closes once the head has stopped");
    CHECK(strstr(strstr(all_messages, "fire masked") + 1, "fire masked") == NULL,
          "a standing pause is said once");
    /* A ~ under the standing verdict: held again, dark this time. */
    cur_state = STATE_CYCLE;
    ncmds = 0;
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && !gfcool_decel_lit(),
          "a resume under the standing verdict is held again with the gate closed");
    CHECK(strstr(all_messages, "held again") != NULL, "and says so");
    cur_state = STATE_HOLD;
    /* The clean verdict resumes the hold the client took. */
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START) && !hold_ours, "the clean verdict resumes the hold the client took");
    CHECK(strstr(all_messages, "resuming") != NULL, "the resume is reported");
    CHECK(fail_calls == 0 && !wrote("cnc/laser_latch=1") && !wrote("cnc/laser_latch=0"),
          "the whole pause episode wrote no latch");
    CHECK(!gfcool_decel_lit(), "the clean verdict leaves no lit-deceleration grace behind");

    /* A second episode says so again. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(false, "COLD", true, false, true, 0.0);
    gfcool_poll();
    CHECK(hold_ours && strstr(all_messages, "fire masked") != NULL, "COLD is the pause tier");

    /* The fail tier, by name. */
    static const char *fail_names[] = { "FIRE", "CRASH", "AIRFLOW", "CRITICAL" };
    for(int i = 0; i < 4; i++) {
        reset_state();
        armed_val = true;
        laser_on_window = true;
        cur_state = STATE_CYCLE;
        publish(false, fail_names[i], true, false, true, 0.0);
        gfcool_poll();
        char msg[64];
        snprintf(msg, sizeof(msg), "%s inside the window is the fail tier", fail_names[i]);
        CHECK(fail_calls == 1 && !strcmp(fail_name, fail_names[i]) &&
              strstr(all_messages, "fire masked") == NULL, msg);
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
        CHECK(fail_calls == 0 && hold_ours && strstr(all_messages, "fire masked") != NULL, msg);
    }

    /* A stale verdict inside the window is the pause tier, with the
       fallback airflow once and the heater off. */
    reset_state();
    armed_val = true;
    laser_on_window = true;
    cur_state = STATE_CYCLE;
    publish(true, "OK", false, true, true, 3.0);
    gfcool_poll();
    CHECK(fail_calls == 0 && strstr(all_messages, "fire masked") != NULL,
          "a stale verdict inside the window is the pause tier");
    CHECK(enqueued(CMD_FEED_HOLD), "a stale verdict holds the job");
    CHECK(wrote("thermal/heater_pwm=0") && wrote("thermal/exhaust_pwm=" FB_EXHAUST),
          "a stale verdict writes the fallback airflow and the heater off");
    CHECK(!wrote("cnc/laser_latch=1"), "a stale verdict writes no latch");
    /* The engine returns clean: the job resumes. */
    cur_state = STATE_HOLD;
    ncmds = 0;
    publish(true, "OK", false, true, true, 0.0);
    gfcool_poll();
    CHECK(enqueued(CMD_CYCLE_START), "the restored engine resumes");
    CHECK(strstr(all_messages, "restored") != NULL, "the restoration is reported");

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
    atomic_store(&v_fresh_until_ms, mono_ms() - 1);
    gfcool_poll();
    CHECK(enqueued(CMD_FEED_HOLD) && hold_ours,
          "a cache that expires between reads holds the job before the next read");
    CHECK(strstr(all_messages, "fire masked") != NULL, "and says so");

    /* Outside the window a blocking verdict is neither tier. */
    reset_state();
    armed_val = false;
    cur_state = STATE_IDLE;
    publish(false, "AIRFLOW", true, false, false, 0.0);
    gfcool_poll();
    CHECK(fail_calls == 0 && strstr(all_messages, "fire masked") == NULL,
          "a blocking verdict outside the window is neither tier");

    unlink(verdict_path);
    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the verdict's fail tier ends the job, its pause tier holds without a "
                      "latch write and resumes on the clean verdict, and every armed change is reported\n",
           failures);
    return failures ? 1 : 0;
}
