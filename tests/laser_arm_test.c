/*
  laser_arm_test.c - host unit test for the operator-arm coolant re-check

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  Regression for G-4: gflaser_arm() must re-check gfcool_fire_ok() after
  the operator button wait, immediately before it opens the armed
  window. The button wait can run for minutes, and the cached cooling
  verdict is refreshed under it (gfcool_poll, ~1 Hz); a verdict that
  went bad or stale during the wait must block the arm at the moment it
  matters, not only at the pre-wait check.

  The real function is exercised directly by including the driver source
  and driving gfcool_fire_ok() from a scripted sequence: the second
  reading models the verdict the button-wait window refreshed. With
  hw_active off (no GFSINK, as on the host) the two checks are the only
  gates in the path, so a scripted good-then-bad sequence isolates the
  post-wait re-check.

  Also covers the button wait itself, with the switch source scripted:
  the lid or the interlock loop opening during the wait cancels the job
  (relock, alarm, never armed), a press with the lid open does not arm,
  and a press with everything closed arms.

  And the dose model: the precompute derives $35 from the floor key
  (density 10 by default; on the host the analog reference mode derives
  its own 16), a typed $35 is overwritten, and the stored settings are
  never touched. Density is the only model on hardware; the analog
  branch exists for the harness's conservatism reference.

  And the gates the audit added: on hardware a missing switch device
  refuses the arm; the head is checked again after the wait; a press
  counts only after the button has been seen up; three unreadable
  switch reads end the wait; a latch that does not unlock refuses the
  arm; the disarm locks the latch whenever this process unlocked it,
  window or no window; the relock at a job's end treats a faulted or
  underrun kernel as done and an unreadable one as done after a bound;
  the verdict's pause tier closes the fire gate and writes no latch,
  the fail tier disarms, resets and alarms; a sender change during a
  re-arm cancels it; check mode never arms; a jog does not hold the
  window open.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* The acknowledgment wait, shortened so the never-acknowledged case
   refuses in a fraction of a second instead of spinning out the
   shipped bound. */
#define COOL_ACK_S 0.2
/* The unreadable-kernel bound on the relock, shortened the same way. */
#define DISARM_UNREADABLE_S 0.05

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

/* gfcool_fire_ok() reads from this script, one entry per call. */
static bool fire_ok_script[4];
static int  fire_ok_n;
static int  fire_ok_calls;
static bool verdict_fire_ok_val = true;     /* gfcool_verdict_fire_ok() */

static int   alarms_raised;
static char  last_message[128];
static char  all_messages[4096];        /* every report, for the multi-step cases */
static bool  stream_armed;          /* gf_stream_laser_arm(true) reached? */
static bool  fire_gate_last;        /* gf_stream_fire_gate() argument */
static uint32_t dose_period_last = 1;  /* gf_stream_laser_model() argument */
static bool  latch_locked_last;     /* the stream engine's ownership record */
static int   latch_writes;
static bool  latch_lock_fail;       /* a lock write does not take */
static bool  latch_unlock_fail;     /* an unlock write does not take */
static int   cool_armed_calls;      /* gfcool_laser_armed() calls */
static bool  cool_armed_last;
static const char *rd_state;            /* cnc/state as gfio_rd_attr reads it, NULL = unreadable */
static int   rd_head_ok_calls = 1 << 30; /* head/hall_sensor reads that succeed before failing */
static const char *conf_model_val;      /* laser_power_model in the config, NULL = absent */
static const char *conf_curve_val;      /* laser_dose_curve, NULL = absent */
static float conf_gamma = -1.0f;        /* laser_corner_gamma, < 0 = absent */
static float conf_floor_analog = -1.0f; /* laser_floor_analog, < 0 = absent */
static float conf_floor_density = -1.0f;
static float conf_button_timeout = -1.0f;   /* laser_button_timeout_s, < 0 = absent */
static float precomputed_min = -1.0f;   /* pwm_min_value at the last precompute */
static int   precompute_calls;
static int   settings_stores;           /* any EEPROM write would count here */

/* Switch source: gfsw_read_raw() plays this script of EV_SW words, one
   per call, holding the last entry. sw_present = gfsw_available().
   Reads from sw_fail_from on fail (unreadable device). */
static bool     sw_present;
static unsigned sw_script[8];
static int      sw_n;
static int      sw_calls;
static int      sw_fail_from = 1 << 30;

bool gfcool_fire_ok(void)
{
    int i = fire_ok_calls < fire_ok_n ? fire_ok_calls : fire_ok_n - 1;
    fire_ok_calls++;
    return fire_ok_script[i];
}

bool gfcool_verdict_fire_ok(void) { return verdict_fire_ok_val; }
static bool decel_lit_val;              /* gfcool_decel_lit() */
bool gfcool_decel_lit(void) { return decel_lit_val; }

/* The cooling engine's acknowledgment of the armed window. False for the
 * first ack_hold calls, so 0 is an engine that has already taken the job
 * (every case that is not about the wait) and a large value is one that
 * never does. */
static int ack_hold;
static int ack_calls;

bool gfcool_run_ack(void)
{
    return ack_calls++ >= ack_hold;
}

bool gfsw_available(void) { return sw_present; }
void gfsw_button_consumed(void) {}
bool gfsw_read_raw(uint8_t *sw)
{
    int i = sw_calls < sw_n ? sw_calls : sw_n - 1;
    bool ok = sw_calls < sw_fail_from;
    sw_calls++;
    sw[0] = (uint8_t)sw_script[i];
    sw[1] = (uint8_t)(sw_script[i] >> 8);
    return ok;
}

void gfcool_laser_armed(bool armed) { cool_armed_calls++; cool_armed_last = armed; }
void gf_stream_laser(unsigned char power, bool fire) { (void)power; (void)fire; }
void gf_stream_laser_arm(bool armed) { stream_armed = armed; }
void gf_stream_jog(bool jog) { (void)jog; }   /* the jog mask lives in the stream */
void gf_stream_fire_gate(bool open) { fire_gate_last = open; }
bool gf_stream_laser_latch(bool lock)
{
    latch_writes++;
    if (lock ? latch_lock_fail : latch_unlock_fail)
        return false;
    latch_locked_last = lock;
    return true;
}
bool gf_stream_latch_locked_by_us(void) { return latch_locked_last; }
int  gfio_rd_attr(const char *a, char *b, size_t l)
{
    if (l) b[0] = '\0';
    if (!strcmp(a, "cnc/state") && rd_state != NULL) {
        snprintf(b, l, "%s", rd_state);
        return 0;
    }
    if (!strcmp(a, "head/hall_sensor") && rd_head_ok_calls-- > 0) {
        snprintf(b, l, "1");
        return 0;
    }
    return -1;
}
float gfio_conf_read_float(const char *k, float fb)
{
    if (!strcmp(k, "laser_floor_analog") && conf_floor_analog >= 0.0f) return conf_floor_analog;
    if (!strcmp(k, "laser_floor_density") && conf_floor_density >= 0.0f) return conf_floor_density;
    if (!strcmp(k, "laser_corner_gamma") && conf_gamma >= 0.0f) return conf_gamma;
    if (!strcmp(k, "laser_button_timeout_s") && conf_button_timeout >= 0.0f) return conf_button_timeout;
    return fb;
}
int gfio_conf_read(const char *k, char *v, size_t n)
{
    if (!strcmp(k, "laser_power_model") && conf_model_val != NULL) {
        snprintf(v, n, "%s", conf_model_val);
        return 0;
    }
    if (!strcmp(k, "laser_dose_curve") && conf_curve_val != NULL) {
        snprintf(v, n, "%s", conf_curve_val);
        return 0;
    }
    return -1;
}
void gf_stream_laser_model(uint32_t period, uint32_t min_ticks) { (void)min_ticks; dose_period_last = period; }
void serial_poll(void) {}
void serial_wait(long us) { (void)us; }
void fflog(int prio, const char *fmt, ...) { (void)prio; (void)fmt; }
void gfhome_reference_z(float z_mm, int below, int above) { (void)z_mm; (void)below; (void)above; }
static unsigned client_gen = 1;     /* bumped to model a sender change */
unsigned serial_client_generation(void) { return client_gen; }
static int resets_requested;
bool protocol_enqueue_realtime_command(uint8_t c) { if (c == 0x18) resets_requested++; return true; }
/* The reset lands in the next pump: protocol_execute_realtime() returns
   false (aborting) once a reset has been requested. */
bool protocol_execute_realtime(void) { return resets_requested == 0; }

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_laser.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
settings_t settings;
grbl_t grbl;
parser_state_t gc_state;
system_t sys;
static spindle_ptrs_t test_spindle;
static spindle_param_t test_param;      /* the active spindle's param: the segment's velocity ratio */
static sys_state_t cur_state = STATE_IDLE;  /* what state_get() reports */

void report_message(const char *msg, message_type_t type)
{
    (void)type;
    snprintf(last_message, sizeof(last_message), "%s", msg ? msg : "");
    size_t n = strlen(all_messages);
    snprintf(all_messages + n, sizeof(all_messages) - n, "%s\n", msg ? msg : "");
}
void system_raise_alarm(alarm_code_t alarm) { (void)alarm; alarms_raised++; }
sys_state_t state_get(void) { return cur_state; }

static void sleep_s(double s)
{
    struct timespec ts = { .tv_sec = (time_t)s, .tv_nsec = (long)((s - (double)(time_t)s) * 1e9) };
    nanosleep(&ts, NULL);
}
bool spindle_precompute_pwm_values(spindle_ptrs_t *s, spindle_pwm_t *p,
                                   spindle_pwm_settings_t *cfg, uint32_t hz)
{ (void)s; (void)p; (void)hz; precomputed_min = cfg->pwm_min_value; precompute_calls++; return true; }
/* The core's persisted-settings writers: the driver must never call them. */
void settings_write_global(void) { settings_stores++; }
spindle_id_t spindle_register(const spindle_ptrs_t *s, const char *n)
{ (void)s; (void)n; return 0; }

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void reset_state(void)
{
    fire_ok_calls = 0;
    ack_hold = 0;
    ack_calls = 0;
    alarms_raised = 0;
    resets_requested = 0;
    last_message[0] = '\0';
    all_messages[0] = '\0';
    stream_armed = false;
    fire_gate_last = false;
    latch_locked_last = true;
    latch_writes = 0;
    latch_lock_fail = latch_unlock_fail = false;
    cool_armed_calls = 0;
    cool_armed_last = false;
    rd_state = NULL;
    rd_head_ok_calls = 1 << 30;
    verdict_fire_ok_val = true;
    decel_lit_val = false;
    conf_button_timeout = -1.0f;
    cur_state = STATE_IDLE;
    memset(&sys, 0, sizeof(sys));
    laser_ok = false;
    disarm_request = false;
    disarm_at = 0.0;
    rearm_pending = false;
    fail_alarm_pending = false;
    arming = false;
    atomic_store(&cur_state_value, 0);
    client_gen = 1;
    hw_active = false;              /* host: no GFSINK */
    sw_present = false;             /* no switch source: no button wait */
    sw_calls = 0;
    sw_n = 0;
    sw_fail_from = 1 << 30;
    conf_model_val = NULL;
    conf_curve_val = NULL;
    conf_gamma = -1.0f;
    conf_floor_analog = conf_floor_density = -1.0f;
    memset(&test_param, 0, sizeof(test_param));
    test_param.rate_ratio = 1.0f;
    rate_param = &test_param;
    precomputed_min = -1.0f;
    precompute_calls = 0;
    settings_stores = 0;
    settings.pwm_spindle.pwm_min_value = 0.0f;
    hal_spindle = &test_spindle;    /* the core's table entry, as spindleConfig records it */
    cur_model = Model_Density;
    dose_period_last = 1;
}


static void script(bool a, bool b, int n)
{
    fire_ok_script[0] = a;
    fire_ok_script[1] = b;
    fire_ok_n = n;
}

/* EV_SW words: bit 2 button, bit 3 doors (closed = set), bit 5 interlock
   loop (set = OPEN). */
#define W_CLOSED         (1u << SW_BIT_DOORS)
#define W_PRESSED        (W_CLOSED | (1u << SW_BIT_BUTTON))
#define W_LID_OPEN       0u
#define W_LID_OPEN_PRESS (1u << SW_BIT_BUTTON)
#define W_LOOP_OPEN      (W_CLOSED | (1u << SW_BIT_INTERLOCK))

static void switches(unsigned a, unsigned b, unsigned c, int n)
{
    sw_present = true;
    sw_script[0] = a;
    sw_script[1] = b;
    sw_script[2] = c;
    sw_n = n;
}

int main(void)
{
    printf("gflaser_arm() coolant re-check (G-4):\n");

    /* Case A - the G-4 case: the gate is clear at the pre-wait check but
       has gone bad by the post-wait re-check. Arming must refuse. */
    reset_state();
    script(true, false, 2);
    bool armed_a = gflaser_arm();
    CHECK(!armed_a, "refuses when the verdict goes bad during the wait");
    CHECK(!laser_ok, "armed window stays closed on the late refusal");
    CHECK(!stream_armed, "stream is never told armed on the late refusal");
    CHECK(latch_locked_last, "latch is left locked on the late refusal");
    CHECK(alarms_raised == 1, "raises an alarm on the late refusal");
    CHECK(fire_ok_calls == 2, "the coolant gate is re-checked after the wait");
    CHECK(strstr(last_message, "blocked") != NULL,
          "reports the fire-blocked reason");

    /* Case B - clear at both checks: arming succeeds. */
    reset_state();
    script(true, true, 2);
    bool armed_b = gflaser_arm();
    CHECK(armed_b, "arms when the gate is clear at both checks");
    CHECK(laser_ok, "armed window opens when clear");
    CHECK(stream_armed, "stream is told armed when clear");
    /* Density is the shipped default: with no laser_power_model key the
     * arm selects it, and passes the base period rather than 0. */
    CHECK(dose_period_last == 20, "no laser_power_model key selects the density dose model");
    CHECK(settings.pwm_spindle.pwm_min_value == 10.0f && precomputed_min == 10.0f,
          "the arm derives $35 from the density floor (10) and re-precomputes");
    CHECK(strstr(last_message, "laser armed (density, floor 10 %, curve bench-default)") != NULL,
          "the arm report names the model, the floor and the curve in force");

    /* Case C - blocked at the pre-wait check: refuses before arming. */
    reset_state();
    script(false, false, 2);
    bool armed_c = gflaser_arm();
    CHECK(!armed_c, "refuses when the gate is blocked up front");
    CHECK(!laser_ok, "armed window stays closed on the early refusal");
    CHECK(fire_ok_calls == 1, "the pre-wait check short-circuits the arm");

    /* Case A2 - the engine never takes the job. The armed window is
       reported to the cooling engine when it opens, and the engine
       answers by applying the cut airflow and the flow interrogation.
       The verdict standing on file until then was computed for the idle
       session before the arm, and at idle nothing is wrong, so it says
       fire is fine - firing on it puts the beam on the work with the
       fans at their idle duty. The gate here reads clear at both checks,
       so the missing acknowledgment is the only thing that can refuse. */
    reset_state();
    script(true, true, 2);
    ack_hold = 1 << 30;
    bool armed_a2 = gflaser_arm();
    CHECK(!armed_a2, "refuses when the engine never takes the armed window");
    CHECK(!laser_ok, "armed window stays closed without the acknowledgment");
    CHECK(!stream_armed, "stream is never told armed without the acknowledgment");
    CHECK(latch_locked_last, "latch is left locked without the acknowledgment");
    CHECK(alarms_raised == 1, "raises an alarm when the engine does not take the job");
    CHECK(strstr(last_message, "did not take the job") != NULL,
          "names the cooling service as the reason");

    /* Case A3 - the acknowledgment arriving a few polls late is the
       ordinary case: the engine sees the report on its next tick. The
       arm waits for it and then proceeds. */
    reset_state();
    script(true, true, 2);
    ack_hold = 3;
    bool armed_a3 = gflaser_arm();
    CHECK(armed_a3, "arms once the engine takes the armed window");
    CHECK(laser_ok && stream_armed, "the armed window opens after the wait");
    CHECK(ack_calls > 3, "the arm waited for the acknowledgment");
    CHECK(alarms_raised == 0, "a late acknowledgment is not an alarm");

    printf("gflaser_arm() button wait against the switches:\n");

    /* Case D - the operator presses with everything closed: arms. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    bool armed_d = gflaser_arm();
    CHECK(armed_d, "press with lid and loop closed arms");
    CHECK(laser_ok && stream_armed, "armed window opens on the press");
    CHECK(alarms_raised == 0, "no alarm on a clean arm");

    /* Case E - the lid opens during the wait: the job is cancelled. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_LID_OPEN, 3);
    bool armed_e = gflaser_arm();
    CHECK(!armed_e, "lid open during the wait refuses");
    CHECK(!laser_ok && !stream_armed, "armed window never opens on a lid open");
    CHECK(latch_locked_last, "latch relocked on the lid open");
    CHECK(resets_requested == 1 && alarms_raised == 0, "lid open cancels with a soft reset, not an alarm");
    CHECK(strstr(last_message, "lid opened") != NULL, "reports the lid as the reason");
    CHECK(fire_ok_calls == 1, "the post-wait coolant check is not reached");

    /* Case F - a press with the lid open does not arm; the lid wins. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_LID_OPEN_PRESS, W_LID_OPEN_PRESS, 3);
    bool armed_f = gflaser_arm();
    CHECK(!armed_f, "press with the lid open does not arm");
    CHECK(!laser_ok && latch_locked_last, "lid-open press leaves the latch locked");
    CHECK(strstr(last_message, "lid opened") != NULL, "lid-open press reported as the lid");

    /* Case G - the interlock loop opens during the wait: cancelled too. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_LOOP_OPEN, W_LOOP_OPEN, 3);
    bool armed_g = gflaser_arm();
    CHECK(!armed_g, "interlock open during the wait refuses");
    CHECK(!laser_ok && latch_locked_last, "interlock open leaves the latch locked");
    CHECK(resets_requested == 1 && alarms_raised == 0, "interlock open cancels with a soft reset, not an alarm");
    CHECK(strstr(last_message, "interlock") != NULL, "reports the interlock as the reason");

    printf("spindleSetState() arms on every laser-on while the window is closed:\n");

    /* Case H - a window that closed while the core still had the spindle
       on (a sender change mid-job, or a job whose M5 never arrived) must
       prompt again at the next laser-on. The spindle-state record stays
       on across the disarm; the arm decision must not read it. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    spindle_state_t on = { .on = On };
    spindleSetState(NULL, on, 1000.0f);
    CHECK(laser_ok && stream_armed, "the first laser-on arms through the press");
    CHECK(((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "the spindle-state record reads on after the arm");
    client_gen++;                       /* the sender changed mid-job */
    gflaser_poll();
    CHECK(!laser_ok && !stream_armed && latch_locked_last,
          "the sender change closes the window and relocks the latch");
    CHECK(((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "the spindle-state record still reads on after the disarm");
    sw_calls = 0;
    fire_ok_calls = 0;
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    spindleSetState(NULL, on, 1000.0f);
    CHECK(sw_calls > 0, "the next laser-on runs the button wait again");
    CHECK(laser_ok && stream_armed, "the next laser-on arms again through a new press");
    CHECK(alarms_raised == 0, "no alarm on the re-arm");

    /* Case I - inside an open window a laser-on never re-prompts (S changes
       and M5/M3 toggles inside a job are not new consent questions). */
    sw_calls = 0;
    spindleSetState(NULL, on, 500.0f);
    CHECK(sw_calls == 0 && laser_ok, "a laser-on inside the open window does not re-prompt");

    printf("the dose model: derived floors:\n");

    /* Case J - the configured analog model derives the analog floor, and
       the floor keys override the defaults. A stale $35 in RAM (a sender
       wrote 3) is overwritten, never honored. */
    reset_state();
    script(true, true, 2);
    conf_model_val = "analog";
    settings.pwm_spindle.pwm_min_value = 3.0f;
    CHECK(gflaser_arm(), "arms under the configured analog model");
    CHECK(dose_period_last == 0, "laser_power_model = analog selects the analog rendering");
    CHECK(settings.pwm_spindle.pwm_min_value == 16.0f && precomputed_min == 16.0f,
          "the arm derives $35 from the analog floor (16), overwriting a typed value");
    CHECK(strstr(last_message, "(analog, floor 16 %, curve bench-default)") != NULL,
          "the arm report says analog, 16, and names the curve");
    reset_state();
    script(true, true, 2);
    conf_floor_density = 12.5f;
    CHECK(gflaser_arm() && settings.pwm_spindle.pwm_min_value == 12.5f,
          "laser_floor_density overrides the density default");
    reset_state();
    script(true, true, 2);
    conf_floor_density = 150.0f;    /* out of range: the default applies */
    CHECK(gflaser_arm() && settings.pwm_spindle.pwm_min_value == 10.0f,
          "an out-of-range floor key falls back to the default");
    reset_state();
    script(true, true, 2);
    conf_floor_density = 0.0f;
    CHECK(gflaser_arm() && settings.pwm_spindle.pwm_min_value == 0.0f,
          "a floor of 0 is honored (the ladders run that way)");

    printf("the dose curve: parse, apply, fall back:\n");

    /* Case P - the compiled default loads with no key, and the wrapper
       maps commanded light through its inverse: half light lands near
       80 percent density, full stays full, and tiny commands ride the
       first span. The core's own math handles rpm 0 and the floor. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm(), "arms with the default curve");
    CHECK(strcmp(gflaser_curve(), "bench-default") == 0, "no key loads the bench default");
    {
        spindle_pwm_t pd;
        memset(&pd, 0, sizeof(pd));
        pd.rpm_min = 0.0f;
        pd.min_value = 12;
        pd.max_value = 127;
        test_spindle.rpm_min = 0.0f;
        test_spindle.rpm_max = 1000.0f;
        conf_gamma = 1.0f;              /* the pure curve; gamma has case R */
        gamma_load();
        uint_fast16_t half = curveComputeValue(&pd, 500.0f, false);
        uint_fast16_t full = curveComputeValue(&pd, 1000.0f, false);
        uint_fast16_t low = curveComputeValue(&pd, 50.0f, false);
        uint_fast16_t least = curveComputeValue(&pd, 5.0f, false);
        CHECK(half >= 97 && half <= 105,
              "S500 (half light) maps near 80 percent density through the curve");
        CHECK(full == 127, "S1000 maps to full");
        CHECK(low >= 30 && low <= 36,
              "S50 (5 percent light) maps through the low span to ~26 percent density");
        CHECK(least >= 12 && least <= 14,
              "the least command lands at the curve's first point, at or above the floor");
        CHECK(least < low && low < half && half < full, "the mapping is monotonic");
        conf_gamma = -1.0f;
        gamma_load();
    }

    /* Case Q - "off" is the identity, a bad value falls back loudly. */
    reset_state();
    script(true, true, 2);
    conf_curve_val = "off";
    CHECK(gflaser_arm() && strcmp(gflaser_curve(), "off") == 0, "curve off is honored");
    CHECK(strstr(last_message, "curve off") != NULL, "the arm names the off curve");
    reset_state();
    script(true, true, 2);
    conf_curve_val = "10:5,9:6";        /* densities not increasing */
    CHECK(gflaser_arm() && strcmp(gflaser_curve(), "invalid: bench-default") == 0,
          "a bad curve falls back to the default and says so");
    reset_state();
    script(true, true, 2);
    conf_curve_val = "10:1,50:20,100:100";
    CHECK(gflaser_arm() && strcmp(gflaser_curve(), "custom") == 0, "a valid custom curve loads");

    printf("the corner rolloff exponent:\n");

    /* Case R - with the default gamma of 2, a velocity-scaled command
       (the core hands over rpm scaled by the segment's velocity ratio and
       records that ratio in the spindle's param) delivers light bent by
       the ratio, while a command at ratio 1 is untouched. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm(), "arms with the default gamma");
    {
        spindle_pwm_t pd;
        memset(&pd, 0, sizeof(pd));
        pd.rpm_min = 0.0f;
        pd.min_value = 12;
        pd.max_value = 127;
        test_spindle.rpm_min = 0.0f;
        test_spindle.rpm_max = 1000.0f;
        test_param.rate_ratio = 1.0f;   /* programmed S1000 at full speed */
        uint_fast16_t at_speed = curveComputeValue(&pd, 1000.0f, false);
        test_param.rate_ratio = 0.5f;   /* the same S at half speed */
        uint_fast16_t corner = curveComputeValue(&pd, 500.0f, false);
        /* Half speed at gamma 2: light = 0.5 * 0.5 = 0.25 -> density
           through the curve between 45 and 60 percent (~61 counts). */
        CHECK(at_speed == 127, "full speed delivers the programmed light");
        CHECK(corner >= 56 && corner <= 66,
              "half speed at gamma 2 delivers a quarter of the light");
        test_param.rate_ratio = 1.0f;   /* programmed S500, full speed: no scaling */
        uint_fast16_t prog = curveComputeValue(&pd, 500.0f, false);
        CHECK(prog >= 97 && prog <= 105,
              "the same rpm at ratio 1 is untouched");
        /* The parser's newest S is nowhere in this: a queued S1000 line
           changes nothing about the S500 block's segments. */
        test_param.rate_ratio = 1.0f;
        CHECK(curveComputeValue(&pd, 500.0f, false) == prog,
              "the ratio is the segment's own, not the newest S over the block's");
        conf_gamma = 1.0f;
        gamma_load();
        test_param.rate_ratio = 0.5f;
        uint_fast16_t plain = curveComputeValue(&pd, 500.0f, false);
        CHECK(plain >= 97 && plain <= 105,
              "gamma 1 is plain proportionality (half speed = half light)");
        conf_gamma = 9.0f;              /* out of range: the default */
        gamma_load();
        CHECK(corner_gamma == 2.0f, "an out-of-range gamma falls back to 2");
        test_param.rate_ratio = 1.0f;
    }

    printf("the switch device on hardware:\n");

    /* Case S1 - hardware with no switch device refuses the arm outright:
       no unlock, no LED, an alarm. The host build (no GFSINK) keeps the
       null-sink auto-arm (Case B above). */
    reset_state();
    script(true, true, 2);
    hw_active = true;
    sw_present = false;
    CHECK(!gflaser_arm(), "hardware with no switch device refuses the arm");
    CHECK(!laser_ok && !stream_armed, "no window opens without a switch device");
    CHECK(latch_locked_last && latch_writes == 0, "the latch is never unlocked without a switch device");
    CHECK(alarms_raised == 1, "the refusal is an alarm");
    CHECK(strstr(last_message, "no switch device") != NULL, "the refusal names the missing switch device");
    CHECK(cool_armed_calls == 0, "the engine is never told armed without a switch device");

    /* Case S16 - the head is checked again after the wait: present at
       the gates, lifted during the wait, refused at the completion. */
    reset_state();
    script(true, true, 2);
    hw_active = true;
    rd_head_ok_calls = 1;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    CHECK(!gflaser_arm(), "a head lifted during the wait refuses the arm");
    CHECK(!laser_ok && latch_locked_last, "the window stays closed and the latch relocks on the late head check");
    CHECK(strstr(last_message, "no head") != NULL, "the late refusal names the head");
    CHECK(alarms_raised == 1, "the late head refusal is an alarm");

    printf("the button must be seen up before a press counts:\n");

    /* Case S11 - a button already down when the wait starts is not a
       press: the wait runs on (here to the lid opening); a release and
       a fresh press arms. */
    reset_state();
    script(true, true, 2);
    switches(W_PRESSED, W_PRESSED, W_LID_OPEN, 3);
    CHECK(!gflaser_arm(), "a button held from before the wait does not arm");
    CHECK(sw_calls >= 3, "the wait ran past the held button");
    CHECK(!laser_ok && latch_locked_last, "the held button leaves the window closed and the latch locked");
    reset_state();
    script(true, true, 2);
    switches(W_PRESSED, W_CLOSED, W_PRESSED, 3);
    CHECK(gflaser_arm() && laser_ok, "a release and a fresh press arm");

    /* Case S17 - three unreadable switch reads in a row end the wait:
       relock, alarm, never armed. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    sw_fail_from = 0;
    CHECK(!gflaser_arm(), "unreadable switches end the wait");
    CHECK(sw_calls == 3, "the wait ends at the third failed read");
    CHECK(!laser_ok && latch_locked_last, "the unreadable wait leaves the window closed and the latch locked");
    CHECK(alarms_raised == 1, "the unreadable wait is an alarm");
    CHECK(strstr(last_message, "cannot be read") != NULL, "the refusal names the switches");

    printf("the latch writes:\n");

    /* Case S6 - an unlock that does not take refuses the arm. */
    reset_state();
    script(true, true, 2);
    latch_unlock_fail = true;
    CHECK(!gflaser_arm(), "a latch that does not unlock refuses the arm");
    CHECK(!laser_ok && !stream_armed, "no window opens on a failed unlock");
    CHECK(latch_locked_last, "the ownership record still says locked after the failed unlock");
    CHECK(alarms_raised == 1, "the failed unlock is an alarm");
    CHECK(strstr(last_message, "did not unlock") != NULL, "the refusal names the latch");
    CHECK(cool_armed_last == false, "the engine is told the window is not armed after the failed unlock");

    /* Case S5 - the disarm locks the latch whenever this process
       unlocked it, window or no window (a reset in the button wait); a
       latch already locked and no window is left alone. */
    reset_state();
    latch_locked_last = false;      /* the arm wait's unlock, no window yet */
    gflaser_disarm();
    CHECK(latch_locked_last && latch_writes == 1, "a disarm with no window relocks a latch this process unlocked");
    CHECK(strstr(all_messages, "disarmed") == NULL, "no disarm message when no window was open");
    reset_state();
    gflaser_disarm();
    CHECK(latch_writes == 0, "a disarm with the latch locked and no window writes nothing");

    printf("the relock at a job's end:\n");

    /* Case S4 - program end on hardware: an idle, faulted or underrun
       kernel relocks at once; a running one waits; an unreadable one
       relocks after the bound, never "still playing" forever. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the program-end cases");
    hw_active = true;
    disarm_request = true;
    rd_state = "running";
    gflaser_poll();
    CHECK(laser_ok, "the relock waits while the kernel plays the tail");
    rd_state = "fault";
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "a faulted kernel relocks at once");
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed again");
    hw_active = true;
    disarm_request = true;
    rd_state = "underrun";
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "an underrun kernel relocks at once");
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed again");
    hw_active = true;
    disarm_request = true;
    rd_state = NULL;
    gflaser_poll();
    CHECK(laser_ok, "an unreadable kernel state defers the relock inside the bound");
    sleep_s(0.08);
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "an unreadable kernel state relocks after the bound");

    printf("the cooling verdict's tiers under an open window:\n");

    /* Case V1 - the pause tier: the fire gate follows the verdict down
       and up again; the window stays open and the latch is never
       written (a lock would set the hardware button latch). */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the verdict cases");
    CHECK(fire_gate_last, "the fire gate opens with the window");
    CHECK(!latch_locked_last && latch_writes == 1, "the latch is unlocked inside the window");
    verdict_fire_ok_val = false;
    decel_lit_val = true;
    gflaser_poll();
    CHECK(fire_gate_last, "the fire gate stays open through the pause's first deceleration");
    decel_lit_val = false;
    gflaser_poll();
    CHECK(!fire_gate_last, "the fire gate follows the verdict down once the head has stopped");
    CHECK(laser_ok && stream_armed && !latch_locked_last && latch_writes == 1,
          "a blocked verdict keeps the window open and writes no latch");
    verdict_fire_ok_val = true;
    gflaser_poll();
    CHECK(fire_gate_last, "the fire gate follows the verdict up");
    CHECK(laser_ok && !latch_locked_last && latch_writes == 1,
          "the clean verdict finds the window open and the latch untouched");
    CHECK(alarms_raised == 0 && resets_requested == 0, "the pause tier neither alarms nor resets");

    /* Case V2 - the fail tier: disarm, latch locked, the job reset, and
       the alarm once the reset has landed at a standstill. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the fail tier");
    gflaser_verdict_fail("AIRFLOW");
    CHECK(!laser_ok && !stream_armed && latch_locked_last, "the fail tier closes the window and locks the latch");
    CHECK(!fire_gate_last, "the fail tier closes the fire gate");
    CHECK(resets_requested == 1, "the fail tier resets the job");
    CHECK(strstr(all_messages, "AIRFLOW") != NULL, "the fail tier names the verdict");
    gflaser_poll();
    CHECK(alarms_raised == 1, "the fail tier alarms once the reset has landed");
    gflaser_poll();
    CHECK(alarms_raised == 1, "the fail-tier alarm is raised once");
    gflaser_verdict_fail("CRASH");
    CHECK(resets_requested == 1, "a fail tier with no window open does nothing");

    printf("the re-arm of a held job:\n");

    /* Case S12 - a sender change during the re-arm cancels it: the
       press must not resume another session's held job. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    CHECK(gflaser_resume_gate() && rearm_pending, "the resume gate takes a held job's resume");
    CHECK(!latch_locked_last, "the re-arm unlocks the latch for the press");
    client_gen++;
    gflaser_poll();
    CHECK(!rearm_pending && latch_locked_last, "a sender change during the re-arm cancels it and relocks");
    CHECK(!laser_ok, "the canceled re-arm opens no window");
    CHECK(strstr(all_messages, "sender changed") != NULL, "the canceled re-arm names the sender change");
    gc_state.modal.spindle[0].state.on = Off;

    /* Case S17b - unreadable switches during the re-arm cancel it too. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    CHECK(gflaser_resume_gate() && rearm_pending, "the resume gate takes the resume again");
    sw_fail_from = 0;
    gflaser_poll();
    gflaser_poll();
    CHECK(rearm_pending, "two unreadable reads keep the re-arm waiting");
    gflaser_poll();
    CHECK(!rearm_pending && latch_locked_last && alarms_raised == 1,
          "the third unreadable read cancels the re-arm, relocks and alarms");
    gc_state.modal.spindle[0].state.on = Off;

    printf("check mode and the jog grace:\n");

    /* Case S15 - $C check mode never arms: no wait, no unlock. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    cur_state = STATE_CHECK_MODE;
    spindleSetState(NULL, on, 1000.0f);
    CHECK(!laser_ok && sw_calls == 0 && latch_writes == 0, "check mode never arms, waits or unlocks");
    CHECK(!((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "check mode records the spindle off");

    /* Case S10 - a jog does not hold the window open: a running grace
       expires under STATE_JOG; a cycle still resets it. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the grace cases");
    atomic_store(&cur_state_value, 0);          /* spindle off */
    cur_state = STATE_CYCLE;
    disarm_at = 1.0;                            /* a grace that has expired */
    gflaser_poll();
    CHECK(laser_ok && disarm_at == 0.0, "a cycle resets the grace");
    cur_state = STATE_JOG;
    disarm_at = 1.0;
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "a jog does not reset the grace: the window closes");

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the arm re-checks the coolant gate after the wait, "
                      "the button wait honors the lid and the interlock, and "
                      "every laser-on against a closed window prompts, and the "
                      "floor is derived, never typed\n",
           failures);
    return failures ? 1 : 0;
}
