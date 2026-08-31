/*
  laser_arm_test.c - host unit test for the operator-arm coolant re-check

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
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
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

/* gfcool_fire_ok() reads from this script, one entry per call. */
static bool fire_ok_script[4];
static int  fire_ok_n;
static int  fire_ok_calls;

static int   alarms_raised;
static char  last_message[128];
static bool  stream_armed;          /* gf_stream_laser_arm(true) reached? */
static uint32_t dose_period_last = 1;  /* gf_stream_laser_model() argument */
static bool  latch_locked_last;
static const char *conf_model_val;      /* laser_power_model in the config, NULL = absent */
static const char *conf_curve_val;      /* laser_dose_curve, NULL = absent */
static float conf_gamma = -1.0f;        /* laser_corner_gamma, < 0 = absent */
static float conf_floor_analog = -1.0f; /* laser_floor_analog, < 0 = absent */
static float conf_floor_density = -1.0f;
static float precomputed_min = -1.0f;   /* pwm_min_value at the last precompute */
static int   precompute_calls;
static int   settings_stores;           /* any EEPROM write would count here */

/* Switch source: gfsw_read_raw() plays this script of EV_SW words, one
   per call, holding the last entry. sw_present = gfsw_available(). */
static bool     sw_present;
static unsigned sw_script[8];
static int      sw_n;
static int      sw_calls;

bool gfcool_fire_ok(void)
{
    int i = fire_ok_calls < fire_ok_n ? fire_ok_calls : fire_ok_n - 1;
    fire_ok_calls++;
    return fire_ok_script[i];
}

bool gfsw_available(void) { return sw_present; }
void gfsw_button_consumed(void) {}
bool gfsw_read_raw(uint8_t *sw)
{
    int i = sw_calls < sw_n ? sw_calls : sw_n - 1;
    sw_calls++;
    sw[0] = (uint8_t)sw_script[i];
    sw[1] = (uint8_t)(sw_script[i] >> 8);
    return true;
}

void gfcool_laser_armed(bool armed) { (void)armed; }
void gf_stream_laser(unsigned char power, bool fire) { (void)power; (void)fire; }
void gf_stream_laser_arm(bool armed) { stream_armed = armed; }
void gf_stream_laser_latch(bool lock) { latch_locked_last = lock; }
int  gfio_rd_attr(const char *a, char *b, size_t l)
{ (void)a; if (l) b[0] = '\0'; return -1; }
float gfio_conf_read_float(const char *k, float fb)
{
    if (!strcmp(k, "laser_floor_analog") && conf_floor_analog >= 0.0f) return conf_floor_analog;
    if (!strcmp(k, "laser_floor_density") && conf_floor_density >= 0.0f) return conf_floor_density;
    if (!strcmp(k, "laser_corner_gamma") && conf_gamma >= 0.0f) return conf_gamma;
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
static spindle_ptrs_t test_spindle;

void report_message(const char *msg, message_type_t type)
{
    (void)type;
    snprintf(last_message, sizeof(last_message), "%s", msg ? msg : "");
}
void system_raise_alarm(alarm_code_t alarm) { (void)alarm; alarms_raised++; }
sys_state_t state_get(void) { return STATE_IDLE; }
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
    alarms_raised = 0;
    resets_requested = 0;
    last_message[0] = '\0';
    stream_armed = false;
    latch_locked_last = true;
    laser_ok = false;
    disarm_request = false;
    atomic_store(&cur_state_value, 0);
    client_gen = 1;
    hw_active = false;              /* host: no GFSINK */
    sw_present = false;             /* no switch source: no button wait */
    sw_calls = 0;
    sw_n = 0;
    conf_model_val = NULL;
    conf_curve_val = NULL;
    conf_gamma = -1.0f;
    conf_floor_analog = conf_floor_density = -1.0f;
    prog_rpm_set(0.0f);
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
       (rpm below the programmed S in param->rpm) delivers light bent by
       the ratio, while the programmed command itself is untouched. */
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
        prog_rpm_set(1000.0f);          /* programmed S1000 */
        uint_fast16_t at_speed = curveComputeValue(&pd, 1000.0f, false);
        uint_fast16_t corner = curveComputeValue(&pd, 500.0f, false);
        /* Half speed at gamma 2: light = 0.5 * 0.5 = 0.25 -> density
           through the curve between 45 and 60 percent (~61 counts). */
        CHECK(at_speed == 127, "full speed delivers the programmed light");
        CHECK(corner >= 56 && corner <= 66,
              "half speed at gamma 2 delivers a quarter of the light");
        prog_rpm_set(500.0f);           /* programmed S500: no scaling */
        uint_fast16_t prog = curveComputeValue(&pd, 500.0f, false);
        CHECK(prog >= 97 && prog <= 105,
              "the same rpm as the programmed S is untouched (ratio 1)");
        conf_gamma = 1.0f;
        gamma_load();
        prog_rpm_set(1000.0f);
        uint_fast16_t plain = curveComputeValue(&pd, 500.0f, false);
        CHECK(plain >= 97 && plain <= 105,
              "gamma 1 is plain proportionality (half speed = half light)");
        conf_gamma = 9.0f;              /* out of range: the default */
        gamma_load();
        CHECK(corner_gamma == 2.0f, "an out-of-range gamma falls back to 2");
        prog_rpm_set(0.0f);
    }

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the arm re-checks the coolant gate after the wait, "
                      "the button wait honors the lid and the interlock, and "
                      "every laser-on against a closed window prompts, and the "
                      "floor is derived, never typed\n",
           failures);
    return failures ? 1 : 0;
}
