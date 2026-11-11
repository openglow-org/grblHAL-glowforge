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
  (density 10 by default), an out-of-range key falls back to it, a 0 is
  honored, and a typed $35 is overwritten.

  And the acknowledgment's budget: the fan grace (cool_fan_grace_s)
  stretches it past COOL_ACK_S, so an engine that takes the job inside
  the grace still arms, and an out-of-range grace falls back to 15 s.

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

  And the end of a refused or canceled arm: every one alarms (unless an
  alarm already stands) and resets the job, so the parser drops the job's
  M3/M4 and the next laser-on asks for the press again.

  And the button lamp as the LEDs read it (GF_LED_ROOT, a scratch tree):
  breathing white while a press is waited for, solid white while the
  machine moves, dark otherwise, each write stopping a pulse first, and a
  lamp that something else changed written back. The driver's clock is
  stepped past a deadline (clock_skip), never slept through.
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
/* The wait of a laser-on for the tail of the job before it, shortened. */
#define RELOCK_WAIT_S 0.1
/* The lamp's check against the LED, shortened for the test. */
#define LAMP_KEEP_S 0.05
/* The lamp the driver last wrote (its own static, defined by the source
 * below), and what it was at the moment a press was read. */
static int lamp_now;
static int lamp_at_press = -2;

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
static float conf_fan_grace = -1.0f;        /* cool_fan_grace_s, < 0 = absent */
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
static bool ack_waits_val;              /* gfcool_ack_waits(): the verdict is ACK */
bool gfcool_ack_waits(void) { return ack_waits_val; }
/* gfcool_hold_stands(), as the client decides it: a fresh verdict short of
   the fail tier that blocks firing by name (pause_val) or by a fault
   waiting for its Acknowledge (ack_waits_val), and never the fail tier
   (fail_tier_val). */
static bool pause_val;
static bool fail_tier_val;
bool gfcool_hold_stands(void) { return !fail_tier_val && (pause_val || ack_waits_val); }
static bool decel_lit_val;              /* gfcool_decel_lit() */
bool gfcool_decel_lit(void) { return decel_lit_val; }

/* The cooling engine's acknowledgment of the armed window. False for the
 * first ack_hold calls, so 0 is an engine that has already taken the job
 * (every case that is not about the wait) and a large value is one that
 * never does. */
static int ack_hold;
static int ack_calls;
/* Or by time: with ack_after_s >= 0 the engine takes the job that long
 * after the arm first asks. */
static double ack_after_s = -1.0;
static double ack_first_s;

static double mono_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

bool gfcool_run_ack(void)
{
    if (ack_after_s >= 0.0) {
        double now = mono_now();
        if (ack_calls++ == 0)
            ack_first_s = now;
        return now - ack_first_s >= ack_after_s;
    }
    return ack_calls++ >= ack_hold;
}

bool gfsw_available(void) { return sw_present; }
void gfsw_button_consumed(void) {}
/* Events under the button wait, keyed to the switch reads (< 0 = off):
   past this many reads the sender generation moves on, or the core
   reports an alarm. */
static int serial_gen_bump_after = -1;
static int alarm_after_reads = -1;

bool gfsw_read_raw(uint8_t *sw)
{
    int i = sw_calls < sw_n ? sw_calls : sw_n - 1;
    bool ok = sw_calls < sw_fail_from;
    sw_calls++;
    if (sw_script[i] & (1u << 2))       /* the button's bit (SW_BIT_BUTTON, defined below) */
        lamp_at_press = lamp_now;
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
    if (!strcmp(k, "cool_fan_grace_s") && conf_fan_grace >= 0.0f) return conf_fan_grace;
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
/* A sender change under the wait for the acknowledgment, keyed to the
   engine being asked (< 0 = off): past this many gfcool_run_ack() calls
   the sender generation has moved on. */
static int gen_bump_after_ack_calls = -1;
unsigned serial_client_generation(void)
{
    return client_gen + (serial_gen_bump_after >= 0 && sw_calls > serial_gen_bump_after)
                      + (gen_bump_after_ack_calls >= 0 && ack_calls > gen_bump_after_ack_calls);
}
static int resets_requested;
static int cycle_starts;                /* CMD_CYCLE_START (0x81) enqueued */
bool protocol_enqueue_realtime_command(uint8_t c)
{
    if (c == 0x18)
        resets_requested++;
    if (c == 0x81)
        cycle_starts++;
    return true;
}
/* The kernel's tail under a pump: with tail_pumps > 0, cnc/state reads as
   it was set for that many more pumps and "idle" after them. The latch as
   it stood while the tail played is recorded. */
static int  tail_pumps;
static int  pumps;
static bool latch_locked_in_tail;
/* The core's suspend loop. In a held state protocol_execute_realtime()
   does not return until the hold ends: protocol_exec_rt_suspend() spins
   on sys.suspend and runs the realtime hook - the driver's poll - on every
   turn. With suspend_model set the stub does the same while the core
   reports Hold or Door: it counts the entry and runs the poll for
   SUSPEND_TURNS turns (a test cannot spin until a hold that never ends).
   A call that arrives here from the poll is one the core would not return
   from. */
#define SUSPEND_TURNS 20
void gflaser_poll(void);
static bool test_held(void);            /* defined below, with the state the core reports */
static bool suspend_model;
static int  suspend_entries;
static int  suspend_depth;
/* The reset lands in the next pump: protocol_execute_realtime() returns
   false (aborting) once a reset has been requested. */
bool protocol_execute_realtime(void)
{
    pumps++;
    if (tail_pumps > 0) {
        latch_locked_in_tail |= latch_locked_last;
        if (--tail_pumps == 0)
            rd_state = "idle";
    }
    if (suspend_model && test_held() && suspend_depth < 8) {
        suspend_entries++;
        suspend_depth++;
        for (int i = 0; i < SUSPEND_TURNS && test_held(); i++)
            gflaser_poll();
        suspend_depth--;
    }
    return resets_requested == 0;
}

/* The driver's clock: wall_s() reads CLOCK_MONOTONIC through this, so a
   case steps past a deadline or a period (clock_skip) instead of sleeping
   through it. The test's own mono_now() reads the real clock. */
static long long clock_skew_ns;
static int skewed_clock_gettime(clockid_t id, struct timespec *ts)
{
    int r = clock_gettime(id, ts);
    long long ns = (long long)ts->tv_sec * 1000000000LL + ts->tv_nsec + clock_skew_ns;
    ts->tv_sec = (time_t)(ns / 1000000000LL);
    ts->tv_nsec = (long)(ns % 1000000000LL);
    return r;
}
static void clock_skip(double s) { clock_skew_ns += (long long)(s * 1e9); }

/* --- driver source under test ---------------------------------------- */
#define clock_gettime skewed_clock_gettime
#include "../src/glowforge_laser.c"
#undef clock_gettime
#include <sys/stat.h>              /* after the core headers: its macros clash with vfs.h */

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
sys_state_t state_get(void)
{
    return alarm_after_reads >= 0 && sw_calls > alarm_after_reads ? STATE_ALARM : cur_state;
}
static bool test_held(void) { return (state_get() & (STATE_HOLD | STATE_SAFETY_DOOR)) != 0; }
/* How many times a message has been reported. */
static int said(const char *needle)
{
    int n = 0;
    for (const char *p = all_messages; (p = strstr(p, needle)) != NULL; p += strlen(needle))
        n++;
    return n;
}

bool spindle_precompute_pwm_values(spindle_ptrs_t *s, spindle_pwm_t *p,
                                   spindle_pwm_settings_t *cfg, uint32_t hz)
{ (void)s; (void)p; (void)hz; precomputed_min = cfg->pwm_min_value; precompute_calls++; return true; }
/* The core's persisted-settings writers: the driver must never call them. */
void settings_write_global(void) { settings_stores++; }
spindle_id_t spindle_register(const spindle_ptrs_t *s, const char *n)
{ (void)s; (void)n; return 0; }
/* The planner: a block still planned, or nothing. */
static bool planner_busy;
static plan_block_t planned_block;
plan_block_t *plan_get_current_block(void) { return planner_busy ? &planned_block : NULL; }
/* gc_spindle_off() as the core has it (gcode.c): the modal spindle state
   cleared up to its hal pointer, and the spindle told off. */
static int spindle_off_calls;
void gc_spindle_off(void)
{
    spindle_off_calls++;
    memset(&gc_state.modal.spindle[0], 0, offsetof(spindle_t, hal));
    spindleSetState(NULL, (spindle_state_t){0}, 0.0f);
}
/* An M3/M4/M5 word as the core executes it (gcode.c, the spindle step):
   the spindle is asked only when the word changes the modal state, and
   the modal state then follows the word. */
static void m_word(spindle_state_t state, float rpm)
{
    if (gc_state.modal.spindle[0].state.value != state.value) {
        spindleSetState(NULL, state, rpm);
        gc_state.modal.spindle[0].state = state;
    }
}
/* The block the core planned last (plan_get_recent_block), or none. */
static plan_block_t recent_block;
static bool recent_valid;
plan_block_t *plan_get_recent_block(void) { return recent_valid ? &recent_block : NULL; }
/* A motion line as the core plans it (gcode.c, the spindle-speed step and
   the planner data; planner.c copies them into the block): an S word
   becomes the parser's modal S; the block takes the modal spindle state,
   and the modal S unless the motion is a rapid (in laser mode the core
   passes S0 for every motion that is not G1/G2/G3); a jog is flagged. A
   negative s_word is a line with no S word. Then the driver's input poll,
   as the stream runs it when the core's main loop reads on. */
enum { Line_Feed = 0, Line_Rapid, Line_Jog };
static void plan_line(int kind, float s_word)
{
    if (s_word >= 0.0f)
        gc_state.modal.spindle[0].rpm = s_word;
    memset(&recent_block, 0, sizeof(recent_block));
    recent_block.spindle.state = gc_state.modal.spindle[0].state;
    recent_block.spindle.rpm = kind == Line_Rapid ? 0.0f : gc_state.modal.spindle[0].rpm;
    recent_block.condition.rapid_motion = kind == Line_Rapid;
    recent_block.condition.jog_motion = kind == Line_Jog;
    recent_valid = true;
}
static void motion_line(int kind, float s_word)
{
    plan_line(kind, s_word);
    gflaser_input_poll();
}

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void led_put(const char *root, int led, const char *attr, int v)
{
    char path[300];
    snprintf(path, sizeof(path), "%s/button_led_%d/%s", root, led, attr);
    FILE *fp = fopen(path, "w");
    if (fp) {
        fprintf(fp, "%d\n", v);
        fclose(fp);
    }
}

static int led_val(const char *root, int led, const char *attr)
{
    char path[300], b[16];
    snprintf(path, sizeof(path), "%s/button_led_%d/%s", root, led, attr);
    FILE *fp = fopen(path, "r");
    if (!fp)
        return -1;
    int v = fgets(b, sizeof(b), fp) ? atoi(b) : -1;
    fclose(fp);
    return v;
}

/* All three LEDs read this pulse (on and off) and this target. */
static bool leds_read(const char *root, int pulse, int target)
{
    for (int i = 1; i <= 3; i++)
        if (led_val(root, i, "pulse_on") != pulse || led_val(root, i, "pulse_off") != pulse ||
            led_val(root, i, "target") != target)
            return false;
    return true;
}

/* A handler that registered for the end of a home before the module did. */
static int homed_calls;
static axes_signals_t homed_cycle;
static bool homed_success;
static void homed_before(axes_signals_t cycle, bool success)
{
    homed_calls++;
    homed_cycle = cycle;
    homed_success = success;
}

static void reset_state(void)
{
    lamp_at_press = -2;
    ack_waits_val = false;
    pause_val = false;
    fail_tier_val = false;
    fire_ok_calls = 0;
    ack_hold = 0;
    ack_calls = 0;
    ack_after_s = -1.0;
    conf_fan_grace = -1.0f;
    alarms_raised = 0;
    resets_requested = 0;
    cycle_starts = 0;
    suspend_model = false;
    suspend_entries = 0;
    suspend_depth = 0;
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
    tail_pumps = 0;
    pumps = 0;
    latch_locked_in_tail = false;
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
    modal_drop = false;
    dark_watch = false;
    recent_valid = false;
    planner_busy = false;
    spindle_off_calls = 0;
    memset(&gc_state.modal.spindle[0], 0, offsetof(spindle_t, hal));
    arming = false;
    atomic_store(&cur_state_value, 0);
    client_gen = 1;
    hw_active = false;              /* host: no GFSINK */
    sw_present = false;             /* no switch source: no button wait */
    sw_calls = 0;
    sw_n = 0;
    sw_fail_from = 1 << 30;
    serial_gen_bump_after = -1;
    gen_bump_after_ack_calls = -1;
    alarm_after_reads = -1;
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
    CHECK(alarms_raised == 1 && resets_requested == 1, "raises an alarm and resets the job on the late refusal");
    CHECK(fire_ok_calls == 2, "the coolant gate is re-checked after the wait");

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

    /* Case C - blocked at the pre-wait check: refuses before arming. */
    reset_state();
    script(false, false, 2);
    bool armed_c = gflaser_arm();
    CHECK(!armed_c, "refuses when the gate is blocked up front");
    CHECK(!laser_ok, "armed window stays closed on the early refusal");
    CHECK(fire_ok_calls == 1, "the pre-wait check short-circuits the arm");
    CHECK(alarms_raised == 1 && resets_requested == 1, "the early refusal alarms and resets the job");

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
    conf_fan_grace = 0.0f;              /* the budget is COOL_ACK_S alone */
    bool armed_a2 = gflaser_arm();
    CHECK(!armed_a2, "refuses when the engine never takes the armed window");
    CHECK(!laser_ok, "armed window stays closed without the acknowledgment");
    CHECK(!stream_armed, "stream is never told armed without the acknowledgment");
    CHECK(latch_locked_last, "latch is left locked without the acknowledgment");
    CHECK(alarms_raised == 1 && resets_requested == 1, "raises an alarm and resets when the engine does not take the job");

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

    /* Case A4 - the fan grace stretches the budget: the engine
       acknowledges once every gated fan reads its floor, and the exhaust
       takes seconds to get there. With cool_fan_grace_s 0.3 the budget
       is 0.5 s, so an acknowledgment at 0.35 s - past COOL_ACK_S alone -
       still arms, and the arm does not complete before it. An
       out-of-range grace falls back to the 15 s default. */
    reset_state();
    script(true, true, 2);
    conf_fan_grace = 0.3f;
    ack_after_s = 0.35;
    double t_a4 = mono_now();
    bool armed_a4 = gflaser_arm();
    double took_a4 = mono_now() - t_a4;
    CHECK(armed_a4 && laser_ok && stream_armed, "an acknowledgment inside the fan grace arms");
    CHECK(took_a4 >= 0.35 && took_a4 > COOL_ACK_S, "the arm waited past COOL_ACK_S for the acknowledgment");
    CHECK(alarms_raised == 0, "an acknowledgment inside the grace is not an alarm");
    conf_fan_grace = 500.0f;
    CHECK(ack_budget_s() == 15.0 + COOL_ACK_S, "an out-of-range fan grace falls back to 15 s");

    /* Case A5 - the window's owner is the sender that asked for the
       press. A sender that changes while the arm waits for the
       acknowledgment does not get the window: the job is canceled the way
       a change during the button wait cancels it. An arm whose sender
       stays is owned by that sender. */
    reset_state();
    script(true, true, 2);
    ack_hold = 3;
    gen_bump_after_ack_calls = 1;       /* the sender changes two polls into the wait */
    bool armed_a5 = gflaser_arm();
    CHECK(!armed_a5 && !laser_ok && !stream_armed && latch_locked_last,
          "a sender change during the wait for the acknowledgment opens no window");
    CHECK(alarms_raised == 1 && resets_requested == 1 && cool_armed_last == false &&
          strstr(all_messages, "sender changed during arm") != NULL,
          "the job is canceled: alarm, reset, the engine told, the sender change named");
    reset_state();
    script(true, true, 2);
    ack_hold = 3;
    client_gen = 7;
    CHECK(gflaser_arm() && laser_ok && armed_client_gen == 7, "the window is owned by the sender that asked for the press");

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
    CHECK(fire_ok_calls == 1, "the post-wait coolant check is not reached");

    /* Case F - a press with the lid open does not arm; the lid wins. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_LID_OPEN_PRESS, W_LID_OPEN_PRESS, 3);
    bool armed_f = gflaser_arm();
    CHECK(!armed_f, "press with the lid open does not arm");
    CHECK(!laser_ok && latch_locked_last, "lid-open press leaves the latch locked");

    /* Case G - the interlock loop opens during the wait: cancelled too. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_LOOP_OPEN, W_LOOP_OPEN, 3);
    bool armed_g = gflaser_arm();
    CHECK(!armed_g, "interlock open during the wait refuses");
    CHECK(!laser_ok && latch_locked_last, "interlock open leaves the latch locked");
    CHECK(resets_requested == 1 && alarms_raised == 0, "interlock open cancels with a soft reset, not an alarm");

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
    spindle_state_t off = { .on = Off };
    spindleSetState(NULL, off, 0.0f);           /* M5 inside the job */
    CHECK(laser_ok && stream_armed, "an M5 inside the job leaves the window open");
    spindleSetState(NULL, on, 300.0f);          /* and M3 again */
    CHECK(sw_calls == 0 && laser_ok && stream_armed,
          "an M5 then an M3 inside the open window does not re-prompt");

    printf("a window does not outlive its job, and the next job is prompted:\n");

    /* Case W1 - a job that ends with M4 still modal (no M5, no M2). At
       Idle with nothing planned the grace runs all the same, the window
       closes, and the parser's M4 goes with it: the next job's M4 is a
       state change again and asks for the press. The M-words go through
       the core's rule (m_word), which asks the spindle only on a change
       of the modal state. */
    spindle_state_t m4 = { .on = On, .ccw = On };
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    CHECK(laser_ok && gc_state.modal.spindle[0].state.on, "M4 arms through the press and is modal");
    cur_state = STATE_CYCLE;
    disarm_at = 1.0;
    gflaser_poll();
    CHECK(laser_ok && disarm_at == 0.0, "the job's cycle holds the window open");
    cur_state = STATE_IDLE;
    planner_busy = true;
    gflaser_poll();
    CHECK(laser_ok && disarm_at == 0.0, "Idle with blocks still planned is not the job's end");
    planner_busy = false;
    gflaser_poll();
    CHECK(laser_ok && disarm_at != 0.0, "at Idle with nothing planned the grace runs under a modal M4");
    disarm_at = 1.0;                    /* the grace has run out */
    gflaser_poll();
    CHECK(!laser_ok && !stream_armed && latch_locked_last, "the grace closes the window under a modal M4");
    CHECK(spindle_off_calls == 1 && !gc_state.modal.spindle[0].state.on &&
          !((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "the parser's M4 goes with the window: M5 in the parser, the spindle told off");
    CHECK(sw_calls == 3 && latch_writes == 2, "turning the modal state off asked for no arm");
    sw_calls = 0;
    fire_ok_calls = 0;
    m_word(m4, 1000.0f);
    CHECK(sw_calls > 0 && laser_ok && stream_armed, "the next job's M4 asks for the press again");
    /* A jog under a modal M4 does not hold the window open either. */
    cur_state = STATE_JOG;
    disarm_at = 1.0;
    gflaser_poll();
    CHECK(!laser_ok && gc_state.modal.spindle[0].state.on, "a jog under a modal M4 lets the grace close the window");
    cur_state = STATE_IDLE;
    gflaser_poll();
    CHECK(!gc_state.modal.spindle[0].state.on, "and the parser's M4 goes once the jog is over");

    /* Case W2 - a sender change at Idle with M4 modal: the window closes
       and the parser's M4 goes in the same poll, so the new sender's M4,
       the first line it sends, asks for the press. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    CHECK(laser_ok, "armed for the sender change");
    client_gen++;
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last && !gc_state.modal.spindle[0].state.on,
          "a sender change at Idle closes the window and takes the parser's M4 in one poll");
    CHECK(strstr(all_messages, "the next M3 or M4 asks for the button") != NULL, "and the sender is told");
    sw_calls = 0;
    fire_ok_calls = 0;
    m_word(m4, 1000.0f);
    CHECK(sw_calls > 0 && laser_ok, "the new sender's M4 asks for the press");

    /* Case W3 - a held job keeps its M4: the grace closes the window in
       the hold, the modal state stays (the resume gate re-arms a held job
       by it), and the press re-arms. The re-arm cancels the drop: the M4
       is still modal when the job is back at Idle under its new window. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    CHECK(laser_ok, "armed for the hold");
    cur_state = STATE_HOLD;
    atomic_store(&cur_state_value, 0);          /* the core turns the laser off in a hold */
    disarm_at = 1.0;
    gflaser_poll();
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "the grace closes the window under the held job");
    CHECK(gc_state.modal.spindle[0].state.on && spindle_off_calls == 0, "the held job keeps its M4");
    sw_calls = 0;
    fire_ok_calls = 0;
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 1);
    CHECK(gflaser_resume_gate() && rearm_pending, "the resume gate re-arms the held job by its modal state");
    gflaser_poll();                             /* the button is up */
    sw_calls = 0;
    switches(W_PRESSED, W_PRESSED, W_PRESSED, 1);
    gflaser_poll();
    CHECK(laser_ok && !rearm_pending, "the press re-arms the held job");
    cur_state = STATE_IDLE;
    gflaser_poll();
    CHECK(gc_state.modal.spindle[0].state.on && spindle_off_calls == 0,
          "the re-arm cancels the drop: the M4 is the open window's");

    /* Case W4 - a window that closes under an alarm keeps the parser's M4
       until the machine is back at Idle (the parser takes no line before
       that), and loses it there. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    cur_state = STATE_ALARM;
    gflaser_poll();
    CHECK(!laser_ok && gc_state.modal.spindle[0].state.on, "an alarm closes the window; the M4 waits for Idle");
    cur_state = STATE_IDLE;
    gflaser_poll();
    CHECK(!gc_state.modal.spindle[0].state.on && spindle_off_calls == 1, "back at Idle the parser's M4 goes");

    /* Case W5 - a parked job whose spindle the core left on keeps its
       window: the grace runs with the spindle on only at rest, and a hold
       is not rest. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the parked job");
    atomic_store(&cur_state_value, on.value);
    cur_state = STATE_HOLD;
    disarm_at = 1.0;                            /* a grace that has expired */
    gflaser_poll();
    CHECK(laser_ok && disarm_at == 0.0, "a spindle left on in a parked job keeps the window open");

    /* Case W6 - the drop waits while an arm waits for its press, the poll
       running inside the arm's pump. No path owes a drop with M3/M4 modal
       at that moment (a close pending at a laser-on follows M2, which
       leaves the parser at M5), so the state is set here directly. */
    reset_state();
    gc_state.modal.spindle[0].state = m4;
    modal_drop = true;
    arming = true;
    gflaser_poll();
    CHECK(modal_drop && spindle_off_calls == 0 && gc_state.modal.spindle[0].state.on,
          "a drop owed while an arm waits for its press waits too");
    arming = false;

    printf("a stalled job does not run on dark:\n");

    /* Case K1 - the job only stalled: its sender stopped for longer than
       the grace with M4 modal, the window closed, the parser's M4 and S
       went. Its next line is a feed move with an S word. The block is
       planned with the laser off and a programmed S above zero, and the
       job is stopped where the core has planned it and not yet started
       it, at the stream's next read: a reset, the alarm after it, one
       message, nothing armed or unlocked. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    gc_state.modal.spindle[0].rpm = 1000.0f;    /* the job's S */
    motion_line(Line_Feed, 1000.0f);
    CHECK(laser_ok && resets_requested == 0, "a lit feed move inside the window is the job cutting");
    cur_state = STATE_IDLE;                     /* the sender stops sending */
    recent_valid = false;
    gflaser_poll();
    disarm_at = 1.0;                            /* past the grace */
    gflaser_poll();
    CHECK(!laser_ok && !gc_state.modal.spindle[0].state.on && gc_state.modal.spindle[0].rpm == 0.0f,
          "the stall closes the window and takes the M4 and its S from the parser");
    {
        int writes = latch_writes;
        all_messages[0] = '\0';
        motion_line(Line_Feed, 500.0f);         /* the job carries on: G1 X.. S500 */
        CHECK(resets_requested == 1, "the first feed move with an S word stops the job at the stream's next read");
        CHECK(alarms_raised == 0, "the reset goes first: no alarm ahead of it");
        CHECK(said("the laser window closed while the job waited - job stopped; start it again") == 1,
              "the sender is told why, once");
        CHECK(!laser_ok && !stream_armed && !fire_gate_last && latch_locked_last && latch_writes == writes &&
              cycle_starts == 0, "nothing is armed, unlocked or started for it");
        gflaser_poll();                         /* the reset has landed at a standstill */
        CHECK(alarms_raised == 1, "the alarm follows the reset");
        motion_line(Line_Feed, 500.0f);
        gflaser_poll();
        CHECK(resets_requested == 1 && alarms_raised == 1 &&
              said("the laser window closed while the job waited") == 1, "the job is stopped once");
    }

    /* The same stall, for the cases below. */
#define STALLED_DROP() do { \
        reset_state(); \
        script(true, true, 2); \
        switches(W_CLOSED, W_CLOSED, W_PRESSED, 3); \
        m_word(m4, 1000.0f); \
        gc_state.modal.spindle[0].rpm = 1000.0f; \
        cur_state = STATE_IDLE; \
        gflaser_poll(); \
        disarm_at = 1.0; \
        gflaser_poll(); \
    } while (0)

    /* Case K2 - what is not a cutting move is left alone: a feed move with
       no S word (the drop left S at zero), one with S0, a rapid with an S
       word, a jog. The S the rapid's line programmed is the parser's from
       then on, so the feed move after it does carry one. */
    STALLED_DROP();
    CHECK(dark_watch && !laser_ok, "the drop under a job sets the watch");
    memset(&recent_block, 0, sizeof(recent_block));
    recent_block.spindle.state.on = On;         /* a block the job planned lit, before the stall */
    recent_block.spindle.rpm = 1000.0f;
    recent_valid = true;
    gflaser_input_poll();
    CHECK(resets_requested == 0 && dark_watch, "a block planned with the laser on is not a dark one");
    memset(&recent_block, 0, sizeof(recent_block));
    recent_block.spindle.rpm = 500.0f;          /* an S above zero, the laser off */
    recent_block.condition.rapid_motion = On;
    gflaser_input_poll();
    recent_block.condition.rapid_motion = Off;
    recent_block.condition.system_motion = On;  /* the same S, on a system motion */
    gflaser_input_poll();
    CHECK(resets_requested == 0 && dark_watch,
          "a rapid and a system motion are spared by their own flags, whatever S they carry");
    motion_line(Line_Feed, -1.0f);
    motion_line(Line_Feed, 0.0f);
    motion_line(Line_Rapid, 500.0f);
    motion_line(Line_Jog, -1.0f);
    gflaser_poll();
    CHECK(resets_requested == 0 && alarms_raised == 0 && dark_watch,
          "a feed move with no S, one with S0, a rapid and a jog move on with no alarm");
    motion_line(Line_Feed, -1.0f);
    CHECK(resets_requested == 1, "a feed move under an S programmed since the drop stops the job");

    /* Case K3 - a new job speaks for itself: its M4 is a state change, it
       is prompted and armed, never alarmed, and the watch is over. */
    STALLED_DROP();
    sw_calls = 0;
    fire_ok_calls = 0;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    CHECK(sw_calls > 0 && laser_ok && !dark_watch, "a new M4 after the drop is prompted and armed, and ends the watch");
    motion_line(Line_Feed, 500.0f);
    gflaser_poll();
    CHECK(resets_requested == 0 && alarms_raised == 0 && laser_ok, "and its job cuts with no alarm");

    /* Case K4 - the other ends of the watch: a program end, a disarm (the
       reset's, a fault's, a home's), an alarm. A feed move with an S word
       after any of them is not the stalled job's. */
    for (int end = 0; end < 3; end++) {
        static const char *const what[] = {
            "a program end ends the watch",
            "a disarm (a reset, a fault, a home) ends the watch",
            "an alarm ends the watch",
        };
        STALLED_DROP();
        if (end == 0)
            onProgramCompleted(ProgramFlow_CompletedM2, false);
        else if (end == 1)
            gflaser_disarm();
        else {
            cur_state = STATE_ALARM;
            gflaser_poll();
            cur_state = STATE_IDLE;
        }
        motion_line(Line_Feed, 500.0f);
        gflaser_poll();
        CHECK(!dark_watch && resets_requested == 0 && alarms_raised == 0, what[end]);
    }

    /* Case K5 - the poll looks too (a line that plans more than one block
       is caught at its second), and from a running cycle the reset is all
       that is asked: the core's reset from motion raises ALARM:3 itself. */
    STALLED_DROP();
    cur_state = STATE_CYCLE;                    /* the job's dry moves are playing */
    plan_line(Line_Feed, 500.0f);
    gflaser_poll();
    CHECK(resets_requested == 1 && alarms_raised == 0, "the poll stops a cutting move planned behind a running cycle");
    cur_state = STATE_ALARM;                    /* the reset from motion, with its own alarm */
    gflaser_poll();
    cur_state = STATE_IDLE;
    gflaser_poll();
    CHECK(alarms_raised == 0, "and no second alarm is raised over the reset's own");

    /* Case K6 - a held job is not a stalled one: the grace closes its
       window in the hold, the drop stays owed, and no watch is set. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    m_word(m4, 1000.0f);
    gc_state.modal.spindle[0].rpm = 1000.0f;
    cur_state = STATE_HOLD;
    atomic_store(&cur_state_value, 0);
    disarm_at = 1.0;
    gflaser_poll();
    gflaser_poll();
    gflaser_input_poll();
    CHECK(!laser_ok && gc_state.modal.spindle[0].state.on && !dark_watch && resets_requested == 0,
          "a held job past the grace keeps its M4 and sets no watch");

    /* Case K7 - the limit of the watch: an M5 that changes nothing never
       reaches the driver (the core tells the spindle only of a change of
       state), so it does not end the watch. */
    STALLED_DROP();
    m_word((spindle_state_t){0}, 0.0f);         /* M5 under a parser already at M5 */
    CHECK(dark_watch, "an M5 that changes nothing is not seen: the watch stands");
#undef STALLED_DROP

    printf("the dose model: derived floors:\n");

    /* Case J - the density floor key: 12.5 is honored, an out-of-range
       key falls back to the default of 10, and 0 is honored. A stale $35
       in RAM (a sender wrote 3) is overwritten, never honored. */
    reset_state();
    script(true, true, 2);
    conf_floor_density = 12.5f;
    settings.pwm_spindle.pwm_min_value = 3.0f;
    CHECK(gflaser_arm() && settings.pwm_spindle.pwm_min_value == 12.5f && precomputed_min == 12.5f,
          "laser_floor_density overrides the density default, overwriting a typed $35");
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
    CHECK(alarms_raised == 1 && resets_requested == 1, "the refusal is an alarm and a reset");
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
    CHECK(alarms_raised == 1 && resets_requested == 1, "the late head refusal is an alarm and a reset");

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
    CHECK(alarms_raised == 1 && resets_requested == 1, "the unreadable wait is an alarm and a reset");

    /* Case S19 - the wait's other ends cancel the job the same way: the
       button timeout and a sender change alarm and reset, and an alarm
       raised under the wait (a stream fault) resets. The reset drops the
       job's M3/M4 from the parser: with the alarm alone, after $X the
       same M3/M4 changed nothing, the arm never ran, and the job moved
       dark with no prompt (found on the bench). */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    conf_button_timeout = 1.0f;
    CHECK(!gflaser_arm() && !laser_ok && latch_locked_last, "the button timeout refuses the arm");
    CHECK(alarms_raised == 1 && resets_requested == 1, "the timeout is an alarm and a reset");
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    serial_gen_bump_after = 1;
    CHECK(!gflaser_arm() && !laser_ok && latch_locked_last, "a sender change during the wait refuses the arm");
    CHECK(alarms_raised == 1 && resets_requested == 1, "the sender change is an alarm and a reset");
    serial_gen_bump_after = -1;
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    alarm_after_reads = 1;
    CHECK(!gflaser_arm() && !laser_ok && latch_locked_last, "an alarm under the wait refuses the arm");
    CHECK(alarms_raised == 0 && resets_requested == 1, "the standing alarm is not raised again, and the job is reset");
    alarm_after_reads = -1;

    printf("the latch writes:\n");

    /* Case S6 - an unlock that does not take refuses the arm. */
    reset_state();
    script(true, true, 2);
    latch_unlock_fail = true;
    CHECK(!gflaser_arm(), "a latch that does not unlock refuses the arm");
    CHECK(!laser_ok && !stream_armed, "no window opens on a failed unlock");
    CHECK(latch_locked_last, "the ownership record still says locked after the failed unlock");
    CHECK(alarms_raised == 1 && resets_requested == 1, "the failed unlock is an alarm and a reset");
    CHECK(cool_armed_last == false, "the engine is told the window is not armed after the failed unlock");

    /* Case S5 - the disarm locks the latch whenever this process
       unlocked it, window or no window (a reset in the button wait); a
       latch already locked and no window is left alone. */
    reset_state();
    latch_locked_last = false;      /* the arm wait's unlock, no window yet */
    gflaser_disarm();
    CHECK(latch_locked_last && latch_writes == 1, "a disarm with no window relocks a latch this process unlocked");
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
    clock_skip(0.08);
    gflaser_poll();
    CHECK(!laser_ok && latch_locked_last, "an unreadable kernel state relocks after the bound");

    printf("a laser-on inside the tail of the job before it:\n");

    /* Case S4b - program end only asks for the close, which waits for the
       kernel to play the job's tail. A laser-on that arrives inside the
       tail is the next job's: the close is finished first (the tail
       waited for, the latch locked after it), and the laser-on then asks
       for its own press. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the first program");
    hw_active = true;
    onProgramCompleted(ProgramFlow_CompletedM2, false);
    CHECK(laser_ok && disarm_request, "program end asks for the close, and the window waits for the tail");
    rd_state = "running";
    tail_pumps = 3;
    sw_calls = 0;
    fire_ok_calls = 0;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    all_messages[0] = '\0';
    spindleSetState(NULL, on, 1000.0f);
    {
        const char *closed = strstr(all_messages, "laser disarmed - latch locked");
        const char *prompt = strstr(all_messages, "press the button to start the laser job");
        CHECK(closed != NULL, "the laser-on inside the tail closes the first program's window");
        CHECK(tail_pumps == 0 && !latch_locked_in_tail, "the close waited for the kernel's tail: no lock while it played");
        CHECK(prompt != NULL && closed != NULL && closed < prompt && sw_calls > 0,
              "and the laser-on asks for a new press, after the close");
        CHECK(laser_ok && stream_armed && !disarm_request && !latch_locked_last,
              "the press opens the second program's own window");
    }
    /* A kernel that never reports idle does not keep the old window: the
       wait is bounded, the window closes, the laser-on prompts. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed again");
    hw_active = true;
    onProgramCompleted(ProgramFlow_CompletedM2, false);
    rd_state = "running";
    sw_calls = 0;
    fire_ok_calls = 0;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    all_messages[0] = '\0';
    {
        double t0 = mono_now();
        spindleSetState(NULL, on, 1000.0f);
        CHECK(mono_now() - t0 >= RELOCK_WAIT_S && strstr(all_messages, "laser disarmed - latch locked") != NULL &&
              sw_calls > 0, "a tail that never ends is waited for no longer than the bound, then closed and prompted");
    }
    /* A reset inside the wait closes the window and arms nothing. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed again");
    hw_active = true;
    onProgramCompleted(ProgramFlow_CompletedM2, false);
    rd_state = "running";
    resets_requested = 1;
    sw_calls = 0;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    spindleSetState(NULL, on, 1000.0f);
    CHECK(!laser_ok && latch_locked_last && sw_calls == 0 &&
          !((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "a reset inside the wait closes the window, arms nothing and leaves the spindle off");
    /* In a held state the wait cannot run: the window closes at once and
       the laser stays dark. */
    reset_state();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed again");
    hw_active = true;
    onProgramCompleted(ProgramFlow_CompletedM2, false);
    rd_state = "running";
    cur_state = STATE_HOLD;
    spindleSetState(NULL, on, 1000.0f);
    CHECK(!laser_ok && latch_locked_last && pumps == 0 &&
          !((spindle_state_t){ .value = atomic_load(&cur_state_value) }).on,
          "a laser-on in a held state closes the pending window with no wait and stays dark");

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
    CHECK(!rearm_pending && latch_locked_last && alarms_raised == 1 && resets_requested == 1,
          "the third unreadable read cancels the re-arm, relocks, alarms and resets");
    gc_state.modal.spindle[0].state.on = Off;

    /* Case S18 - a fault waiting for its Acknowledge: the resume is refused
       and the job stays held, no alarm, no unlock; a re-arm already waiting
       when the fault comes and goes ends the same way at the press. */
    reset_state();
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    ack_waits_val = true;               /* the engine's ACK: fire blocked */
    script(false, false, 1);
    fire_ok_calls = 0;
    CHECK(gflaser_resume_gate() && !rearm_pending, "under ACK the resume gate takes the resume and starts no re-arm");
    CHECK(alarms_raised == 0 && resets_requested == 0 && latch_writes == 0 && latch_locked_last,
          "no alarm, no reset, the latch never unlocked: the job stays held");
    CHECK(strstr(last_message, "Acknowledge") && strstr(last_message, "stays held"), "the refusal points at the panel");
    ack_waits_val = false;
    script(true, true, 1);
    fire_ok_calls = 0;
    CHECK(gflaser_resume_gate() && rearm_pending, "after the Acknowledge the resume re-arms");
    gflaser_poll();                     /* the button is up: the next press counts */
    ack_waits_val = true;               /* a fault came and went during the wait */
    script(false, false, 1);
    fire_ok_calls = 0;
    sw_calls = 0;
    switches(W_PRESSED, W_PRESSED, W_PRESSED, 1);   /* the next read is the press */
    gflaser_poll();
    CHECK(!rearm_pending && !laser_ok && latch_locked_last && alarms_raised == 0 && resets_requested == 0,
          "a press under ACK ends the re-arm, the latch relocked, the job held, no alarm");
    ack_waits_val = false;
    gc_state.modal.spindle[0].state.on = Off;

    /* Case S18b - a standing pause-tier verdict (OVERTEMP, say) under a
       held job whose window has closed: the press or the ~ is refused
       with the gate's words and the job stays held - no alarm, no reset,
       no unlock, the engine never told armed - and the same resume re-arms
       once the verdict has cleared. A verdict that comes back while the
       re-arm waits ends it the same way at the press. The fail tier and a
       stale verdict still end the job, the fail tier under a pending
       Acknowledge included. */
    reset_state();
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    pause_val = true;
    script(false, false, 1);
    CHECK(gflaser_resume_gate() && !rearm_pending,
          "under a standing pause verdict the resume gate takes the resume and starts no re-arm");
    CHECK(alarms_raised == 0 && resets_requested == 0 && latch_writes == 0 && latch_locked_last &&
          cool_armed_calls == 0, "no alarm, no reset, no unlock, nothing reported armed: the job stays held");
    CHECK(strstr(last_message, "a cooling gate is closed") && strstr(last_message, "stays held") &&
          !strstr(last_message, "Acknowledge"), "the refusal is the gate's own, and says the job stays held");
    pause_val = false;
    script(true, true, 1);
    fire_ok_calls = 0;
    CHECK(gflaser_resume_gate() && rearm_pending, "once the verdict has cleared the same resume re-arms");
    gflaser_poll();                     /* the button is up: the next press counts */
    pause_val = true;                   /* the verdict comes back during the wait */
    script(false, false, 1);
    fire_ok_calls = 0;
    sw_calls = 0;
    switches(W_PRESSED, W_PRESSED, W_PRESSED, 1);
    gflaser_poll();
    CHECK(!rearm_pending && !laser_ok && latch_locked_last && alarms_raised == 0 && resets_requested == 0 &&
          strstr(last_message, "stays held"),
          "a press under a verdict that came back ends the re-arm: relocked, held, no alarm");
    pause_val = false;
    fail_tier_val = true;               /* the fail tier, or a stale verdict: no standing hold */
    ack_waits_val = true;               /* even with an Acknowledge pending under it */
    script(false, false, 1);
    fire_ok_calls = 0;
    CHECK(gflaser_resume_gate() && !rearm_pending && alarms_raised == 1 && resets_requested == 1 &&
          latch_locked_last, "the fail tier and a stale verdict still end the job: alarm and reset");
    ack_waits_val = false;
    gc_state.modal.spindle[0].state.on = Off;

    printf("the re-arm waits for the acknowledgment from the poll, never from a pump:\n");

    /* Case S21 - the press is recorded on the poll that reads it, and the
       window opens on a later poll, once the engine has acknowledged the
       job. The poll runs inside the core's suspend loop, so nothing on
       this path may pump the protocol: the stub models that loop
       (suspend_model), and an entry into it is the defect. Here the press
       is a short one, up again before the acknowledgment, and the
       acknowledgment comes after the button's deadline: that deadline is
       the wait for the press, and stops counting once the press is in. */
    reset_state();
    suspend_model = true;
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    conf_button_timeout = 1.0f;         /* inside the acknowledgment's budget (15 s of grace) */
    CHECK(gflaser_resume_gate() && rearm_pending && lamp_now == Lamp_Press, "the resume gate starts the re-arm");
    gflaser_poll();                     /* the button is up: the next press counts */
    ack_hold = 3;                       /* the engine takes the job three polls later */
    ack_calls = 0;
    all_messages[0] = '\0';
    sw_calls = 0;
    switches(W_PRESSED, W_CLOSED, W_CLOSED, 2);
    gflaser_poll();                     /* the press */
    CHECK(!laser_ok && rearm_pending && gflaser_arming(), "a press before the acknowledgment is taken, and the window waits");
    CHECK(lamp_now == Lamp_Off, "the button stops breathing the moment the press is taken");
    clock_skip(1.1);                    /* past the button's deadline */
    gflaser_poll();
    CHECK(rearm_pending, "past the button's deadline the re-arm still waits: the press is in");
    gflaser_poll();
    CHECK(!laser_ok && cycle_starts == 0, "no window and no resume before the acknowledgment");
    gflaser_poll();                     /* the engine has the job */
    CHECK(laser_ok && stream_armed && !rearm_pending && !latch_locked_last,
          "the window opens on the poll that finds the acknowledgment");
    CHECK(said("laser armed") == 1, "the job is armed exactly once");
    CHECK(cycle_starts == 1, "and resumed by exactly one cycle start");
    CHECK(suspend_entries == 0, "the re-arm never pumped the protocol from the held state");
    cur_state = STATE_CYCLE;
    gflaser_poll();
    gflaser_poll();
    CHECK(laser_ok && alarms_raised == 0 && resets_requested == 0 && cycle_starts == 1 && said("laser armed") == 1,
          "nothing refuses or re-arms the resumed job afterward");
    gc_state.modal.spindle[0].state.on = Off;

    /* The button held down through the acknowledgment: one arm, one
       cycle start, no pump. */
    reset_state();
    suspend_model = true;
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    CHECK(gflaser_resume_gate() && rearm_pending, "the resume gate starts the re-arm again");
    gflaser_poll();
    ack_hold = 3;
    ack_calls = 0;
    all_messages[0] = '\0';
    sw_calls = 0;
    switches(W_PRESSED, W_PRESSED, W_PRESSED, 1);
    for (int i = 0; i < 6; i++)
        gflaser_poll();
    CHECK(laser_ok && said("laser armed") == 1 && cycle_starts == 1 && suspend_entries == 0,
          "a button held through the acknowledgment arms once, resumes once, pumps nothing");
    gc_state.modal.spindle[0].state.on = Off;

    /* Case S22 - the other ends of the wait for the acknowledgment, each
       with the latch locked and no window: the budget running out refuses
       the job as it does at the first arm, with the button held down
       through it (one press: the budget runs from it); a fault that came
       and went keeps the job held; the lid, a sender change and a reset
       end the re-arm; and an acknowledgment that arrives afterward opens
       nothing. */
    reset_state();
    suspend_model = true;
    script(true, true, 2);
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    conf_fan_grace = 0.0f;              /* the budget is COOL_ACK_S alone */
    CHECK(gflaser_resume_gate() && rearm_pending, "re-arm for the budget");
    gflaser_poll();
    ack_hold = 1 << 30;                 /* the engine never takes the job */
    sw_calls = 0;
    switches(W_PRESSED, W_PRESSED, W_PRESSED, 1);   /* pressed, and held */
    gflaser_poll();
    CHECK(rearm_pending && alarms_raised == 0, "inside the budget the re-arm waits");
    clock_skip(COOL_ACK_S + 0.05);
    gflaser_poll();
    CHECK(!rearm_pending && !laser_ok && latch_locked_last && cool_armed_last == false,
          "past the budget the re-arm ends with the latch locked and the engine told");
    CHECK(alarms_raised == 1 && resets_requested == 1 && cycle_starts == 0 &&
          strstr(all_messages, "did not take the job") != NULL && suspend_entries == 0,
          "and the job is refused as at the first arm: alarm, reset, no resume");
    gc_state.modal.spindle[0].state.on = Off;

    /* A window refused at the acknowledgment starts nothing: the verdict
       has gone stale by then (no standing hold, fire blocked), the window's
       gate refuses the job as the first arm's does, and no cycle start
       follows the refusal. */
    reset_state();
    suspend_model = true;
    script(true, false, 2);             /* clear at the gate, blocked at the window */
    switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    CHECK(gflaser_resume_gate() && rearm_pending, "re-arm for the refused window");
    gflaser_poll();
    sw_calls = 0;
    switches(W_PRESSED, W_CLOSED, W_CLOSED, 2);
    gflaser_poll();                     /* the press; the engine has the job */
    CHECK(!laser_ok && !rearm_pending && latch_locked_last && alarms_raised == 1 && resets_requested == 1 &&
          cycle_starts == 0, "a window refused at the acknowledgment ends the job with no cycle start");
    gc_state.modal.spindle[0].state.on = Off;

    for (int end = 0; end < 4; end++) {
        static const char *const what[] = {
            "a fault that came and went during the wait keeps the job held: relocked, no alarm",
            "a lid opened during the wait ends the re-arm: relocked, no window",
            "a sender change during the wait ends the re-arm: relocked, the job held",
            "a reset during the wait ends the re-arm: relocked, no window",
        };
        reset_state();
        suspend_model = true;
        script(true, true, 2);
        switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
        cur_state = STATE_HOLD;
        gc_state.modal.spindle[0].state.on = On;
        gflaser_resume_gate();
        gflaser_poll();
        ack_hold = 2;
        ack_calls = 0;
        sw_calls = 0;
        switches(W_PRESSED, W_CLOSED, W_CLOSED, 2);
        gflaser_poll();                 /* the press; the acknowledgment is two polls away */
        if (end == 0)
            ack_waits_val = true;
        else if (end == 1) {
            sw_calls = 0;
            switches(W_LID_OPEN, W_LID_OPEN, W_LID_OPEN, 1);
        } else if (end == 2)
            client_gen++;
        else
            cur_state = STATE_ALARM;
        gflaser_poll();
        bool ended = !rearm_pending && !gflaser_arming() && !laser_ok && latch_locked_last;
        ack_waits_val = false;
        sw_calls = 0;
        switches(W_CLOSED, W_CLOSED, W_CLOSED, 1);
        if (end == 3)
            cur_state = STATE_HOLD;
        gflaser_poll();
        gflaser_poll();
        gflaser_poll();                 /* the acknowledgment arrives: nothing waits for it */
        CHECK(ended && !laser_ok && latch_locked_last && cycle_starts == 0 && alarms_raised == 0 &&
              resets_requested == 0 && suspend_entries == 0, what[end]);
        gc_state.modal.spindle[0].state.on = Off;
    }

    /* Case S23 - no button (the host build): the resume is the consent,
       and the same poll path opens the window, at once when the engine has
       the job already and on a later poll when it has not. */
    reset_state();
    suspend_model = true;
    script(true, true, 2);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    CHECK(gflaser_resume_gate() && laser_ok && cycle_starts == 1 && !rearm_pending,
          "with no button and the job acknowledged, the gate opens the window and issues the cycle start");
    reset_state();
    suspend_model = true;
    script(true, true, 2);
    cur_state = STATE_HOLD;
    gc_state.modal.spindle[0].state.on = On;
    ack_hold = 2;
    CHECK(gflaser_resume_gate() && !laser_ok && rearm_pending, "with no button and no acknowledgment yet, the gate waits");
    gflaser_poll();
    gflaser_poll();
    CHECK(laser_ok && cycle_starts == 1 && said("laser armed") == 1 && suspend_entries == 0,
          "and the poll opens the window when the acknowledgment arrives, with no pump");
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

    printf("a home closes the window:\n");

    /* Case S20 - a completed home closes the window whichever provider
       ran it. The manual provider never enters the homing state, so the
       poll's own close does not see it; the close hangs on the event every
       provider ends on, which the module registers for. A handler that
       registered before the module still hears of the home. */
    reset_state();
    grbl.on_homing_completed = homed_before;
    gflaser_init();
    script(true, true, 2);
    CHECK(gflaser_arm() && laser_ok, "armed for the home");
    gflaser_poll();
    CHECK(laser_ok, "at Idle the poll sees no home");
    CHECK(grbl.on_homing_completed != NULL, "the module registers for the end of a home");
    if (grbl.on_homing_completed)
        grbl.on_homing_completed((axes_signals_t){ .mask = X_AXIS_BIT | Y_AXIS_BIT }, true);
    CHECK(!laser_ok && !stream_armed && latch_locked_last, "a completed home closes the window and locks the latch");
    CHECK(homed_calls == 1 && homed_cycle.mask == (X_AXIS_BIT | Y_AXIS_BIT) && homed_success,
          "the end of the home is passed on to the handler registered before the module");

    printf("the button lamp, and the refusal that names the panel:\n");

    /* The press: breathing white while the arm waits for it, dark after. */
    reset_state();
    script(true, true, 2);
    sw_present = true;
    switches(W_CLOSED, W_CLOSED, W_PRESSED, 3);
    CHECK(gflaser_arm() && laser_ok, "the press arms");
    CHECK(lamp_at_press == Lamp_Press, "the button breathes white while it waits for the press");
    CHECK(lamp_now == Lamp_Off, "and stops at the press");
    gflaser_disarm();

    /* The lamp follows the machine while nothing waits for a press. */
    reset_state();
    cur_state = STATE_CYCLE;
    gflaser_poll();
    CHECK(lamp_now == Lamp_Run, "the button is solid white while the machine moves");
    cur_state = STATE_JOG;
    gflaser_poll();
    CHECK(lamp_now == Lamp_Run, "a jog moves the machine too");
    cur_state = STATE_HOLD;
    gflaser_poll();
    CHECK(lamp_now == Lamp_Off, "a held machine is not moving");
    cur_state = STATE_IDLE;
    gflaser_poll();
    CHECK(lamp_now == Lamp_Off, "and neither is an idle one");

    /* An unacknowledged fault: the same refusal, with words that point at the panel. */
    reset_state();
    script(false, false, 2);
    ack_waits_val = true;
    CHECK(!gflaser_arm() && alarms_raised == 1 && resets_requested == 1 && latch_writes == 0,
          "an unacknowledged fault refuses the job and resets it");
    CHECK(strstr(all_messages, "a fault waits for its Acknowledge on the control panel") != NULL,
          "and points at the panel");
    reset_state();
    script(false, false, 2);
    CHECK(!gflaser_arm() && strstr(all_messages, "a cooling gate is closed") != NULL,
          "any other closed gate keeps its own words");

    /* The lamp put back after something wrote over it, never over the red. */
    {
        char root[] = "/tmp/laser-arm-leds-XXXXXX";
        if (mkdtemp(root)) {
            setenv("GF_LED_ROOT", root, 1);
            char d[256], f[300];
            for (int i = 1; i <= 3; i++) {
                snprintf(d, sizeof(d), "%s/button_led_%d", root, i);
                mkdir(d, 0755);
                /* sysfs attributes exist; a write never creates one */
                led_put(root, i, "pulse_on", 0);
                led_put(root, i, "pulse_off", 0);
                led_put(root, i, "speed", 0);
                led_put(root, i, "target", 0);
            }
            reset_state();
            lamp_now = -1;
            cur_state = STATE_CYCLE;
            gflaser_poll();
            CHECK(lamp_now == Lamp_Run && led_val(root, 1, "target") == 255, "the lamp is written solid white");
            /* the engine's red flash over it: left alone */
            led_put(root, 1, "pulse_on", 1000); led_put(root, 1, "target", 255);
            for (int i = 2; i <= 3; i++) { led_put(root, i, "target", 0); led_put(root, i, "pulse_on", 0); }
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(led_val(root, 1, "pulse_on") == 1000 && led_val(root, 2, "target") == 0,
                  "the red flash is not written over");
            /* the red released: the button dark, the machine still moving */
            for (int i = 1; i <= 3; i++) { led_put(root, i, "target", 0); led_put(root, i, "pulse_on", 0); }
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(led_val(root, 1, "target") == 255 && led_val(root, 2, "target") == 255 &&
                  led_val(root, 3, "target") == 255, "the lamp lost under it is written again");
            /* The LED trigger reports a target of 255 as 254 once the fade
               has settled: that is solid white, and nothing is written (a
               write would put the speed back to 64). */
            for (int i = 1; i <= 3; i++) { led_put(root, i, "target", 254); led_put(root, i, "speed", 7); }
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(led_val(root, 1, "speed") == 7 && led_val(root, 2, "speed") == 7 &&
                  led_val(root, 3, "speed") == 7 && led_val(root, 1, "target") == 254,
                  "a solid white that has settled (254) is not written again");
            led_put(root, 2, "target", 253);
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(led_val(root, 2, "target") == 255 && led_val(root, 2, "speed") == 64,
                  "a lamp short of white is written again");
            /* A change of the machine's state under the red flash: the lamp
               is recorded, not written over the red, and written once the
               red has ended. */
            cur_state = STATE_HOLD;
            gflaser_poll();
            CHECK(lamp_now == Lamp_Off && led_val(root, 1, "target") == 0, "a held machine writes the lamp dark");
            led_put(root, 1, "pulse_on", 1000); led_put(root, 1, "target", 255);
            cur_state = STATE_CYCLE;
            gflaser_poll();
            CHECK(lamp_now == Lamp_Run, "the machine moves: the lamp wanted is solid white");
            CHECK(led_val(root, 1, "pulse_on") == 1000 && led_val(root, 2, "target") == 0 &&
                  led_val(root, 3, "target") == 0, "the change of state is not written over the red flash");
            for (int i = 1; i <= 3; i++) { led_put(root, i, "target", 0); led_put(root, i, "pulse_on", 0); }
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(led_val(root, 1, "target") == 255 && led_val(root, 2, "target") == 255 &&
                  led_val(root, 3, "target") == 255, "and it is written once the red has ended");
            cur_state = STATE_HOMING;
            gflaser_poll();
            CHECK(leds_read(root, 0, 255), "a home moves the machine: the LEDs are solid white");
            /* A dark lamp that something else lit is put back dark. */
            cur_state = STATE_IDLE;
            gflaser_poll();
            led_put(root, 2, "target", 255);
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(leds_read(root, 0, 0), "a dark lamp lit by something else is written dark again");
            /* The re-arm's press wait: all three LEDs breathe white, the held
               machine's own lamp (dark) does not overwrite them, a breath
               written over is put back, and the press leaves them dark with
               the pulse stopped. */
            reset_state();
            script(true, true, 2);
            switches(W_CLOSED, W_CLOSED, W_CLOSED, 3);
            cur_state = STATE_HOLD;
            gc_state.modal.spindle[0].state.on = On;
            CHECK(gflaser_resume_gate() && rearm_pending && leds_read(root, 1800, 200) &&
                  led_val(root, 1, "speed") == 12 && led_val(root, 2, "speed") == 12 &&
                  led_val(root, 3, "speed") == 12, "the press wait breathes white on all three LEDs");
            gflaser_poll();                     /* the button is up */
            CHECK(leds_read(root, 1800, 200), "the held machine's lamp does not write over the press wait");
            for (int i = 1; i <= 3; i++) { led_put(root, i, "pulse_on", 0); led_put(root, i, "target", 0); }
            clock_skip(LAMP_KEEP_S);
            gflaser_poll();
            CHECK(leds_read(root, 1800, 200), "a press wait written over is put back");
            sw_calls = 0;
            switches(W_PRESSED, W_CLOSED, W_CLOSED, 2);
            gflaser_poll();                     /* the press; the engine has the job */
            CHECK(laser_ok && leds_read(root, 0, 0), "the press leaves the LEDs dark, the pulse stopped");
            gflaser_disarm();
            gc_state.modal.spindle[0].state.on = Off;
            cur_state = STATE_IDLE;
            gflaser_poll();
            for (int i = 1; i <= 3; i++) {
                snprintf(f, sizeof(f), "rm -rf %s/button_led_%d", root, i);
                if (system(f) != 0) { /* scratch */ }
            }
            rmdir(root);
            unsetenv("GF_LED_ROOT");
        }
    }

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: the arm re-checks the coolant gate after the wait, "
                      "the button wait honors the lid and the interlock, and "
                      "every laser-on against a closed window prompts, and the "
                      "floor is derived, never typed\n",
           failures);
    return failures ? 1 : 0;
}
