/*
  glowforge_laser.c - laser spindle + operator arming for the Glowforge board

  Part of grblHAL-glowforge. Maps grblHAL's laser spindle (M3/M4 + S
  words, $32 laser mode) onto the pulse stream: S values become raw
  7-bit power bytes (the kernel writes them straight into PWMSAR against
  a 127-count period, so the spindle PWM is precomputed to a period of
  exactly 127), and fire rides the per-tick FIRE bit. Power lands via
  the core's per-segment spindle update, which the stepper producer runs
  at exact virtual-tick positions - so the laser only ever fires inside
  motion segments of laser blocks. G0 and homing are fire-free by
  construction; a jog carries the modal spindle in Grbl, so the stream
  masks fire for as long as the core is jogging (gf_stream_jog). The
  kernel's end-of-data backstop forces the lines low whenever a stream
  ends.

  ARMING - the operator stays in the loop. FIRE reaches the tube only
  through the hardware AND chain (lid switches, interlock, HV OK, the
  charge-pump watchdog the kernel feeds while a run plays, and the
  BUTTON latch). On the first laser-on of a job this module:

    1. refuses outright while a coolant fire gate is active (the run
       ceiling, for one - glowforge_cooling.c owns the verdicts),
    2. forces the run fan profile on (so the cut airflow covers every
       fire window) and unlocks the kernel laser latch,
    3. lights the button white and BLOCKS the gcode stream - pumping
       real-time traffic like the homing session does - until the
       operator presses the physical button (EV_SW bit 2 on the input
       device), a soft reset aborts, the lid or the interlock loop
       opens (the job is canceled: relock, alarm), or the timeout
       expires. A press with the lid open does not arm - the hardware
       button latch would not clear on it either.

  The armed window is JOB-based: it persists across the job (S changes
  and M5/M3 toggles do not re-prompt) and closes - relocking the
  latch - at program end (M2/M30), whenever the sender's connection
  changes (the consent belonged to the displaced session), after
  laser_disarm_s of grace (counting down with the spindle off in Idle,
  in a jog, and in a job parked in Hold, Door or Tool Change, and with
  M3/M4 still modal whenever the machine is at rest, which is Idle with
  nothing planned or a jog - a window does not outlive its job, and a
  jog is not the job the press consented to; only a cycle, or a spindle
  left on in a parked job, keeps the window open), on the cooling
  verdict's fail tier (FIRE, CRASH, AIRFLOW, CRITICAL: the latch locks,
  the job is reset with ALARM:3 and nothing resumes it), or immediately
  on alarm, homing, reset or a stream fault. A window that closes with
  M3/M4 still modal and no job parked takes the parser's M3/M4 with it
  once the machine is at Idle, so the next M3/M4 is a state change and
  asks for the press (modal_poll). A job that had only stalled does not
  then run on dark: its first cutting move, a feed move programmed with
  S above zero and planned with the laser off, ends it with ALARM:3 and
  a reset (dark_watch_poll).
  The verdict's pause tier (every other fire_ok=false,
  and a stale verdict) holds the job under the open window, lit through
  that first deceleration and masked from the stop on, and the clean
  verdict's resume_ok resumes it with no new press. The pause never
  writes the latch: a lock sets the hardware button latch, which only a
  physical press clears, so a lock is a new press by construction and
  belongs to the fail tier alone. The coolant fire gate is re-checked
  after the button wait, so a window can never open against a verdict
  that went bad during the wait. While unarmed or gated, fire requests
  are suppressed at the stream: the stream engine masks the FIRE bit on
  every tick with the gate this module publishes (the window, and the
  verdict past the pause's deceleration), so no fire in flight survives
  a closed window beyond the bytes already in the kernel's queue.

  The latch has an owner: every lock and unlock this module makes goes
  through the stream engine's one writer, which records it, and a lock
  this process did not write (the engine's) is never undone by a run
  start. A disarm locks the latch whenever this process unlocked it,
  whether or not the window ever opened (the button wait, the re-arm
  wait), and a state that cannot be read is never "still playing": the
  relock after a job waits for the kernel to idle, and a kernel whose
  state stays unreadable is relocked after a bound.

  Gates, in order, at the first laser-on of a job: the coolant verdict
  (fire_ok), head presence (the head driver has probed - lens, air assist
  and beam detector are on the head), a switch device on hardware (no
  button to read means no arm), then the operator's button press: the
  button must be seen up before a press counts, so a stuck button
  never arms a job, and three unreadable reads in a row end the wait.
  The head and the verdict are checked again after the wait.

  Config keys (shared machine config, re-read at each arm):
    laser_button_timeout_s   button wait budget (default 300; clamped
                             to 1-3600 - out-of-range values fall back
                             to the default, never wait-forever)
    laser_disarm_s           spindle-off grace before the armed window
                             closes (default 60)
    laser_floor_density      the S-range floor, percent of full
                             (default 10): the lowest density that
                             still marks, so a commanded 1 % is a real
                             mark rather than pulses too far apart to
                             re-strike. Loaded into $35 at every
                             precompute (boot included) and at every
                             arm, in RAM only - the stored $35 is never
                             written, $$ reports the floor in force,
                             and a $35 typed by a sender is overwritten
                             on the spot.
    laser_pulse_ticks        density base period in ticks of the x8
                             reference tick, 28160 Hz (default 20 =
                             710 us); the stream scales it to the tick
                             in force, so the period is a time at every
                             microstep mode
    laser_pulse_min_ticks    shortest pulse the density model will emit,
                             in the same ticks (default 3 = 106 us).
                             Below it a period is
                             skipped and its debt carried, so a low
                             level arrives as fewer full-width pulses
                             rather than stubs the supply cannot strike.

    laser_corner_gamma       the corner rolloff exponent (default 2,
                             range 0.25 to 4). Under M4 the core scales
                             the commanded power with velocity; with
                             the curve making light proportional to the
                             command, corners get exactly proportional
                             light - and the heat still accumulates
                             where the head slows, so they over-burn.
                             The exponent bends the rolloff: delivered
                             light follows (v/v_programmed)^gamma, so
                             gamma 1 is plain proportionality and
                             higher values starve the slow spots. It
                             rides the curve (a curve of "off" disables
                             both) and touches only the velocity-scaled
                             path - a constant-power M3 never sees it.
    M102 reloads laser_corner_gamma, laser_dose_curve, and
    laser_floor_density inside an armed job, after every buffered
    motion has played (synchronized), through the same spindle
    configuration the arm runs, so the PWM mapping is precomputed
    against the new floor: the setup sheet's corner card burns
    one pattern per gamma in a single press, and its floor and
    dose-curve cards burn their text under the machine's keys and
    switch the floor and the curve off for their rungs, the daemon
    writing the keys and streaming M102 ahead.

    laser_dose_curve         the measured dose curve, as density:light
                             percent pairs ("10:0.5,30:7,45:21,60:37,
                             80:50,100:100"). S commands a light
                             fraction and the driver maps it through
                             the curve's inverse onto the density that
                             delivers it, so S500 means half the light
                             rather than half the pulses. Absent = the
                             bench-measured default; "off" = identity
                             (S maps straight onto density, the old
                             behavior); an unparsable value falls back
                             to the default and the arm says so. Read
                             at every precompute; $35/$36 still floor
                             and ceil the result in density percent.

  DENSITY IS THE ONLY DOSE MODEL. The tube fires a strike transient at
  every beam-on, so a continuous analog duty carries a spot at every
  turn-on whatever the power, and the 3 to 14 % duty range is a dead
  band; density pins every pulse at full power and cannot reach either.
  The analog rendering (duty as the power byte, continuous FIRE) exists
  only as the host harness's conservatism reference - the mask rule
  proves density's fire is a subset of what analog would fire - and is
  selectable (laser_power_model = analog, laser_floor_analog) only in
  the null-sink build, never with real hardware attached.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "driver.h"
#include "fflog.h"
#include "glowforge_laser.h"
#include "glowforge_cooling.h"
#include "glowforge_io.h"
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/gcode.h"
#include "grbl/hal.h"
#include "grbl/planner.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <fcntl.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define BUTTON_TIMEOUT_S_DEFAULT 300.0f
#define DISARM_S_DEFAULT         60.0f

/* Hardware PWM resolution: the power byte's 7 bits are written raw into
 * PWMSAR against a 127-count period (scope-verified 40 kHz carrier).
 * One definition, in stepper_stream.h - the shipper renders against it. */
#define PWM_PERIOD GF_PWM_PERIOD

/* Dose model. The tube draws current from ~3 % duty but does not lase
 * usefully below ~16 %, so an analog duty scaled by S walks into a dead
 * band at every corner and short segment. The density model instead
 * pins the duty at full and modulates how many ticks of each base
 * period fire, which cannot land in the band by construction. Default
 * period: 20 ticks of the x8 reference tick (28160 Hz) = 710 us, the
 * factory's ~1.43 kHz pulse rate. The stream scales the count to the
 * tick in force (stepper_stream.h), so the period is a time. */
#define PULSE_TICKS_DEFAULT 20.0f

/* Shortest pulse worth emitting, in the same reference ticks. A 36 us
 * stub (one tick) draws no discharge at all on this supply, and the
 * factory never emits below one of its 100 us ticks; 3 ticks is 106 us.
 * Below this a period is skipped and its debt carried, so a low level
 * arrives as fewer full-width pulses rather than stubs. */
#define PULSE_MIN_TICKS_DEFAULT 3.0f

/* The S-range floor, percent of full, loaded into $35 at every
 * precompute. Density: the lowest density that still marks (bench:
 * strikes ~5 %, marks ~10 %). The analog floor exists only for the
 * host-test reference mode (the duty the tube lases at: strikes 3 %,
 * lases 16 %; 3 to 14 % is a dead band). */
#define FLOOR_DENSITY_DEFAULT 10.0f
#define FLOOR_ANALOG_DEFAULT  16.0f
#define POWER_MODEL_KEY   "laser_power_model"
#define FLOOR_DENSITY_KEY "laser_floor_density"
#define FLOOR_ANALOG_KEY  "laser_floor_analog"
#define PULSE_TICKS_KEY   "laser_pulse_ticks"
#define PULSE_MIN_KEY     "laser_pulse_min_ticks"

typedef enum {
    Model_Density = 0,
    Model_Analog                /* host-test reference only, never on hardware */
} dose_model_t;

/* Spindle state is written by the protocol thread (set_state) and read
 * by the stepper producer's segment updates and the poll below: kept in
 * an atomic byte so an arm edge is never seen half-written. */
static _Atomic uint8_t cur_state_value = 0;
static spindle_pwm_t spindle_pwm;
static bool hw_active;              /* GFSINK set: real device + button */
static _Atomic bool laser_ok = false;   /* armed window open */
static double disarm_at;            /* 0 = no grace running */
static unsigned armed_client_gen;   /* sender session the arm belongs to */
static _Atomic bool disarm_request = false; /* program end: close the window */
static _Atomic bool arming = false;         /* blocked in the button wait */
static bool rearm_pending;                  /* a held job's resume waits for the press */
static double rearm_deadline;
static unsigned rearm_client_gen;           /* the sender the re-arm belongs to */
static int rearm_unreadable;                /* consecutive failed switch reads */
static double disarmed_at;          /* margin for the sample-window lag */
static bool rearm_released;         /* the button came up since the re-arm began */
static bool rearm_pressed;          /* the re-arm's press is in; the acknowledgment is waited for */
static double rearm_ack_deadline;   /* when that wait refuses the job */
static double rearm_ack_budget;
static bool fail_alarm_pending;     /* a reset this module asked for from a standstill still owes its alarm */
static bool modal_drop;             /* a window closed: the parser's M3/M4 is owed its M5 */
static bool dark_watch;             /* the M3/M4 was dropped under a job that may only have stalled */

/* How long an unreadable kernel state may defer the relock at a job's
 * end (the tail the relock waits for is one queue depth, 200 ms). */
#ifndef DISARM_UNREADABLE_S
#define DISARM_UNREADABLE_S 2.0
#endif
/* Consecutive unreadable switch reads that end a button wait. */
#define SWITCH_READ_FAILS_MAX 3
static double next_emission_check;  /* ~1 Hz witness pacing */
static on_program_completed_ptr on_program_completed;
static spindle_ptrs_t *hal_spindle;         /* the registered spindle's table entry */
static dose_model_t cur_model = Model_Density;   /* the model last applied */
static on_state_change_ptr on_state_change;
static on_homing_completed_ptr on_homing_completed;
static bool jogging;                        /* the core is in STATE_JOG */
static spindle_param_t *rate_param;         /* the active spindle's param, as the
                                             * core hands it to get_pwm and
                                             * set_state; the config callback sees
                                             * the registration entry, which has
                                             * none */

/* Producer-thread fire suppression, reported from the protocol thread
 * once per episode: a segment sets the note, the poll reports the first
 * of a run of them and stays quiet until the run has been over for a
 * second. */
enum { Suppress_None = 0, Suppress_Unarmed, Suppress_Coolant };
static _Atomic int suppressed = Suppress_None;
static int suppress_shown = Suppress_None;
static double suppress_last;

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

/* The button says what the machine wants: dark, breathing white (a press
 * unlocks the job), or solid white (the machine is moving). A write stops any
 * pulse first: the LED trigger keeps a pulse going under a new target. The
 * cooling engine's red flash (firing blocked) stands over all of these: the
 * lamp that follows the machine's state is not written over it (lamp_follow),
 * and it puts itself back within a second of any other write here. */
typedef enum { Lamp_Off = 0, Lamp_Press, Lamp_Run } lamp_t;
static int lamp_now = -1;               /* what this process last wrote */

/* How often the lamp is checked against what it should show (lamp_keep). */
#ifndef LAMP_KEEP_S
#define LAMP_KEEP_S 1.0
#endif

/* The LED class; a host test moves it (the test hook GF_LED_ROOT, as
 * forgectrl's), which a machine run ignores. */
static const char *led_root (void)
{
    const char *r = gfio_test_hook("GF_LED_ROOT");
    return r && *r ? r : "/sys/class/leds";
}

static void lamp_wr (uint32_t led, const char *attr, uint32_t val)
{
    char path[160], v[16];
    snprintf(v, sizeof(v), "%u", val);
    snprintf(path, sizeof(path), "%s/button_led_%u/%s", led_root(), led, attr);
    int fd = open(path, O_WRONLY | O_TRUNC | O_CLOEXEC);    /* sysfs ignores the truncation */
    if(fd >= 0) {
        if(write(fd, v, strlen(v)) < 0) { /* LED only; ignore */ }
        close(fd);
    }
}

static void button_lamp (lamp_t lamp)
{
    if((int)lamp == lamp_now)
        return;
    lamp_now = (int)lamp;
    for(uint32_t led = 1; led <= 3; led++) {
        lamp_wr(led, "pulse_on", 0);
        lamp_wr(led, "pulse_off", 0);
        lamp_wr(led, "speed", lamp == Lamp_Press ? 12 : 64);
        lamp_wr(led, "target", lamp == Lamp_Off ? 0 : lamp == Lamp_Press ? 200 : 255);
        if(lamp == Lamp_Press) {
            lamp_wr(led, "pulse_on", 1800);
            lamp_wr(led, "pulse_off", 1800);
        }
    }
}

/* One LED attribute, or -1. */
static int lamp_rd (uint32_t led, const char *attr)
{
    char path[160], v[16];
    snprintf(path, sizeof(path), "%s/button_led_%u/%s", led_root(), led, attr);
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if(fd < 0)
        return -1;
    ssize_t n = read(fd, v, sizeof(v) - 1);
    close(fd);
    if(n <= 0)
        return -1;
    v[n] = '\0';
    return atoi(v);
}

/* The three LEDs as they read: each one's pulse and target. False when an
 * attribute cannot be read, which leaves nothing to judge. */
static bool lamp_read (int p[3], int t[3])
{
    for(uint32_t i = 0; i < 3; i++) {
        p[i] = lamp_rd(i + 1, "pulse_on");
        t[i] = lamp_rd(i + 1, "target");
        if(p[i] < 0 || t[i] < 0)
            return false;
    }
    return true;
}

/* The engine's red flash: red pulsing, green and blue dark. */
static bool lamp_red (const int p[3], const int t[3])
{
    return p[0] > 0 && p[1] <= 0 && p[2] <= 0 && t[1] <= 0 && t[2] <= 0;
}

/* Solid white as the LED trigger reports it. A fade stops within a
 * sixteenth of a count of its target and the trigger then reports where it
 * stopped, so a lamp that has settled on 255 reads its target back as 254;
 * one still fading reads 255. */
#define LAMP_WHITE_MIN 254

/* The lamp put back when something else wrote over it: the engine's red
 * flash, once it ends, leaves the button dark, and a press wait under it
 * must not look like no wait at all. Checked once a second; never written
 * over the red flash itself, which is the engine's to end. */
static void lamp_keep (void)
{
    static double next_at;
    double now = wall_s();
    if(lamp_now < 0 || now < next_at)
        return;
    next_at = now + LAMP_KEEP_S;
    int p[3], t[3];
    if(!lamp_read(p, t))
        return;                         /* unreadable: nothing to judge */
    if(lamp_red(p, t))
        return;                         /* the red flash stands */
    bool ok = lamp_now == Lamp_Press ? (p[0] > 0 && p[1] > 0 && p[2] > 0)
            : lamp_now == Lamp_Run ? (p[0] == 0 && p[1] == 0 && p[2] == 0 &&
                                      t[0] >= LAMP_WHITE_MIN && t[1] >= LAMP_WHITE_MIN &&
                                      t[2] >= LAMP_WHITE_MIN)
            : (p[0] == 0 && p[1] == 0 && p[2] == 0 && t[0] == 0 && t[1] == 0 && t[2] == 0);
    if(!ok) {
        lamp_t want = (lamp_t)lamp_now;
        lamp_now = -1;
        button_lamp(want);
    }
}

/* The lamp for the machine's state, written when the state changes it.
 * Under the engine's red flash the change is recorded and not written: the
 * red is the engine's to end, and lamp_keep writes the lamp once it has. */
static void lamp_follow (lamp_t lamp)
{
    int p[3], t[3];

    if((int)lamp == lamp_now)
        return;
    if(lamp_read(p, t) && lamp_red(p, t))
        lamp_now = (int)lamp;
    else
        button_lamp(lamp);
}

/* The refusal's words for a closed cooling gate: an ACK names the panel. */
static const char *cooling_refusal (void)
{
    return gfcool_ack_waits()
        ? "laser fire blocked: a fault waits for its Acknowledge on the control panel"
        : "laser fire blocked: a cooling gate is closed";
}

/* One reading of the switches for the arm wait. A press only counts with
 * the lid closed and the interlock loop closed - the same condition under
 * which the hardware button latch would clear on it. An unreadable
 * device is its own outcome: the callers bound how long they tolerate
 * it. */
enum { Arm_Waiting = 0, Arm_Pressed, Arm_LidOpen, Arm_InterlockOpen, Arm_Unreadable };

static int arm_switches (void)
{
    uint8_t sw[SW_BYTES];

    if(!gfsw_read_raw(sw))
        return Arm_Unreadable;

    if(!gfsw_bit_set(sw, SW_BIT_DOORS))
        return Arm_LidOpen;
    if(gfsw_bit_set(sw, SW_BIT_INTERLOCK))
        return Arm_InterlockOpen;

    return gfsw_bit_set(sw, SW_BIT_BUTTON) ? Arm_Pressed : Arm_Waiting;
}

/* Pump the protocol while blocked in the button wait (same pattern as
 * the homing session). Returns false once the system is aborting. */
static bool pump (long timeout_us)
{
    serial_wait(timeout_us);
    serial_poll();
    return protocol_execute_realtime();
}

/* All latch writes go through the stream engine's serialized writer:
 * the shipper's run-start relight must be atomic against a concurrent
 * disarm here, or it can re-unlock a latch this module just locked.
 * The writer records the owner and retries; a lock that still fails
 * faults the stream, an unlock that fails returns false. */
static bool latch_lock (bool lock)
{
    return gf_stream_laser_latch(lock);
}

/* The per-tick fire gate the stream engine masks FIRE with: the window
 * and the verdict's own fire_ok, held open through the pause tier's
 * first deceleration so a pause leaves no gap in the cut. Published on
 * every poll and on every arm and disarm. */
static void fire_gate_publish (void)
{
    gf_stream_fire_gate(laser_ok && (gfcool_verdict_fire_ok() || gfcool_decel_lit()));
}

bool gflaser_arming (void)
{
    return atomic_load(&arming);
}

/* Close the window and lock the latch. Idempotent, and the lock does not
 * depend on the window: the latch is locked whenever this process
 * unlocked it, so a reset or a stream fault inside the button wait or
 * the re-arm wait relocks too. */
void gflaser_disarm (void)
{
    bool was = laser_ok;
    bool unlocked = !gf_stream_latch_locked_by_us();

    dark_watch = false;                 /* a reset, a fault, a cancel, a home: no stalled job is left */
    laser_ok = false;
    gf_stream_laser_arm(false);
    gf_stream_fire_gate(false);
    if(unlocked)
        latch_lock(true);
    if(!was && !unlocked)
        return;

    button_lamp(Lamp_Off);
    gfcool_laser_armed(false);
    disarm_at = 0.0;
    disarm_request = false;
    disarmed_at = wall_s();
    if(was) {
        modal_drop = true;              /* the parser's M3/M4 goes with the window (modal_poll) */
        report_message("laser disarmed - latch locked", Message_Info);
    }
}

/* --- dose model ---------------------------------------------------------- */

/* The measured dose curve: light output is convex in pulse density on
 * this tube (80 % density delivers about half the CW light), so a raw
 * S-to-density mapping makes S mean pulses, not light. The curve maps
 * the commanded light fraction through the measured points' inverse
 * onto the density that delivers it. Points are density:light percent
 * pairs, strictly increasing in both, measured by the head thermopile
 *. */
#define CURVE_KEY "laser_dose_curve"
#define GAMMA_KEY "laser_corner_gamma"
#define GAMMA_DEFAULT 2.0f
#define CURVE_MAX_POINTS 12

typedef struct {
    float density[CURVE_MAX_POINTS];    /* percent of full, ascending */
    float light[CURVE_MAX_POINTS];      /* percent of CW, ascending */
    uint8_t n;                          /* 0 = identity (curve off) */
    const char *name;                   /* for the arm report */
} dose_curve_t;

static const dose_curve_t curve_default = {
    .density = { 10.0f, 20.0f, 30.0f, 45.0f, 60.0f, 80.0f, 100.0f },
    .light   = {  0.5f,  2.0f,  7.0f, 21.0f, 37.0f, 50.0f, 100.0f },
    .n = 7, .name = "bench-default"
};

static dose_curve_t curve;              /* the curve in force */
static float corner_gamma = GAMMA_DEFAULT;  /* velocity-rolloff exponent */

/* Parse "d:l,d:l,..." percent pairs. Both sequences must be strictly
 * increasing and inside 0..100, at least two points. Returns false on
 * any violation, leaving *out untouched. */
static bool curve_parse (const char *text, dose_curve_t *out)
{
    dose_curve_t c = { .n = 0, .name = "custom" };
    const char *p = text;
    while(*p && c.n < CURVE_MAX_POINTS) {
        char *end;
        float d = strtof(p, &end);
        if(end == p || *end != ':')
            return false;
        p = end + 1;
        float l = strtof(p, &end);
        if(end == p)
            return false;
        p = *end == ',' ? end + 1 : end;
        if(!(d >= 0.0f && d <= 100.0f && l >= 0.0f && l <= 100.0f) || (c.n == 0 && l <= 0.0f))
            return false;
        if(c.n && !(d > c.density[c.n - 1] && l > c.light[c.n - 1]))
            return false;
        c.density[c.n] = d;
        c.light[c.n] = l;
        c.n++;
    }
    if(*p || c.n < 2)
        return false;
    *out = c;
    return true;
}

/* Load the curve key: absent = the default, "off" = identity, a bad
 * value = the default (reported at the arm). Call at every precompute. */
static void curve_load (void)
{
    char text[192] = "";
    if(gfio_conf_read(CURVE_KEY, text, sizeof(text)) != 0 || text[0] == '\0') {
        curve = curve_default;
        return;
    }
    if(strcmp(text, "off") == 0) {
        curve.n = 0;
        curve.name = "off";
        return;
    }
    if(!curve_parse(text, &curve)) {
        curve = curve_default;
        curve.name = "invalid: bench-default";
    }
}

static void gamma_load (void)
{
    float g = gfio_conf_read_float(GAMMA_KEY, GAMMA_DEFAULT);
    if(!(g >= 0.25f && g <= 4.0f))      /* also catches NaN */
        g = GAMMA_DEFAULT;
    corner_gamma = g;
}

/* Light fraction (0..1) to density fraction (0..1) through the curve's
 * inverse, piecewise linear, clamped to the curve's ends. n = 0 is the
 * identity. */
static float curve_apply (float light)
{
    if(curve.n == 0)
        return light;
    float l = light * 100.0f;
    if(l <= curve.light[0])
        return curve.density[0] * (l / curve.light[0]) / 100.0f;
    uint8_t i = 1;
    while(i < curve.n - 1 && l > curve.light[i])
        i++;
    float span = curve.light[i] - curve.light[i - 1];
    float f = span > 0.0f ? (l - curve.light[i - 1]) / span : 0.0f;
    if(f > 1.0f)
        f = 1.0f;
    return (curve.density[i - 1] + f * (curve.density[i] - curve.density[i - 1])) / 100.0f;
}

const char *gflaser_curve (void)
{
    return curve.n ? curve.name : "off";
}

/* compute_value wrapper: rpm (velocity-scaled by the core under M4)
 * commands a light fraction of full; the curve turns it into the
 * density that delivers it, and $35/$36 still floor and ceil the
 * result. The core's own computation handles off, overrides and the
 * floor for the identity path; the wrapper only bends the middle. */
static uint_fast16_t (*core_compute)(spindle_pwm_t *pwm_data, float rpm, bool pid_limit);

static uint_fast16_t curveComputeValue (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
{
    if(curve.n && rpm > pwm_data->rpm_min && hal_spindle != NULL &&
       hal_spindle->rpm_max > hal_spindle->rpm_min) {
        float light = (rpm - hal_spindle->rpm_min) /
                      (hal_spindle->rpm_max - hal_spindle->rpm_min);
        if(light > 1.0f)
            light = 1.0f;
        /* Corner rolloff: under M4 the core hands over rpm scaled by the
         * segment's velocity ratio (actual over programmed speed) and
         * records that ratio in the spindle's param as it prepares the
         * segment - the block's own, whatever S the parser has reached
         * since, and free of the override the rpm already carries. The
         * synchronous path prepares no segment and reads 1. Bending the
         * ratio by gamma - 1 makes the delivered light follow
         * ratio^gamma: proportional at 1, corner-starved above it. */
        if(corner_gamma != 1.0f && rate_param != NULL) {
            float ratio = rate_param->rate_ratio;
            if(ratio > 0.0f && ratio < 1.0f)
                light *= powf(ratio, corner_gamma - 1.0f);
        }
        uint_fast16_t v = (uint_fast16_t)(curve_apply(light) * (float)PWM_PERIOD + 0.5f);
        if(v > pwm_data->max_value)
            v = pwm_data->max_value;
        if(v < pwm_data->min_value)
            v = pwm_data->min_value;
        return v;
    }
    return core_compute(pwm_data, rpm, pid_limit);
}

static const char *model_name (dose_model_t m)
{
    return m == Model_Analog ? "analog" : "density";
}

/* Density, always, on a machine. The analog reference mode exists only
 * for the host harness (no GFSINK = the null-sink build): with hardware
 * attached the key is ignored, because analog's strike transient puts a
 * spot at every beam-on and its low duties are a dead band. */
static dose_model_t selected_model (void)
{
    char model[16] = "";
    return !hw_active &&
           gfio_conf_read(POWER_MODEL_KEY, model, sizeof(model)) == 0 &&
           strcmp(model, "analog") == 0 ? Model_Analog : Model_Density;
}

/* The model's S-range floor, percent of full, clamped to 0..100 (a
 * missing or unparsable key is the default; NaN fails the range test). */
static float model_floor (dose_model_t m)
{
    float pct = m == Model_Analog
                 ? gfio_conf_read_float(FLOOR_ANALOG_KEY, FLOOR_ANALOG_DEFAULT)
                 : gfio_conf_read_float(FLOOR_DENSITY_KEY, FLOOR_DENSITY_DEFAULT);
    if(!(pct >= 0.0f && pct <= 100.0f))
        pct = m == Model_Analog ? FLOOR_ANALOG_DEFAULT : FLOOR_DENSITY_DEFAULT;
    return pct;
}

static bool spindleConfig (spindle_ptrs_t *spindle);

/* Put the model in force: the stream's rendering, and $35 as the floor
 * with the PWM mapping re-precomputed against it (spindleConfig derives
 * the floor, so boot, a sender's settings write and the arm all agree).
 * RAM only - the stored $35 is never written from here. */
static void apply_model (dose_model_t m)
{
    if(m == Model_Density) {
        float ticks = gfio_conf_read_float(PULSE_TICKS_KEY, PULSE_TICKS_DEFAULT);
        if(!(ticks >= 1.0f))            /* also catches NaN */
            ticks = PULSE_TICKS_DEFAULT;
        float min_ticks = gfio_conf_read_float(PULSE_MIN_KEY, PULSE_MIN_TICKS_DEFAULT);
        if(!(min_ticks >= 1.0f))
            min_ticks = PULSE_MIN_TICKS_DEFAULT;
        gf_stream_laser_model((uint32_t)ticks, (uint32_t)min_ticks);
    } else
        gf_stream_laser_model(0, 0);

    cur_model = m;
    spindleConfig(hal_spindle);
}

bool gflaser_density (void)
{
    return cur_model == Model_Density;
}

bool gflaser_armed (void)
{
    return laser_ok;
}

static void report_model (const char *what)
{
    char msg[112];
    snprintf(msg, sizeof(msg), "%s (%s, floor %g %%, curve %s)", what,
             model_name(cur_model), (double)settings.pwm_spindle.pwm_min_value,
             gflaser_curve());
    report_message(msg, Message_Info);
}

/* The first laser-on of a job: gate, unlock, wait for the operator's
 * button press. Runs on the protocol thread with the planner synced
 * (the core syncs every spindle state change, laser mode included). */

/* A refused or canceled arm ends the job: ALARM:3, then a reset the way
 * an incoming ^X resets it. The core keeps the alarm across the reset,
 * and the reset puts the parser back to M5 and drops the rest of the
 * stream. With the alarm alone the parser kept the job's M3/M4 and its S:
 * after $X the same M3/M4 changed nothing, so the core never asked the
 * spindle again, the arm never ran, and the job moved with every fire
 * suppressed and no prompt. */
static void arm_refused (void)
{
    system_raise_alarm(Alarm_AbortCycle);
    protocol_enqueue_realtime_command(CMD_RESET);
}

/* No head, no beam: the lens, air assist and beam detector live on the
 * head, and the hardware safety chain does not include head presence.
 * Presence is the head driver having probed - its sysfs group exists -
 * not the EV_SW head line. Checked before the latch is unlocked and
 * before the button ever lights, and again after the wait. */
static bool head_present (void)
{
    char hall[16];
    if(!hw_active)
        return true;
    if(gfio_rd_attr("head/hall_sensor", hall, sizeof(hall)) == 0)
        return true;
    report_message("laser fire blocked: no head detected", Message_Warning);
    arm_refused();
    return false;
}

static bool arm_gates (void)
{
    if(!gfcool_fire_ok()) {
        report_message((char *)cooling_refusal(), Message_Warning);
        arm_refused();
        return false;
    }

    if(!head_present())
        return false;

    /* On hardware the button is the consent, and a machine with no
     * switch device has no button to read: no arm. The host build (no
     * GFSINK) keeps the null-sink auto-arm. */
    if(hw_active && !gfsw_available()) {
        report_message("laser fire blocked: no switch device (the button cannot be read)", Message_Warning);
        fflog(LOG_ERR, "gflaser: arm refused - no switch device");
        arm_refused();
        return false;
    }

    return true;
}

/* The arm's completion once the consent exists: the cooling engine's
 * acknowledgment of this job waited for, the coolant gate re-checked at
 * the moment it matters (the wait can run for minutes, and the verdict
 * may have gone bad or stale during it), the dose model put in force,
 * then the window opened. */

/* How long the acknowledgment may take before the job is refused. The
 * engine acknowledges once every gated fan reads its floor, which the
 * big exhaust fan takes seconds to reach, so the budget is the fan
 * grace (cool_fan_grace_s, the engine's own spin-up window) plus a
 * margin, never under COOL_ACK_S. The host harness keeps the grace at
 * 0 so its refusal cases stay short. */
#ifndef COOL_ACK_S
#define COOL_ACK_S 5.0
#endif

static double ack_budget_s (void)
{
    float grace = gfio_conf_read_float("cool_fan_grace_s", 15.0f);
    if(!(grace >= 0.0f && grace <= 120.0f))
        grace = 15.0f;
    double budget = (double)grace + COOL_ACK_S;
    return budget < COOL_ACK_S ? COOL_ACK_S : budget;
}

/* The acknowledgment did not arrive inside its budget: the job is refused. */
static void arm_ack_timeout (double budget)
{
    char msg[112];

    latch_lock(true);
    gfcool_laser_armed(false);
    snprintf(msg, sizeof(msg), "laser fire blocked: the cooling service did not take the "
             "job within %.0f s (the fans at their floors?)", budget);
    report_message(msg, Message_Warning);
    arm_refused();
}

static bool arm_open (unsigned sender_gen);

/* The first arm's completion. It blocks the gcode stream, pumping
 * real-time traffic, until the engine has acknowledged the job, so it runs
 * at Idle only: a held job's re-arm waits for the acknowledgment from the
 * poll instead (rearm_complete). sender_gen is the sender session that
 * asked for the press, taken when the arm's wait began. */
static bool arm_complete (unsigned sender_gen)
{
    /* The armed window is reported to the engine when it opens, and the
     * engine answers by applying the cut airflow for the job. Until that answer arrives the verdict
     * on file is the one computed for the idle session before the arm -
     * permissive, because nothing is wrong at idle - so firing on it
     * puts the beam on the work with the fans still at their idle duty.
     * A press that lands the instant the button lights is exactly the
     * case that reaches the gate first, so the arm waits here. */
    double budget = ack_budget_s();
    double ack_deadline = wall_s() + budget;
    while(!gfcool_run_ack()) {
        if(!pump(50000)) {
            latch_lock(true);
            gfcool_laser_armed(false);
            return false;               /* soft reset during the wait */
        }
        if(wall_s() > ack_deadline) {
            arm_ack_timeout(budget);
            return false;
        }
    }

    /* The window belongs to the sender that asked for the press. A sender
     * that changed while the acknowledgment was waited for is another
     * session: the job is canceled the way a change during the button
     * wait cancels it, and no window opens for the newcomer. */
    if(serial_client_generation() != sender_gen) {
        latch_lock(true);
        gfcool_laser_armed(false);
        report_message("sender changed during arm - job canceled", Message_Warning);
        arm_refused();
        return false;
    }

    return arm_open(sender_gen);
}

/* The window opened, once the engine has acknowledged the job: the gates
 * re-checked at the moment they matter, the dose model put in force.
 * Nothing here waits or pumps the protocol, so the re-arm of a held job
 * runs it from the poll, inside the core's suspend loop. The window's
 * owner is sender_gen, the session that asked for the press: the poll
 * closes the window when the sender in place is any other. */
static bool arm_open (unsigned sender_gen)
{
    if(!gfcool_fire_ok()) {
        latch_lock(true);
        gfcool_laser_armed(false);
        report_message((char *)cooling_refusal(), Message_Warning);
        arm_refused();
        return false;
    }

    /* The head is checked again: the wait can run for minutes, and a
     * head lifted during it must not arm. */
    if(!head_present()) {
        latch_lock(true);
        gfcool_laser_armed(false);
        return false;
    }

    /* Put the dose model in force for this window - rendering and
     * floor both - before any fire reaches the stream. A floor of 0 is
     * a deliberate setting (the threshold ladders run that way) and
     * gets a note, never a refusal. */
    apply_model(selected_model());
    if(settings.pwm_spindle.pwm_min_value <= 0.0f)
        report_message("laser floor is 0: the bottom of the S range will not fire",
                        Message_Warning);

    disarm_request = false;
    modal_drop = false;
    dark_watch = false;
    laser_ok = true;
    disarm_at = 0.0;
    armed_client_gen = sender_gen;
    gf_stream_laser_arm(true);
    fire_gate_publish();
    report_model("laser armed");

    return true;
}

/* The arm's unlock of the latch. An unlock that does not take refuses
 * the arm: the window never opens against a latch of unknown state. */
static bool arm_unlock (void)
{
    if(latch_lock(false))
        return true;
    gfcool_laser_armed(false);
    report_message("laser fire blocked: the laser latch did not unlock", Message_Warning);
    fflog(LOG_ERR, "gflaser: arm refused - the latch unlock write failed");
    arm_refused();
    return false;
}

static bool gflaser_arm (void)
{
    if(!arm_gates())
        return false;

    /* The fan run profile covers the whole armed window, whatever
     * the sender's M8/M9 state. */
    gfcool_laser_armed(true);
    if(!arm_unlock())
        return false;

    /* The sender that asks for the press: the wait for the button and the
     * wait for the acknowledgment both belong to it, and so does the
     * window they open. */
    unsigned sender_gen = serial_client_generation();

    if(gfsw_available()) {
        /* The wait runs with the latch unlocked, so it must always be
         * bounded: out-of-range values (including 0 and garbage that
         * parses negative or NaN) fall back to the default. */
        float timeout_s = gfio_conf_read_float("laser_button_timeout_s", BUTTON_TIMEOUT_S_DEFAULT);
        if(!(timeout_s >= 1.0f && timeout_s <= 3600.0f))
            timeout_s = BUTTON_TIMEOUT_S_DEFAULT;
        double deadline = wall_s() + (double)timeout_s;

        button_lamp(Lamp_Press);
        report_message("press the button to start the laser job", Message_Info);

        /* The lid and the interlock loop are checked here explicitly:
         * the wait runs at Idle, where the door signal is hidden from
         * the core, and an open lid or loop cancels the job outright
         * (the factory does the same; the hardware button latch sets on
         * the lid and would ignore a press anyway). The press counts
         * only after the button has been seen up: a button held down
         * from before the job, or stuck, is not consent. */
        bool pressed = false, aborted = false, released = false;
        int unreadable = 0;
        arming = true;
        while(!pressed && !aborted) {
            /* The consent belongs to the sender that asked for it: a
             * sender change during the wait ends the arm, so a press
             * never lands on another session's lines. */
            if(serial_client_generation() != sender_gen) {
                report_message("sender changed during arm - job canceled", Message_Warning);
                arm_refused();
                aborted = true;
                break;
            }
            /* An alarm raised under the wait (a stream fault) ends it:
             * the disarm has locked the latch already. The reset is
             * arm_refused's, so the job's M3/M4 goes with it. */
            if(state_get() & (STATE_ALARM | STATE_ESTOP)) {
                report_message("alarm during arm - job canceled", Message_Warning);
                protocol_enqueue_realtime_command(CMD_RESET);
                aborted = true;
                break;
            }
            int sw = arm_switches();
            if(sw == Arm_Waiting)
                released = true;
            else if(sw == Arm_Pressed && !released)
                sw = Arm_Waiting;           /* down since before the wait: not consent */
            if(sw == Arm_Unreadable) {
                if(++unreadable >= SWITCH_READ_FAILS_MAX) {
                    report_message("laser arm canceled: the switches cannot be read", Message_Warning);
                    fflog(LOG_ERR, "gflaser: arm canceled - %d consecutive switch reads failed",
                          unreadable);
                    arm_refused();
                    aborted = true;
                    break;
                }
            } else
                unreadable = 0;
            switch(sw) {
                case Arm_Pressed:
                    pressed = true;
                    gfsw_button_consumed();   /* not a pause press */
                    break;
                /* Lid or loop open: the job is canceled the way a mid-job
                 * open cancels it - a soft reset from a standstill (the
                 * position is kept, no alarm), so the sender's stream ends
                 * on the banner and nothing needs unlocking afterward. The
                 * reset lands in the pump below, which then returns false. */
                case Arm_LidOpen:
                    report_message("lid opened during arm - job canceled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                case Arm_InterlockOpen:
                    report_message("interlock open during arm - job canceled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                default:
                    if(!pump(50000))
                        aborted = true;             /* soft reset during the wait */
                    else if(wall_s() > deadline) {
                        report_message("laser arm timed out waiting for the button", Message_Warning);
                        arm_refused();
                        aborted = true;
                    }
                    break;
            }
        }
        arming = false;
        button_lamp(Lamp_Off);

        if(aborted) {
            latch_lock(true);
            gfcool_laser_armed(false);
            return false;
        }
    }

    return arm_complete(sender_gen);
}

/* --- resuming a held job whose window has closed -------------------------- */

/* A cycle start against a held laser job whose window has closed must
 * not reach the core as it is: the core restores the spindle inside the
 * held state, and the blocking arm wait cannot run there - its pump
 * enters the core's suspend loop, which spins until the hold ends, so
 * the press would never be read. The press is collected from the poll
 * instead, the engine's acknowledgment of the job is waited for from the
 * poll as well, and the cycle start is issued once the window is open.
 * Nothing on this path pumps the protocol: the poll runs inside that
 * suspend loop, and a pump from it would never return. */

/* A standing verdict short of the fail tier refuses the resume and keeps
 * the job held, with no alarm, no reset and no unlock: a closed cooling
 * gate (the same resume re-arms once it has cleared), or a fault waiting
 * for its Acknowledge (the panel's Acknowledge, then a press, resumes
 * it). The job is the one the pause tier holds under an open window; with
 * the window closed it is held the same way. The fail tier and a stale
 * verdict end the job on the resume path (arm_gates, arm_open). */
static const char ACK_HOLD_WORDS[] =
    "laser fire blocked: a fault waits for its Acknowledge on the control panel - the job stays held";
static const char GATE_HOLD_WORDS[] =
    "laser fire blocked: a cooling gate is closed - the job stays held";

static const char *hold_refusal (void)
{
    return gfcool_ack_waits() ? ACK_HOLD_WORDS : GATE_HOLD_WORDS;
}

static void rearm_end (bool armed)
{
    rearm_pending = false;
    rearm_pressed = false;
    arming = false;
    button_lamp(Lamp_Off);
    if(!armed) {
        latch_lock(true);
        gfcool_laser_armed(false);
    }
}

/* The consent for the re-arm is in: the press, or on a machine with no
 * button the resume itself. The acknowledgment's budget starts here. */
static void rearm_press_taken (void)
{
    rearm_pressed = true;
    rearm_ack_budget = ack_budget_s();
    rearm_ack_deadline = wall_s() + rearm_ack_budget;
    button_lamp(Lamp_Off);              /* the press is taken */
}

/* The re-arm's completion, tried on every poll once the consent is in:
 * the window opens when the engine has acknowledged the job (the gates
 * re-checked then, as the first arm re-checks them), and the cycle start
 * follows it. An acknowledgment that does not arrive inside its budget
 * refuses the job, as it does at the first arm. */
static void rearm_complete (void)
{
    if(gfcool_hold_stands()) {          /* a gate closed, or a fault came and went, during the wait */
        rearm_end(false);
        report_message((char *)hold_refusal(), Message_Warning);
        return;
    }

    if(!gfcool_run_ack()) {
        if(wall_s() > rearm_ack_deadline) {
            rearm_end(false);
            arm_ack_timeout(rearm_ack_budget);
        }
        return;
    }

    rearm_end(arm_open(rearm_client_gen));
    if(laser_ok)
        protocol_enqueue_realtime_command(CMD_CYCLE_START);
}

bool gflaser_resume_gate (void)
{
    sys_state_t st = state_get();

    if(!(st & (STATE_HOLD | STATE_SAFETY_DOOR)))
        return false;
    if(rearm_pending)
        return true;                    /* the wait already running takes it */
    if(laser_ok || !gc_state.modal.spindle[0].state.on)
        return false;                   /* window open, or nothing to fire */

    if(gfcool_hold_stands()) {
        report_message((char *)hold_refusal(), Message_Warning);
        return true;                    /* refused, the job held */
    }

    if(!arm_gates())
        return true;                    /* refused: the alarm ends the job */

    gfcool_laser_armed(true);
    if(!arm_unlock())
        return true;                    /* refused: the alarm ends the job */

    float timeout_s = gfio_conf_read_float("laser_button_timeout_s", BUTTON_TIMEOUT_S_DEFAULT);
    if(!(timeout_s >= 1.0f && timeout_s <= 3600.0f))
        timeout_s = BUTTON_TIMEOUT_S_DEFAULT;
    rearm_deadline = wall_s() + (double)timeout_s;
    rearm_pending = true;
    rearm_pressed = false;
    rearm_released = false;             /* the press that asked for this must end first */
    rearm_client_gen = serial_client_generation();
    rearm_unreadable = 0;
    arming = true;
    if(gfsw_available()) {
        button_lamp(Lamp_Press);
        report_message("press the button to resume the laser job", Message_Info);
    } else {
        /* No button (host builds): the resume is the consent, and the
         * window opens here when the engine has the job already. */
        rearm_press_taken();
        rearm_complete();
    }

    return true;
}

static void rearm_poll (void)
{
    if(!rearm_pending)
        return;

    if(!(state_get() & (STATE_HOLD | STATE_SAFETY_DOOR))) {
        rearm_end(false);               /* a reset, an alarm or a cancel took the job */
        return;
    }

    /* The consent belongs to the sender whose resume asked for it: a
     * sender change ends the re-arm, so a press never resumes another
     * session's held job. */
    if(serial_client_generation() != rearm_client_gen) {
        rearm_end(false);
        report_message("sender changed - the resume is canceled; the job stays held",
                       Message_Warning);
        return;
    }

    /* The switches are read through the whole re-arm, the wait for the
     * acknowledgment included: a lid or a loop that opens after the press
     * ends it before the window can open. */
    int sw = gfsw_available() ? arm_switches() : Arm_Waiting;
    if(sw == Arm_Unreadable) {
        if(++rearm_unreadable >= SWITCH_READ_FAILS_MAX) {
            rearm_end(false);
            report_message("laser re-arm canceled: the switches cannot be read", Message_Warning);
            fflog(LOG_ERR, "gflaser: re-arm canceled - %d consecutive switch reads failed",
                  rearm_unreadable);
            arm_refused();
        }
        return;
    }
    rearm_unreadable = 0;

    switch(sw) {
        case Arm_Pressed:
            if(rearm_pressed || !rearm_released)
                break;                  /* taken already, or still the press that started the re-arm */
            gfsw_button_consumed();     /* not a pause toggle */
            rearm_press_taken();
            break;
        case Arm_LidOpen:
        case Arm_InterlockOpen:
            /* The lid policy takes the held job from here (a cancel, or
             * the stock door hold); the wait ends without consent. */
            rearm_end(false);
            report_message("lid or interlock open - the resume is canceled", Message_Warning);
            return;
        case Arm_Waiting:
            rearm_released = true;      /* the button is up: the next press counts */
            /* fall through */
        default:
            if(!rearm_pressed && wall_s() > rearm_deadline) {
                rearm_end(false);
                report_message("laser arm timed out waiting for the button - the job stays held",
                               Message_Warning);
                return;
            }
            break;
    }

    if(rearm_pressed)
        rearm_complete();
}

/* --- the cooling verdict's fail tier -------------------------------------- */

/* The job cannot continue. The window closes and the latch locks first
 * (the lock sets the hardware button latch too: the next job needs its
 * own press); the job is then reset the way an incoming ^X resets it
 * (a controlled stop; ALARM:3 from motion). A reset from a standstill
 * raises no alarm of its own, so the alarm is owed until the reset has
 * landed (gflaser_poll pays it). The pause tier has no hook here: the
 * cooling client holds the job and the fire gate masks the stream. */
void gflaser_verdict_fail (const char *name)
{
    if(!laser_ok)
        return;

    char msg[96];
    snprintf(msg, sizeof(msg), "cooling verdict %s - job aborted, laser disarmed", name);
    gflaser_disarm();
    report_message(msg, Message_Warning);
    fflog(LOG_WARNING, "gflaser: %s", msg);
    protocol_enqueue_realtime_command(CMD_RESET);
    fail_alarm_pending = true;
}

/* --- spindle backends ----------------------------------------------------- */

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    hal_spindle = spindle;              /* the core's table entry: stable */

    /* $35 is derived, never typed: the model's floor key is loaded here,
     * so boot reports the true floor, and a $35 written by a sender is
     * overwritten the moment the core re-runs this config. The dose
     * curve reloads on the same cadence. */
    settings.pwm_spindle.pwm_min_value = model_floor(cur_model);
    curve_load();
    gamma_load();

    /* Precompute against a clock that makes one PWM period exactly the
     * hardware's 127 counts, so computed values are raw power bytes. */
    spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle,
                                   (uint32_t)((float)PWM_PERIOD * settings.pwm_spindle.pwm_freq));

    /* S commands light, not pulses: bend the core's computation through
     * the measured curve (the identity when the curve is off). */
    if(spindle_pwm.compute_value != NULL && spindle_pwm.compute_value != curveComputeValue) {
        core_compute = spindle_pwm.compute_value;
        spindle_pwm.compute_value = curveComputeValue;
    }

    return true;
}

static void spindleUpdatePWM (spindle_ptrs_t *spindle, uint_fast16_t pwm);
static bool relock_ready (void);

/* How long a laser-on waits for the kernel to play the tail of the job
 * before it (one queue depth, 200 ms) before that job's window is closed
 * on the spot. */
#ifndef RELOCK_WAIT_S
#define RELOCK_WAIT_S 2.0
#endif

/* A program end has spent the consent: the window it left open is only
 * waiting for the kernel to play the job's tail (gflaser_poll). A laser-on
 * that arrives inside that tail belongs to the next job, so the close is
 * finished here first - the tail waited for, bounded, then the disarm -
 * and the laser-on then meets a closed window and asks for its own press.
 * In a held state the wait cannot run (its pump enters the core's suspend
 * loop) and the window closes at once. Returns false when the system is
 * aborting. */
static bool window_close_pending (void)
{
    bool ok = true;

    if(!(state_get() & (STATE_HOLD | STATE_SAFETY_DOOR))) {
        double deadline = wall_s() + RELOCK_WAIT_S;
        while(ok && laser_ok && !relock_ready() && wall_s() < deadline)
            ok = pump(10000);
    }
    gflaser_disarm();

    return ok;
}

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    /* An M3 or M4 is a job speaking for itself: it gets the arm below,
     * and it is no stalled job for the dark-job watch to stop. */
    if(state.on)
        dark_watch = false;

    /* A window whose close is pending is a closed window. */
    if(state.on && laser_ok && disarm_request && !window_close_pending())
        state.on = Off;                 /* aborting: stay dark */

    /* The arm question is "is the window open", never "was the spindle
     * off a moment ago": a window that closed without the core turning
     * the spindle off (a sender change mid-job, a job whose M5 never
     * arrived) must prompt again at the next laser-on, or the job runs
     * with no press, no run report and no airflow. */
    if(state.on && !laser_ok) {
        if(state_get() == STATE_CHECK_MODE) {
            /* A dry run: nothing unlocks, nothing lights. */
            state.on = Off;
            report_message("laser not armed in check mode", Message_Info);
        } else if(state_get() & (STATE_HOLD | STATE_SAFETY_DOOR)) {
            /* A spindle restore inside a held state, against a closed
             * window: a cycle start the resume gate did not see. The
             * blocking arm wait cannot run here (its pump enters the
             * core's suspend loop, which spins until the hold ends), so
             * the job stays dark and says so; the gate is the path that
             * re-arms a held job. */
            state.on = Off;
            report_message("laser not armed - hold and resume again to re-arm", Message_Warning);
        } else if(!gflaser_arm())
            state.on = Off;             /* refused/aborted: stay dark */
    }

    atomic_store(&cur_state_value, state.value);

    /* The synchronous path carries the whole laser state, and it is the
     * only path that does so between blocks. The core takes a set_state
     * as applied: st_rpm_changed records the level, and the per-segment
     * update that would re-assert it at the next block is skipped while
     * the level is unchanged. So an M3 at the level the previous job cut
     * at (S is modal across M2), or an S word executed with the planner
     * drained, reaches the stream only from here - fire included, through
     * the same gates as the per-segment path - and an M5 reaches it as
     * fire off. The stream re-asserts its wanted state at the first byte
     * of every run: what is pushed here is what the next run lights with,
     * and nothing older may be. */
    uint_fast16_t pwm = spindle_pwm.off_value;
    if(state.on && spindle_pwm.compute_value) {
        if(spindle->param != NULL) {
            spindle->param->rate_ratio = 1.0f;      /* no segment: the programmed speed itself */
            rate_param = spindle->param;
        }
        pwm = spindle_pwm.compute_value(&spindle_pwm, rpm, false);
    }
    spindleUpdatePWM(spindle, pwm);
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    (void)spindle;
    return (spindle_state_t){ .value = atomic_load(&cur_state_value) };
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    /* The per-segment path: the core has just recorded the segment's
     * velocity ratio in this spindle's param. */
    if(spindle->param != NULL)
        rate_param = spindle->param;
    if(spindle_pwm.compute_value == NULL)
        return spindle_pwm.off_value;
    return spindle_pwm.compute_value(&spindle_pwm, rpm, false);
}

/* Runs on the stepper producer thread (per-segment, under the core
 * lock) as well as the protocol thread (state changes, overrides) - no
 * reporting from here, only the deferred suppression note. */
static void spindleUpdatePWM (spindle_ptrs_t *spindle, uint_fast16_t pwm)
{
    (void)spindle;

    bool fire = pwm != spindle_pwm.off_value;

    if(fire && !laser_ok) {
        fire = false;
        pwm = spindle_pwm.off_value;
        suppressed = Suppress_Unarmed;
    } else if(fire && !gfcool_fire_ok()) {
        fire = false;
        pwm = spindle_pwm.off_value;
        suppressed = Suppress_Coolant;
    }

    gf_stream_laser((uint8_t)(pwm > PWM_PERIOD ? PWM_PERIOD : pwm), fire);
}

/* --- lifecycle ------------------------------------------------------------ */

/* True when the latch may relock at a job's end: the kernel is idle or
 * disabled, or has faulted or underrun (nothing left to sever), or its
 * state has stayed unreadable for the bound. Unreadable is never "still
 * playing" for longer than that. */
static bool relock_ready (void)
{
    static double unreadable_since;
    char state[16] = "";

    if(!hw_active)
        return true;
    if(gfio_rd_attr("cnc/state", state, sizeof(state)) != 0) {
        double now = wall_s();
        if(unreadable_since == 0.0)
            unreadable_since = now;
        if(now - unreadable_since < DISARM_UNREADABLE_S)
            return false;
        fflog(LOG_WARNING, "gflaser: kernel state unreadable for %.1f s - relocking anyway",
              now - unreadable_since);
        unreadable_since = 0.0;
        return true;
    }
    unreadable_since = 0.0;
    return strcmp(state, "idle") == 0 || strcmp(state, "disabled") == 0 ||
           strcmp(state, "fault") == 0 || strcmp(state, "underrun") == 0;
}

static void window_poll (void)
{
    fire_gate_publish();

    /* A reset this module asked for from a standstill (the verdict's fail
     * tier, the dark-job watch) owes its alarm once the reset has landed;
     * a reset from motion raised ALARM:3 itself. */
    if(fail_alarm_pending) {
        sys_state_t st = state_get();
        if(st & (STATE_ALARM | STATE_ESTOP))
            fail_alarm_pending = false;
        else if(st == STATE_IDLE && !sys.reset_pending) {
            fail_alarm_pending = false;
            system_raise_alarm(Alarm_AbortCycle);
        }
    }

    {
        int note = atomic_exchange(&suppressed, Suppress_None);
        double t = wall_s();
        if(note != Suppress_None) {
            if(note != suppress_shown)
                report_message(note == Suppress_Coolant
                                ? "laser fire suppressed: a cooling gate is closed"
                                : "laser fire suppressed: laser not armed", Message_Warning);
            suppress_shown = note;
            suppress_last = t;
        } else if(suppress_shown != Suppress_None && t - suppress_last > 1.0)
            suppress_shown = Suppress_None;          /* the episode is over */
    }

    /* Emission witness (~1 Hz): laser_on_sampled counts the last ~1 s
     * window's emitting samples on the gated output of the hardware
     * AND-gate - physical evidence, not a commanded state. Emission
     * while the window is closed (3 s of margin covers the sample
     * window's lag past a disarm) is never legitimate. */
    if(hw_active) {
        double t = wall_s();
        if(t >= next_emission_check) {
            next_emission_check = t + 1.0;
            char v[8] = "";
            if(!laser_ok && (disarmed_at == 0.0 || t - disarmed_at > 3.0) &&
               gfio_rd_attr("cnc/laser_on_sampled", v, sizeof(v)) == 0 &&
               atoi(v) > 0) {
                latch_lock(true);
                report_message("uncommanded laser emission sensed - latch relocked", Message_Warning);
                system_raise_alarm(Alarm_AbortCycle);
            }
        }
    }

    rearm_poll();

    /* Solid white while the machine moves. The arm and the re-arm have the
     * button to themselves while they wait. Either way the lamp is put back
     * if something wrote over it. */
    lamp_keep();
    if(!atomic_load(&arming))
        lamp_follow(state_get() & (STATE_CYCLE | STATE_JOG | STATE_HOMING) ? Lamp_Run : Lamp_Off);

    if(!laser_ok)
        return;

    sys_state_t st = state_get();

    if(st & (STATE_ALARM | STATE_ESTOP | STATE_HOMING)) {
        gflaser_disarm();
        return;
    }

    /* A sender change closes the window: the button press that armed
     * it belonged to the displaced session. A job still running is held
     * as well, so the next sender finds the cut where it stopped rather
     * than a dark pass run to its end; ~ or the button re-arms and
     * resumes it through the resume gate. */
    if(serial_client_generation() != armed_client_gen) {
        if(st == STATE_CYCLE) {
            protocol_enqueue_realtime_command(CMD_FEED_HOLD);
            fflog(LOG_NOTICE, "gflaser: sender changed mid-job - job held, laser disarmed");
            report_message("sender changed - job held, laser disarmed; a resume re-arms",
                           Message_Warning);
        }
        gflaser_disarm();
        return;
    }

    /* Program end closes the window too - the consent is spent with
     * the job. The relock waits for the kernel to finish the queue
     * tail, so the request stays pending until the device is idle (or
     * faulted, or unreadable past the bound). A laser-on that arrives
     * while it is pending finishes the close itself and asks for its own
     * press (window_close_pending). */
    if(disarm_request) {
        if(relock_ready()) {
            gflaser_disarm();
            return;
        }
    }

    /* Otherwise the window closes after a grace. The grace counts down
     * whenever the spindle is off outside a cycle - including a jog, and
     * a job parked in Hold, Door or Tool Change - and, with the spindle
     * still on (a job that left M3/M4 modal), whenever the machine is at
     * rest: Idle with nothing planned, or a jog. A window does not
     * outlive its job at rest, and a jog is not the job the press
     * consented to, so neither widens the window. Only a cycle, or a
     * spindle the core left on in a parked job, keeps it open. */
    spindle_state_t cur = { .value = atomic_load(&cur_state_value) };
    bool at_rest = st == STATE_JOG || (st == STATE_IDLE && plan_get_current_block() == NULL);
    if((st & STATE_CYCLE) || (cur.on && !at_rest)) {
        disarm_at = 0.0;
        return;
    }

    double now = wall_s();
    if(disarm_at == 0.0) {
        float grace_s = gfio_conf_read_float("laser_disarm_s", DISARM_S_DEFAULT);
        if(!(grace_s >= 1.0f && grace_s <= 3600.0f))
            grace_s = DISARM_S_DEFAULT;             /* nan, inf, nonsense: the default */
        disarm_at = now + (double)grace_s;
    } else if(now >= disarm_at) {
        /* Never relock while the kernel still plays a queue tail - a
         * severed FIRE there would truncate the job's last bytes. The
         * grace makes this unreachable in practice; check anyway, with
         * the same bound on an unreadable state. */
        if(relock_ready())
            gflaser_disarm();
        else
            disarm_at = now + 0.5;
    }
}

/* The parser's M3/M4 goes with the window. The core asks the spindle only
 * when an M3/M4/M5 changes the modal state, so a window that closed with
 * M3/M4 still modal would leave the next job's identical M-word asking
 * nothing: no arm, no prompt, the whole job moving with its fire
 * suppressed. Once the machine is back at Idle the modal state is turned
 * off the way the core turns it off (gc_spindle_off: M5 and S0 in the
 * parser and in the reports, and the spindle told off, which asks for no
 * arm), so the next M3/M4 is a state change and asks for the press.
 *
 * A parked job keeps its M3/M4: in Hold, Door or Tool Change the drop
 * waits, because the resume gate re-arms a held job by its modal state,
 * and an arm that opens the window again cancels it. Under an alarm, a
 * homing session or a jog it waits for Idle, which the machine reaches
 * before the parser takes another line. */
static void modal_poll (void)
{
    if(!modal_drop)
        return;

    if(laser_ok) {
        modal_drop = false;             /* armed again: the M3/M4 is the open window's */
        return;
    }

    if(state_get() != STATE_IDLE || rearm_pending || atomic_load(&arming))
        return;

    modal_drop = false;
    if(gc_state.modal.spindle[0].state.on) {
        gc_spindle_off();
        dark_watch = true;              /* a job that was only stalled must not run on dark */
        report_message("laser off (M5): the armed window closed - the next M3 or M4 asks for the button",
                       Message_Info);
        fflog(LOG_NOTICE, "gflaser: the window closed with M3/M4 modal - the parser's spindle state turned off");
    }
}

/* A job does not run on dark. The drop above takes M3/M4 from the parser
 * under whatever was in progress, and a job that had only stalled (a
 * sender that stopped sending for longer than the grace, a long dwell)
 * would carry on with the laser off: every move made, nothing cut, one
 * line in the console. So from the drop on, the first cutting move the
 * core plans with the laser off ends the job the way a refused arm ends
 * it, ALARM:3 and a reset: a feed move (never a rapid, a jog or a system
 * motion) whose block carries a programmed S above zero and a spindle
 * state that is off. The drop zeroes the parser's S, so that S is a word
 * the job sent after it, and a move with no S, or S0, is a dry run and is
 * left alone.
 *
 * The watch ends with whatever shows the job is not a stalled one: an
 * M3 or M4 (a new job asks for its own press), a program end, a reset, an
 * alarm, a disarm of any kind, a window that opens. An M5 that changes
 * nothing is not among them: the core tells the spindle only of a change.
 *
 * The block is looked at where the core has planned it and has not yet
 * started it: on every poll, and each time the core's main loop reads its
 * input (gflaser_input_poll), which follows the line that planned the
 * block with no real-time processing in between. The reset goes first and
 * the alarm follows it, as for the verdict's fail tier: asked for from a
 * running cycle, the core's reset kills the steppers itself and raises
 * ALARM:3, and from a standstill it stops the planned block from ever
 * starting and the alarm is owed until it has landed. */
static void dark_watch_poll (void)
{
    plan_block_t *block;

    if(!dark_watch)
        return;

    if(state_get() & (STATE_ALARM | STATE_ESTOP)) {
        dark_watch = false;
        return;
    }

    if((block = plan_get_recent_block()) == NULL || block->spindle.state.on ||
        !(block->spindle.rpm > 0.0f) || block->condition.rapid_motion ||
        block->condition.jog_motion || block->condition.system_motion)
        return;

    dark_watch = false;
    report_message("the laser window closed while the job waited - job stopped; start it again",
                   Message_Warning);
    fflog(LOG_WARNING, "gflaser: a cutting move was planned with the laser off after the window "
          "closed on a stalled job - job stopped");
    protocol_enqueue_realtime_command(CMD_RESET);
    fail_alarm_pending = true;
}

void gflaser_input_poll (void)
{
    dark_watch_poll();
}

void gflaser_poll (void)
{
    window_poll();
    modal_poll();
    dark_watch_poll();
}

/* Program end (M2/M30): the core calls this on the protocol thread
 * after protocol_buffer_synchronize, so the planner is drained; the
 * kernel ring tail may still be playing, hence the deferred close in
 * gflaser_poll. A % is not a program end here: the core reports one as
 * a program end only for a program read from a file stream, which this
 * controller's transport is not, so a program that ends on % closes its
 * window by the grace. */
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    dark_watch = false;                 /* the job is over: nothing of it is left to stop */

    if(!check_mode && laser_ok)
        disarm_request = true;

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

/* A home closes the window, whichever provider ran it. The web-service
 * home runs in the homing state, which the poll closes the window on; the
 * manual home never leaves Idle, so the close is taken from the event
 * every provider ends on. A home that failed closes it too. */
static void onHomingCompleted (axes_signals_t cycle, bool success)
{
    gflaser_disarm();

    if(on_homing_completed)
        on_homing_completed(cycle, success);
}

/* A jog carries the modal spindle in Grbl (the core copies it into the
 * jog block and never disables the laser for it), so the stream masks
 * fire for as long as the core is jogging: the mask follows the state
 * transitions, which bracket the jog's own blocks in the stream. */
static void onStateChange (sys_state_t state)
{
    bool jog = state == STATE_JOG;
    if(jog != jogging) {
        jogging = jog;
        gf_stream_jog(jog);
    }
    if(on_state_change)
        on_state_change(state);
}

float gflaser_gamma (void)
{
    return corner_gamma;
}

/* M102: reload the keys a job may change between its passes. Chained
 * behind whatever handlers the core or another plugin installed. */
static user_mcode_ptrs_t user_mcode;

static user_mcode_type_t mcodeCheck (user_mcode_t mcode)
{
    if(mcode == UserMCode_Generic2)
        return UserMCode_Normal;
    return user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcodeValidate (parser_block_t *gc_block)
{
    if(gc_block->user_mcode == UserMCode_Generic2) {
        gc_block->user_mcode_sync = true;
        return Status_OK;
    }
    return user_mcode.validate ? user_mcode.validate(gc_block) : Status_Unhandled;
}

static void mcodeExecute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == UserMCode_Generic2) {
        char msg[112];
        spindleConfig(hal_spindle);     /* the floor, the curve, the gamma, the PWM mapping */
        snprintf(msg, sizeof(msg), "laser keys reloaded: corner gamma %.2f, curve %s, floor %g %%",
                 (double)corner_gamma, gflaser_curve(),
                 (double)settings.pwm_spindle.pwm_min_value);
        report_message(msg, Message_Info);
        fflog(LOG_INFO, "%s", msg);
    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

void gflaser_init (void)
{
    const char *dev = getenv("GFSINK");
    hw_active = dev != NULL && *dev != '\0';

    /* A fresh controller establishes the button lamp off: it starts
     * disarmed, so the lamp reflects that. A controller ended mid-arm
     * (a fail-tier stop, a kill) never ran its disarm, which is what
     * clears the arming lamp, and the pulse device is the broker's, so
     * nothing closed it on the way out - without this the ring would
     * glow "press to arm" on the disarmed machine the supervisor just
     * respawned. The same reasoning as clearing a dead session's stream
     * flag when this one takes the device. */
    button_lamp(Lamp_Off);

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;
    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChange;
    on_homing_completed = grbl.on_homing_completed;
    grbl.on_homing_completed = onHomingCompleted;
    user_mcode = grbl.user_mcode;
    grbl.user_mcode.check = mcodeCheck;
    grbl.user_mcode.validate = mcodeValidate;
    grbl.user_mcode.execute = mcodeExecute;

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleUpdatePWM,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "Glowforge laser");
}
