/*
  glowforge_laser.c - laser spindle + operator arming for the Glowforge board

  Part of grblHAL-glowforge. Maps grblHAL's laser spindle (M3/M4 + S
  words, $32 laser mode) onto the pulse stream: S values become raw
  7-bit power bytes (the kernel writes them straight into PWMSAR against
  a 127-count period, so the spindle PWM is precomputed to a period of
  exactly 127), and fire rides the per-tick FIRE bit. Power lands via
  the core's per-segment spindle update, which the stepper producer runs
  at exact virtual-tick positions - so the laser only ever fires inside
  motion segments of laser blocks. Jogs, G0 and homing are fire-free by
  construction, and the kernel's end-of-data backstop forces the lines
  low whenever a stream ends.

  ARMING - the operator stays in the loop. FIRE reaches the tube only
  through the hardware AND chain (lid switches, interlock, HV OK, the
  charge-pump watchdog the kernel feeds while a run plays, and the
  BUTTON latch). On the first laser-on of a job this module:

    1. refuses outright while a coolant fire gate is active (flow FAULT
       or over-temperature - glowforge_cooling.c owns the verdicts),
    2. forces the run fan profile on (so flow interrogation covers
       every fire window) and unlocks the kernel laser latch,
    3. lights the button white and BLOCKS the gcode stream - pumping
       real-time traffic like the homing session does - until the
       operator presses the physical button (EV_SW bit 2 on the input
       device), a soft reset aborts, the lid or the interlock loop
       opens (the job is cancelled: relock, alarm), or the timeout
       expires. A press with the lid open does not arm - the hardware
       button latch would not clear on it either.

  The armed window is JOB-based: it persists across the job (S changes
  and M5/M3 toggles do not re-prompt) and closes - relocking the
  latch - at program end (M2/M30/%), whenever the sender's connection
  changes (the consent belonged to the displaced session), after
  laser_disarm_s of spindle-off grace (counting down in Idle and in a
  job parked in Hold, Door or Tool Change - only a cycle, a jog or a
  lingering M3 keeps the window open), or immediately on alarm, homing,
  reset or a stream fault. The coolant fire gate is re-checked after
  the button wait, so a window can never open against a verdict that
  went bad during the wait. While unarmed or gated, fire requests are
  suppressed at the stream and reported.

  Gates, in order, at the first laser-on of a job: the coolant verdict
  (fire_ok), head presence (the head driver has probed - lens, air assist
  and beam detector are on the head), then the operator's button press.

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
    laser_pulse_ticks        density base period in machine ticks
                             (default 20 = 710 us at 28160 Hz)
    laser_pulse_min_ticks    shortest pulse the density model will emit
                             (default 3 = 106 us). Below it a period is
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

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "driver.h"
#include "glowforge_laser.h"
#include "glowforge_cooling.h"
#include "glowforge_io.h"
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/gcode.h"
#include "grbl/hal.h"
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
 * period: 20 ticks of the 28160 Hz stream = 710 us, the factory's
 * ~1.43 kHz pulse rate. */
#define PULSE_TICKS_DEFAULT 20.0f

/* Shortest pulse worth emitting, in machine ticks. A 36 us stub (one
 * tick) draws no discharge at all on this supply, and the factory never
 * emits below one of its 100 us ticks; 3 ticks is 106 us. Below this a
 * period is skipped and its debt carried, so a low level arrives as
 * fewer full-width pulses rather than stubs. */
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
static double disarmed_at;          /* margin for the sample-window lag */
static double next_emission_check;  /* ~1 Hz witness pacing */
static on_program_completed_ptr on_program_completed;
static spindle_ptrs_t *hal_spindle;         /* the registered spindle's table entry */
static _Atomic uint32_t prog_rpm_bits;      /* the programmed rpm, recorded at every
                                             * synced set_state (already override-
                                             * scaled, like the per-segment value):
                                             * in laser mode the core keeps no
                                             * programmed speed anywhere the driver
                                             * can reach at compute time. */
static dose_model_t cur_model = Model_Density;   /* the model last applied */

/* Producer-thread fire suppression, reported from the protocol thread. */
enum { Suppress_None = 0, Suppress_Unarmed, Suppress_Coolant };
static _Atomic int suppressed = Suppress_None;

static void prog_rpm_set (float rpm)
{
    uint32_t b;
    memcpy(&b, &rpm, sizeof(b));
    atomic_store(&prog_rpm_bits, b);
}

static float prog_rpm_get (void)
{
    uint32_t b = atomic_load(&prog_rpm_bits);
    float rpm;
    memcpy(&rpm, &b, sizeof(rpm));
    return rpm;
}

static double wall_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void button_led (uint32_t val)
{
    char path[64], v[16];
    snprintf(v, sizeof(v), "%u", val);
    for(uint32_t led = 1; led <= 3; led++) {
        snprintf(path, sizeof(path), "/sys/class/leds/button_led_%u/target", led);
        int fd = open(path, O_WRONLY);
        if(fd >= 0) {
            if(write(fd, v, strlen(v)) < 0) { /* LED only; ignore */ }
            close(fd);
        }
    }
}

/* One reading of the switches for the arm wait. A press only counts with
 * the lid closed and the interlock loop closed - the same condition under
 * which the hardware button latch would clear on it. */
enum { Arm_Waiting = 0, Arm_Pressed, Arm_LidOpen, Arm_InterlockOpen };

static int arm_switches (void)
{
    uint8_t sw[SW_BYTES];

    if(!gfsw_read_raw(sw))
        return Arm_Waiting;             /* unreadable: keep waiting */

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
 * disarm here, or it can re-unlock a latch this module just locked. */
static void latch_lock (bool lock)
{
    gf_stream_laser_latch(lock);
}

bool gflaser_arming (void)
{
    return atomic_load(&arming);
}

void gflaser_disarm (void)
{
    if(!laser_ok)
        return;

    laser_ok = false;
    gf_stream_laser_arm(false);
    latch_lock(true);
    button_led(0);
    gfcool_laser_armed(false);
    disarm_at = 0.0;
    disarm_request = false;
    disarmed_at = wall_s();
    report_message("laser disarmed - latch locked", Message_Info);
}

/* --- dose model ---------------------------------------------------------- */

/* The measured dose curve: light output is convex in pulse density on
 * this tube (80 % density delivers about half the CW light), so a raw
 * S-to-density mapping makes S mean pulses, not light. The curve maps
 * the commanded light fraction through the measured points' inverse
 * onto the density that delivers it. Points are density:light percent
 * pairs, strictly increasing in both, measured by the head thermopile
 * and judged on material (2026-08-30). */
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
        if(!(d >= 0.0f && d <= 100.0f && l >= 0.0f && l <= 100.0f))
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
        /* Corner rolloff: rpm arrives velocity-scaled under M4; the
         * programmed speed is the one this driver recorded at the last
         * synced set_state (in laser mode the core keeps it nowhere the
         * compute path can reach). Bending the ratio by gamma - 1 makes
         * the delivered light follow ratio^gamma: proportional at 1,
         * corner-starved above it. The synchronous path passes the
         * programmed rpm itself (ratio 1), so only real velocity
         * scaling is shaped; an S change buried deep in a full planner
         * queue reaches the recording one sync late, which a raster
         * never sees (its velocity is constant where it fires). */
        if(corner_gamma != 1.0f) {
            float prog = prog_rpm_get();
            if(prog > hal_spindle->rpm_max)
                prog = hal_spindle->rpm_max;
            if(prog > 0.0f && rpm < prog) {
                float ratio = rpm / prog;
                light *= powf(ratio, corner_gamma - 1.0f);
            }
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
static bool gflaser_arm (void)
{
    if(!gfcool_fire_ok()) {
        report_message("laser fire blocked: coolant flow fault or over-temperature", Message_Warning);
        system_raise_alarm(Alarm_AbortCycle);
        return false;
    }

    /* No head, no beam: the lens, air assist and beam detector live on
     * the head, and the hardware safety chain does not include head
     * presence. Presence is the head driver having probed - its sysfs
     * group exists - not the EV_SW head line. Checked before the latch
     * is unlocked and before the button ever lights. */
    if(hw_active) {
        char hall[16];
        if(gfio_rd_attr("head/hall_sensor", hall, sizeof(hall)) != 0) {
            report_message("laser fire blocked: no head detected", Message_Warning);
            system_raise_alarm(Alarm_AbortCycle);
            return false;
        }
    }

    /* Fan run profile + flow interrogation cover the whole armed
     * window, whatever the sender's M8/M9 state. */
    gfcool_laser_armed(true);
    latch_lock(false);

    if(gfsw_available()) {
        /* The wait runs with the latch unlocked, so it must always be
         * bounded: out-of-range values (including 0 and garbage that
         * parses negative or NaN) fall back to the default. */
        float timeout_s = gfio_conf_read_float("laser_button_timeout_s", BUTTON_TIMEOUT_S_DEFAULT);
        if(!(timeout_s >= 1.0f && timeout_s <= 3600.0f))
            timeout_s = BUTTON_TIMEOUT_S_DEFAULT;
        double deadline = wall_s() + (double)timeout_s;

        button_led(255);
        report_message("press the button to start the laser job", Message_Info);

        /* The lid and the interlock loop are checked here explicitly:
         * the wait runs at Idle, where the door signal is hidden from
         * the core, and an open lid or loop cancels the job outright
         * (the factory does the same; the hardware button latch sets on
         * the lid and would ignore a press anyway). */
        bool pressed = false, aborted = false;
        arming = true;
        while(!pressed && !aborted) {
            switch(arm_switches()) {
                case Arm_Pressed:
                    pressed = true;
                    gfsw_button_consumed();   /* not a pause press */
                    break;
                /* Lid or loop open: the job is cancelled the way a mid-job
                 * open cancels it - a soft reset from a standstill (the
                 * position is kept, no alarm), so the sender's stream ends
                 * on the banner and nothing needs unlocking afterward. The
                 * reset lands in the pump below, which then returns false. */
                case Arm_LidOpen:
                    report_message("lid opened during arm - job cancelled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                case Arm_InterlockOpen:
                    report_message("interlock open during arm - job cancelled", Message_Warning);
                    protocol_enqueue_realtime_command(CMD_RESET);
                    aborted = !pump(50000);
                    break;
                default:
                    if(!pump(50000))
                        aborted = true;             /* soft reset during the wait */
                    else if(wall_s() > deadline) {
                        report_message("laser arm timed out waiting for the button", Message_Warning);
                        system_raise_alarm(Alarm_AbortCycle);
                        aborted = true;
                    }
                    break;
            }
        }
        arming = false;
        button_led(0);

        if(aborted) {
            latch_lock(true);
            gfcool_laser_armed(false);
            return false;
        }
    }

    /* Re-check the coolant gate at the moment it matters: the wait can
     * run for minutes, and the verdict may have gone bad (or stale)
     * during it. */
    if(!gfcool_fire_ok()) {
        latch_lock(true);
        gfcool_laser_armed(false);
        report_message("laser fire blocked: coolant flow fault or over-temperature", Message_Warning);
        system_raise_alarm(Alarm_AbortCycle);
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
    laser_ok = true;
    disarm_at = 0.0;
    armed_client_gen = serial_client_generation();
    gf_stream_laser_arm(true);
    report_model("laser armed");

    return true;
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

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    /* The arm question is "is the window open", never "was the spindle
     * off a moment ago": a window that closed without the core turning
     * the spindle off (a sender change mid-job, a job whose M5 never
     * arrived) must prompt again at the next laser-on, or the job runs
     * with no press, no run report and no airflow. */
    /* In laser mode the core turns the spindle on at rpm 0 and the
     * programmed S never reaches this call; the poll below mirrors the
     * parser's programmed speed instead. Record what arrives anyway so
     * a non-zero synchronous change is never lost. */
    if(state.on && rpm > 0.0f)
        prog_rpm_set(rpm);
    else if(!state.on)
        prog_rpm_set(0.0f);

    if(state.on && !laser_ok && !gflaser_arm())
        state.on = Off;                 /* refused/aborted: stay dark */

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
    if(state.on && spindle_pwm.compute_value)
        pwm = spindle_pwm.compute_value(&spindle_pwm, rpm, false);
    spindleUpdatePWM(spindle, pwm);
}

static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    (void)spindle;
    return (spindle_state_t){ .value = atomic_load(&cur_state_value) };
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    (void)spindle;
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

void gflaser_poll (void)
{
    /* The programmed S for the corner rolloff: mirrored from the parser
     * state on the protocol thread every pass (atomic bits, so the
     * producer's compute reads it torn-free). Exact for a job cutting
     * at one S; an S change buried deep in a full planner queue reaches
     * the mirror early by the lookahead, which shapes at most the
     * boundary segments of the previous block - and never a raster,
     * whose velocity is constant wherever it fires. */
    {
        spindle_state_t st_now = { .value = atomic_load(&cur_state_value) };
        if(st_now.on) {
            float g = gc_state.modal.spindle[0].rpm;
            if(g > 0.0f)
                prog_rpm_set(g);
        }
    }
    int note = atomic_exchange(&suppressed, Suppress_None);
    if(note != Suppress_None)
        report_message(note == Suppress_Coolant
                        ? "laser fire suppressed: coolant flow fault or over-temperature"
                        : "laser fire suppressed: laser not armed", Message_Warning);

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

    if(!laser_ok)
        return;

    sys_state_t st = state_get();

    if(st & (STATE_ALARM | STATE_ESTOP | STATE_HOMING)) {
        gflaser_disarm();
        return;
    }

    /* A sender change closes the window: the button press that armed
     * it belonged to the displaced session. */
    if(serial_client_generation() != armed_client_gen) {
        gflaser_disarm();
        return;
    }

    /* Program end closes the window too - the consent is spent with
     * the job. The relock waits for the kernel to finish the queue
     * tail, so the request stays pending until the device is idle. */
    if(disarm_request) {
        char state[16] = "";
        if(!hw_active || (gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                           strcmp(state, "idle") == 0)) {
            gflaser_disarm();
            return;
        }
    }

    /* Otherwise the window closes after a spindle-off grace. The grace
     * counts down whenever the spindle is off - including a job parked
     * in Hold, Door or Tool Change - and only a cycle, a jog or a
     * lingering M3 keeps the window open. */
    spindle_state_t cur = { .value = atomic_load(&cur_state_value) };
    if(cur.on || (st & (STATE_CYCLE | STATE_JOG))) {
        disarm_at = 0.0;
        return;
    }

    double now = wall_s();
    if(disarm_at == 0.0)
        disarm_at = now + (double)gfio_conf_read_float("laser_disarm_s", DISARM_S_DEFAULT);
    else if(now >= disarm_at) {
        /* Never relock while the kernel still plays a queue tail - a
         * severed FIRE there would truncate the job's last bytes. The
         * grace makes this unreachable in practice; check anyway. */
        char state[16] = "";
        if(!hw_active || (gfio_rd_attr("cnc/state", state, sizeof(state)) == 0 &&
                           strcmp(state, "idle") == 0))
            gflaser_disarm();
        else
            disarm_at = now + 1.0;
    }
}

/* Program end (M2/M30/%): the core calls this on the protocol thread
 * after protocol_buffer_synchronize, so the planner is drained; the
 * kernel ring tail may still be playing, hence the deferred close in
 * gflaser_poll. */
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(!check_mode && laser_ok)
        disarm_request = true;

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

void gflaser_init (void)
{
    const char *dev = getenv("GFSINK");
    hw_active = dev != NULL && *dev != '\0';

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

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
