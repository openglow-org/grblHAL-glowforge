/*
  glowforge_homing.c - homing method dispatch for the Glowforge board

  Part of grblHAL-glowforge. The machine has no limit or home switches;
  homing method is selected at runtime by the operator (forgectrl web
  UI) through the shared config file /data/forgefirm/forgefirm.conf:

    homing_mode = gfcloud   $H runs the Glowforge web-service homing
                            sequence via the external one-shot runner
                            (gfhome.py: cloud vision homes X/Y to the
                            factory home corner, Z to the hall sensor).
    homing_mode = manual    $H moves nothing: the operator has pushed the
                            head to the home corner by hand (with the
                            motors released, glowforge_release.c), and $H
                            declares that spot manual_home_x, manual_home_y
                            (the origin by default, never negative). Z is
                            left as it is.
    homing_mode = switches  $H is refused: no limit switch backend exists.
    homing_mode = none      $H falls through to the core, which rejects
                            it while homing is disabled ($22=0).

  The registered "H" system command shadows the core's (the command
  chain searches driver registrations first) and re-reads the config on
  every invocation, so mode changes apply without a restart. The modes
  are rows of one provider table, homing_providers[].

  The gfcloud path hands /dev/glowforge to the runner for the whole
  session: the stream engine is suspended (only possible from a fully
  idle kernel - see gf_stream_suspend), the runner is spawned through
  /bin/sh, and the protocol loop keeps pumping real-time traffic so
  senders get status reports for the minutes the session can take. On
  success the machine position is set to the configured post-homing
  coordinates: the factory home corner is machine origin (back-left,
  workspace all-positive); the runner leaves the lens on the hall
  sensor's rising edge, whose focal height the focus card measured, and
  parks it the half-steps the driver hands it (GFHOME_PARK_HALF_STEPS,
  inside the head's free travel as the focus card found it) at the park
  height, so Z after a home is the park height. A soft
  reset (^X) aborts the session (SIGTERM, then SIGKILL); failures raise
  the homing-fail alarm.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

/* grbl headers first: glibc's stat.h (via fcntl.h) defines an st_mtime
 * macro that would otherwise mangle the field of that name in vfs.h */
#include "fflog.h"
#include "driver.h"
#include "glowforge_homing.h"
#include "glowforge_laser.h"
#include "glowforge_release.h"
#include "glowforge_io.h"
#include "stepper_stream.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define CMD_DEFAULT       "/usr/sbin/gfhome.py"
#define TIMEOUT_S_DEFAULT 300.0f
#define SUSPEND_WAIT_MS   3000     /* decel tail + hold-current drop */
#define KILL_GRACE_MS     5000

/* Position anchor for external status readers (forgectrl): written on
 * homing success, right after the kernel position counters are zeroed,
 * so anchor + counters = machine position from then on. Removed
 * whenever the reference stops being trustworthy: controller start, a
 * homing session starting (the runner moves the machine and clears the
 * counters), or a stream fault. */
#define HOMED_ANCHOR      "/run/grblhal.homed"

/* The "key = value" config parser lives in glowforge_io (shared with
 * the cooling tunables); these are kept as local names only. */
#define cfg_read       gfio_conf_read
#define cfg_read_float gfio_conf_read_float

/* The Z reference: whether Z is referenced, and the envelope it opened. */
static bool z_referenced;
static float z_env_min, z_env_max;

/* What the anchor file holds, so it can be rewritten for fewer axes. */
static float anchor_home[N_AXIS];
static uint8_t anchor_axes;
static char anchor_source[16];

void gfhome_apply_z_limit (void)
{
    float z_spm = settings.axis[Z_AXIS].steps_per_mm;
    if(z_referenced) {
        sys.work_envelope.min.values[Z_AXIS] = z_env_min;
        sys.work_envelope.max.values[Z_AXIS] = z_env_max;
    } else {
        /* Unreferenced: Z stays where it is. */
        float z = (float)sys.position[Z_AXIS] / z_spm;
        sys.work_envelope.min.values[Z_AXIS] = z;
        sys.work_envelope.max.values[Z_AXIS] = z;
    }
    /* The core checks the limit on homed axes only: Z is always
     * referenced, to the edge or to where it stands. X and Y are
     * limited to the bed while they are homed (a gfcloud home sets
     * them, an invalidated position clears them): the core's own $20
     * cannot be turned on with $22 off, so the mask is the driver's. */
    sys.homed.mask |= Z_AXIS_BIT;
    sys.soft_limits.mask |= Z_AXIS_BIT;
    if(sys.homed.mask & X_AXIS_BIT)
        sys.soft_limits.mask |= X_AXIS_BIT;
    if(sys.homed.mask & Y_AXIS_BIT)
        sys.soft_limits.mask |= Y_AXIS_BIT;
}

/* The free travel each way from the edge: the values given, else the
 * focus card's settings, else the fallback window. */
static void lens_window (int *below, int *above)
{
    if(*below < 1 || *below > 40) {
        float f = cfg_read_float("lens_stop_below_steps", 0.0f);
        *below = f >= 1.0f && f <= 40.0f ? (int)f : LENS_WINDOW_DOWN;
    }
    if(*above < 1 || *above > 40) {
        float f = cfg_read_float("lens_stop_above_steps", 0.0f);
        *above = f >= 1.0f && f <= 40.0f ? (int)f : LENS_WINDOW_UP;
    }
}

void gfhome_reference_z (float z_mm, int below, int above)
{
    float z_spm = settings.axis[Z_AXIS].steps_per_mm;
    long steps = gfhome_z_steps(z_mm, z_spm);
    lens_window(&below, &above);
    sys.position[Z_AXIS] = steps;
    sys.home_position[Z_AXIS] = (float)steps / z_spm;
    /* A half-step of slack at each end: the free travel is already two
     * short of the contact. */
    z_env_min = (float)(steps - below - 1) / z_spm;
    z_env_max = (float)(steps + above + 1) / z_spm;
    z_referenced = true;
    sys.homed.mask |= Z_AXIS_BIT;
    gfhome_apply_z_limit();
    sync_position();
    fflog(LOG_INFO, "lens referenced at Z%.2f, free %d half-steps below and %d above",
          (double)sys.home_position[Z_AXIS], below, above);
}

static void anchor_write (const float *home, uint8_t axes, const char *source);

/* The lens reference forgectrl took before this controller started. The
 * daemon sweeps the lens onto the hall sensor's rising edge in the
 * motion-verify window and leaves this marker in the state directory; a
 * lens that could not reach its edge is a motion fault there, so no
 * controller starts at all and this is never read on a machine whose lens
 * did not home. The focal height of the edge is ours to know, not the
 * daemon's: it comes from the same settings a gfcloud home reads. */
void gfhome_startup_reference (void)
{
    const char *dir = getenv("GF_STATE_DIR");
    char path[160], buf[64];
    snprintf(path, sizeof(path), "%s/lens.home",
             dir && *dir ? dir : "/run/forgefirm");
    FILE *f = fopen(path, "r");
    if(f == NULL)
        return;
    char *line = fgets(buf, sizeof(buf), f);
    fclose(f);
    if(line == NULL || strncmp(buf, "edge", 4) != 0)
        return;
    gfhome_reference_z(cfg_read_float("lens_hall_edge_z_mm",
                                      DEFAULT_LENS_HALL_EDGE_Z_MM), 0, 0);

    /* The daemon stepped the lens over GPIO, which the kernel counters
     * never saw, so they no longer describe where the lens is. Zero
     * them and anchor Z on the edge: the counters and the anchor agree
     * again from here, and a reader adding them lands on the same Z the
     * controller holds. X and Y carry no reference and stay relative. */
    float home[N_AXIS] = {0};
    home[Z_AXIS] = sys.home_position[Z_AXIS];
    gf_stream_clear_position();
    anchor_write(home, Z_AXIS_BIT, "startup");
}

void gfhome_invalidate (void)
{
    unlink(HOMED_ANCHOR);
    anchor_axes = 0;
    z_referenced = false;
    /* The position is not trusted: X and Y are no longer homed, and
     * their soft limits go with the reference. */
    sys.homed.mask &= (uint8_t)~(X_AXIS_BIT | Y_AXIS_BIT);
    sys.soft_limits.mask &= (uint8_t)~(X_AXIS_BIT | Y_AXIS_BIT);
    gfhome_apply_z_limit();
}

void gfhome_invalidate_xy (void)
{
    /* The head is about to move without the counters (a motor release):
     * X and Y lose their reference. The lens is not released, so Z keeps
     * its own, and the anchor keeps carrying it: the Z counter was not
     * cleared, so the anchor's Z plus the counter is still the lens. */
    if(anchor_axes & Z_AXIS_BIT)
        anchor_write(anchor_home, Z_AXIS_BIT, anchor_source);
    else {
        unlink(HOMED_ANCHOR);
        anchor_axes = 0;
    }
    sys.homed.mask &= (uint8_t)~(X_AXIS_BIT | Y_AXIS_BIT);
    sys.soft_limits.mask &= (uint8_t)~(X_AXIS_BIT | Y_AXIS_BIT);
    gfhome_apply_z_limit();
    report_add_realtime(Report_Homed);
}

/* `axes` names which components carry a reference: a full home writes
 * all three, the lens reference at startup writes Z alone. `source` names
 * what set it (a provider id, or "startup"), so a reader can tell a
 * position a hand declared from one the machine found. A reader that
 * takes only the three coordinates sees what it always saw. */
static void anchor_write (const float *home, uint8_t axes, const char *source)
{
    FILE *f = fopen(HOMED_ANCHOR, "w");
    if(f) {
        fprintf(f, "%.3f %.3f %.3f %u %s\n",
                home[X_AXIS], home[Y_AXIS], home[Z_AXIS], axes, source);
        fclose(f);
    }
    if(home != anchor_home)
        memcpy(anchor_home, home, sizeof(anchor_home));
    anchor_axes = axes;
    snprintf(anchor_source, sizeof(anchor_source), "%s", source);
}

/* ---- the envelope's far edges ---- */

static bool envelope_is_open;
static float envelope_closed[2];                 /* X's and Y's far edges before the envelope was opened */

/* An axis's far edge: the measured one (envelope_x_mm, _y) for X and Y,
 * the travel for Z. max_travel is stored negative. */
static float far_edge (uint_fast8_t a)
{
    float travel = -settings.axis[a].max_travel;
    if(a > Y_AXIS)
        return travel;
    static const char *const keys[] = { "envelope_x_mm", "envelope_y_mm" };
    float key = cfg_read_float(keys[a], -1.0f), edge = gfhome_clamp_envelope_mm(key, travel);
    if(key > 0.0f && edge != key)
        fflog(LOG_WARNING, "gfhome: %s %g is out of range; using %g", keys[a], (double)key, (double)edge);
    return edge;
}

int gfhome_envelope (bool open)
{
    if((sys.homed.mask & (X_AXIS_BIT | Y_AXIS_BIT)) != (X_AXIS_BIT | Y_AXIS_BIT))
        return -1;
    if(state_get() != STATE_IDLE || gflaser_armed())
        return -2;
    char msg[120];
    for(uint_fast8_t a = X_AXIS; a <= Y_AXIS; a++) {
        if(open && !envelope_is_open)
            envelope_closed[a] = sys.work_envelope.max.values[a];
        sys.work_envelope.max.values[a] = open ? -settings.axis[a].max_travel + GFHOME_ENVELOPE_EXTRA_MM : far_edge(a);
    }
    envelope_is_open = open;
    snprintf(msg, sizeof(msg), open ? "Envelope open for the bed check: X to %.1f, Y to %.1f; jog carefully"
                                    : "Envelope: X to %.1f, Y to %.1f",
             (double)sys.work_envelope.max.values[X_AXIS], (double)sys.work_envelope.max.values[Y_AXIS]);
    report_message(msg, open ? Message_Warning : Message_Info);
    fflog(LOG_NOTICE, "gfhome: %s", msg);
    return 0;
}

bool gfhome_envelope_is_open (void)
{
    return envelope_is_open;
}

void gfhome_envelope_close (void)
{
    if(envelope_is_open) {
        sys.work_envelope.max.values[X_AXIS] = envelope_closed[X_AXIS];
        sys.work_envelope.max.values[Y_AXIS] = envelope_closed[Y_AXIS];
        envelope_is_open = false;
    }
}

/* What every provider does once the machine stands at its home and
 * sys.position says so: the limits, the planner, the counters and the
 * anchor, the core's completion event, and the state. `homed` is what this
 * home referenced; `anchored` is what the anchor carries, which also names
 * an axis whose earlier reference still stands. */
static void home_completed (const float *home, uint8_t homed, uint8_t anchored, const char *source)
{
    gfhome_apply_z_limit();
    sync_position();

    gf_stream_clear_position();
    anchor_write(home, anchored, source);

    if(grbl.on_homing_completed)
        grbl.on_homing_completed((axes_signals_t){ .mask = homed }, true);
    report_add_realtime(Report_Homed);

    state_set(STATE_IDLE);
    st_go_idle();
    grbl.report.feedback_message(Message_None);
}

/* --- session orchestration -------------------------------------------- */

/* Pump the protocol while blocked in the session: flush/collect serial
 * traffic and run the real-time executive (status reports, overrides,
 * reset). Returns false once the system is aborting. */
static bool pump (long timeout_us)
{
    serial_wait(timeout_us);
    serial_poll();
    return protocol_execute_realtime();
}

/* A move that has just ended may still be playing out its decel tail in the
 * kernel. Wait for it, pumping the protocol, for a bounded time. */
bool gfhome_wait_kernel_idle (uint32_t timeout_ms)
{
    uint32_t give_up = hal.get_elapsed_ticks() + timeout_ms;
    while(!gf_stream_kernel_idle()) {
        if((int32_t)(give_up - hal.get_elapsed_ticks()) <= 0 || !pump(20000))
            return false;
    }
    return true;
}

static status_code_t gfcloud_home (sys_state_t entry_state)
{
    char cmd[256];
    /* The command override is a host-test hook: a config file cannot
     * choose what runs as root on the machine. */
    const char *sink = getenv("GFSINK");
    if((sink != NULL && *sink != '\0') ||
       cfg_read("gfcloud_home_cmd", cmd, sizeof(cmd)) != 0 || cmd[0] == '\0')
        strcpy(cmd, CMD_DEFAULT);

    float timeout_key = cfg_read_float("gfcloud_home_timeout_s", TIMEOUT_S_DEFAULT);
    float timeout_s = gfhome_clamp_timeout_s(timeout_key, TIMEOUT_S_DEFAULT);
    if(timeout_s != timeout_key)
        fflog(LOG_WARNING, "gfhome: gfcloud_home_timeout_s %g is out of range; using %g s",
              (double)timeout_key, (double)timeout_s);
    uint32_t timeout_ms = (uint32_t)(timeout_s * 1000.0f);

    state_set(STATE_HOMING);
    gfhome_invalidate();   /* the session moves the machine */

    /* Hand the pulse device over; a just-finished move may still be
     * playing out its decel tail in the kernel. */
    bool suspended = false;
    uint32_t give_up = hal.get_elapsed_ticks() + SUSPEND_WAIT_MS;
    while(!(suspended = gf_stream_suspend()) &&
           (int32_t)(give_up - hal.get_elapsed_ticks()) > 0) {
        if(!pump(20000))
            break;
    }
    if(!suspended) {
        state_set(entry_state);
        return sys.abort ? Status_OK : Status_IdleError;
    }

    fflog(LOG_INFO, "gfhome: starting homing session: %s", cmd);

    /* Hand the runner its share of the budget so it gives up before the
     * SIGTERM deadline. Set pre-fork (env calls are not async-signal-
     * safe in the child); every getenv user runs on this thread. */
    char budget[16];
    snprintf(budget, sizeof(budget), "%u",
             timeout_ms > 60000 ? timeout_ms / 1000 - 30 : 30);
    setenv("GFHOME_TIMEOUT_S", budget, 1);
    /* The park: from the edge to the park height, in whole half-steps,
     * inside the window every head reaches without touching a stop. The
     * runner takes the steps after its reference; Z below is set to
     * match. */
    float edge_z = cfg_read_float("lens_hall_edge_z_mm", DEFAULT_LENS_HALL_EDGE_Z_MM);
    float park_z = cfg_read_float("lens_park_z_mm", DEFAULT_LENS_PARK_Z_MM);
    float z_spm = settings.axis[Z_AXIS].steps_per_mm;
    /* The head's free travel each way from the edge, as the focus card
     * found its stops; the fallback window until then. */
    float below_f = cfg_read_float("lens_stop_below_steps", 0.0f);
    float above_f = cfg_read_float("lens_stop_above_steps", 0.0f);
    int below = below_f >= 1.0f && below_f <= 40.0f ? (int)below_f : LENS_WINDOW_DOWN;
    int above = above_f >= 1.0f && above_f <= 40.0f ? (int)above_f : LENS_WINDOW_UP;
    int park = gfhome_park_steps(edge_z, park_z, z_spm, below, above);
    char park_s[16];
    snprintf(park_s, sizeof(park_s), "%d", park);
    setenv("GFHOME_PARK_HALF_STEPS", park_s, 1);

    pid_t pid = fork();
    if(pid == 0) {
        setpgid(0, 0);   /* own process group: SIGTERM reaches the whole tree */
        execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
        _exit(127);
    }
    unsetenv("GFHOME_TIMEOUT_S");
    unsetenv("GFHOME_PARK_HALF_STEPS");
    if(pid < 0) {
        fflog(LOG_ERR, "gfhome: cannot spawn the homing runner");
        if(!gf_stream_resume())
            system_set_exec_alarm(Alarm_HomingFail);
        state_set(entry_state);
        return Status_IdleError;
    }

    int wstatus = 0;
    bool exited = false, aborted = false, term_sent = false, kill_sent = false;
    uint32_t deadline = hal.get_elapsed_ticks() + timeout_ms;
    uint32_t kill_at = 0;

    while(!exited) {
        pid_t r = waitpid(pid, &wstatus, WNOHANG);
        if(r == pid)
            exited = true;
        else if(r < 0)
            break;
        else {
            if(!pump(50000) && !aborted) {
                aborted = true;
                fflog(LOG_WARNING, "gfhome: abort - terminating the homing session");
            }
            bool overdue = (int32_t)(hal.get_elapsed_ticks() - deadline) > 0;
            if((aborted || overdue) && !term_sent) {
                if(overdue)
                    fflog(LOG_WARNING, "gfhome: homing session timed out");
                kill(-pid, SIGTERM);
                term_sent = true;
                kill_at = hal.get_elapsed_ticks() + KILL_GRACE_MS;
            } else if(term_sent && !kill_sent &&
                       (int32_t)(hal.get_elapsed_ticks() - kill_at) > 0) {
                kill(-pid, SIGKILL);
                kill_sent = true;
            }
        }
    }

    bool homed = exited && !aborted && !term_sent &&
                  WIFEXITED(wstatus) && WEXITSTATUS(wstatus) == 0;

    bool resumed = gf_stream_resume();

    if(!resumed || !homed) {
        if(exited && !aborted && !term_sent && !homed)
            fflog(LOG_ERR, "gfhome: homing runner failed (status %d)",
                  WIFEXITED(wstatus) ? WEXITSTATUS(wstatus) : -1);
        /* Same shape as a failed core homing cycle: back to idle with
         * the alarm queued - the protocol loop broadcasts ALARM and
         * enters the alarm state. On a clean abort the reset path owns
         * both; a lost device alarms unconditionally. */
        if(!resumed || !aborted) {
            system_set_exec_alarm(Alarm_HomingFail);
            if(state_get() == STATE_HOMING)
                state_set(STATE_IDLE);
        }
        return Status_OK;
    }

    /* Homed: the head sits at the factory home position. Machine origin
     * is that corner (back-left, +Y toward the front), workspace all-
     * positive. Z is the focal point's height above the tray: the runner
     * left the lens on the hall sensor's rising edge, whose focal height
     * the focus card measured (lens_hall_edge_z_mm; the default is the
     * bench reference machine's), then parked it the half-steps handed
     * to it, so Z is the edge's height on the step grid plus the park.
     * The Z envelope is the head's free travel around the edge.
     * NOTE: settings.axis[].max_travel is stored negative. */
    long edge_steps = gfhome_z_steps(edge_z, z_spm);
    float home[N_AXIS];
    static const char *const home_keys[] = { "gfcloud_home_x", "gfcloud_home_y" };
    for(uint_fast8_t a = X_AXIS; a <= Y_AXIS; a++) {
        float key = cfg_read_float(home_keys[a], 0.0f);
        home[a] = gfhome_clamp_cloud_home_mm(key, -settings.axis[a].max_travel);
        if(home[a] != key)
            fflog(LOG_WARNING, "gfhome: %s %g is out of range; using %g", home_keys[a],
                  (double)key, (double)home[a]);
    }
    home[Z_AXIS] = (float)(edge_steps + park) / z_spm;

    /* The envelope is the bed. A camera home may lie behind the origin (a
     * negative gfcloud_home_x or _y): the head stands there, so the envelope
     * reaches back to it, or no move from the home could land. */
    uint_fast8_t idx;
    for(idx = 0; idx < N_AXIS; idx++) {
        sys.position[idx] = lroundf(home[idx] * settings.axis[idx].steps_per_mm);
        /* On the step grid, so the head never starts a rounding error
         * outside an envelope that begins where it stands. */
        home[idx] = (float)sys.position[idx] / settings.axis[idx].steps_per_mm;
        sys.home_position[idx] = home[idx];
        sys.work_envelope.min.values[idx] = idx <= Y_AXIS && home[idx] < 0.0f ? home[idx] : 0.0f;
        sys.work_envelope.max.values[idx] = far_edge(idx);
    }
    envelope_is_open = false;
    /* The Z envelope: the head's free travel around the edge, a
     * half-step of slack at each end; the Z soft limit stands on it. */
    z_env_min = (float)(edge_steps - below - 1) / z_spm;
    z_env_max = (float)(edge_steps + above + 1) / z_spm;
    z_referenced = true;
    sys.homed.mask = X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT;
    home_completed(home, X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT, X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT, "gfcloud");

    fflog(LOG_NOTICE, "gfhome: homed - X%.2f Y%.2f Z%.2f (the hall edge at Z%.2f, the lens "
          "parked %+d half-steps from it)",
          home[X_AXIS], home[Y_AXIS], home[Z_AXIS], gfhome_grid_z(edge_z, z_spm), park);

    return Status_OK;
}

/* The manual provider: the operator has put the head in the home corner
 * by hand (back-left, against the stop blocks the operator installed) and
 * $H declares it. Nothing moves, so there is no session, no runner,
 * and no lid gate: the lid is open while the head is being pushed. X and
 * Y become X0 Y0 and homed, and the bed becomes their envelope, which is
 * exactly as true as the placement; the sender is told so at every home.
 * Z is left as it is: the lens carries its own reference from the start. */
static status_code_t manual_home (sys_state_t entry_state)
{
    (void)entry_state;

    /* The counters are cleared below: the kernel has to be done playing. */
    if(!gfhome_wait_kernel_idle(SUSPEND_WAIT_MS))
        return sys.abort ? Status_OK : Status_IdleError;

    /* The declared position has to be one the motors hold. */
    status_code_t status = gfrelease_energize();
    if(status != Status_OK)
        return status;

    /* The coordinate the stop blocks stand for (manual_home_x and _y): the
     * origin unless the operator says otherwise, and never negative. They
     * belong to this provider alone. The blocks are a wall, so they are
     * also where the envelope starts: nothing is reachable behind them. */
    float home[N_AXIS];
    static const char *const home_keys[] = { "manual_home_x", "manual_home_y" };
    for(uint_fast8_t a = X_AXIS; a <= Y_AXIS; a++) {
        float key = cfg_read_float(home_keys[a], 0.0f);
        home[a] = gfhome_clamp_home_mm(key, -settings.axis[a].max_travel);
        if(home[a] != key)
            fflog(LOG_WARNING, "gfhome: %s %g is out of range; using %g", home_keys[a],
                  (double)key, (double)home[a]);
        sys.position[a] = lroundf(home[a] * settings.axis[a].steps_per_mm);
        home[a] = (float)sys.position[a] / settings.axis[a].steps_per_mm;     /* on the step grid */
        sys.home_position[a] = home[a];
        sys.work_envelope.min.values[a] = home[a];
        sys.work_envelope.max.values[a] = far_edge(a);
    }
    envelope_is_open = false;
    /* The counters are cleared for all three axes, so the anchor carries
     * where the lens stands now. */
    home[Z_AXIS] = (float)sys.position[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

    sys.homed.mask |= X_AXIS_BIT|Y_AXIS_BIT;
    home_completed(home, X_AXIS_BIT|Y_AXIS_BIT,
                   X_AXIS_BIT|Y_AXIS_BIT|(z_referenced ? Z_AXIS_BIT : 0), "manual");

    report_message("Manual home: position set where the head was placed. "
                   "Soft limits may not match the machine.", Message_Warning);
    fflog(LOG_NOTICE, "gfhome: manual home - X%.2f Y%.2f declared where the head stands, Z%.2f kept",
          (double)home[X_AXIS], (double)home[Y_AXIS], (double)home[Z_AXIS]);

    return Status_OK;
}

/* No limit switch backend exists: the core's cycle would drive the gantry
 * into the frame for the full search distance against signals that never
 * assert. */
static status_code_t switches_home (sys_state_t entry_state)
{
    (void)entry_state;
    report_message("homing_mode = switches: no limit switches on this machine yet", Message_Warning);
    return Status_SettingDisabled;
}

/* The providers of the homing role: homing_mode names one by id. The kind
 * is a property of the code, never of configuration. A builtin provider is
 * a function in this driver. A runner-fd provider hands the pulse device
 * to a root process that moves the machine for minutes; it suspends and
 * resumes the stream engine around its session, and it is refused while
 * the motors are released, because it would energize them. */
typedef enum {
    HomingProvider_Builtin,
    HomingProvider_RunnerFd
} homing_provider_kind_t;

typedef struct {
    const char *id;
    homing_provider_kind_t kind;
    status_code_t (*run)(sys_state_t entry_state);
} homing_provider_t;

static const homing_provider_t homing_providers[] = {
    { "gfcloud",  HomingProvider_RunnerFd, gfcloud_home },
    { "manual",   HomingProvider_Builtin,  manual_home },
    { "switches", HomingProvider_Builtin,  switches_home }
};

static status_code_t home_cmd (sys_state_t state, char *args)
{
    (void)args;

    char mode[24] = "";
    cfg_read("homing_mode", mode, sizeof(mode));

    const homing_provider_t *provider = NULL;
    for(size_t i = 0; i < sizeof(homing_providers) / sizeof(homing_providers[0]); i++) {
        if(!strcmp(mode, homing_providers[i].id))
            provider = &homing_providers[i];
    }
    if(provider == NULL) {
        if(!gfrelease_active())
            return Status_Unhandled;   /* none (or unset): core $H semantics, disabled */
        gfrelease_report();            /* and never the core's cycle over a released gantry */
        return Status_SystemGClock;
    }

    if(!(state == STATE_IDLE || state == STATE_ALARM))
        return Status_IdleError;

    if(provider->kind == HomingProvider_RunnerFd && gfrelease_active()) {
        gfrelease_report();
        return Status_SystemGClock;
    }

    return provider->run(state);
}

static const sys_command_t homing_command_list[] = {
    { "H", home_cmd }
};

static sys_commands_t homing_commands = {
    .n_commands = sizeof(homing_command_list) / sizeof(sys_command_t),
    .commands = homing_command_list
};

void gfhome_init (void)
{
    gfhome_invalidate();   /* a fresh controller is not homed */
    system_register_commands(&homing_commands);
}
