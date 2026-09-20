/*
  glowforge_release.c - X and Y motor release

  Part of grblHAL-glowforge. The manual homing provider needs the head
  pushed to the home corner by hand, with the machine on and the 40 V rail
  up. Two system commands do it:

    $MD   release the X and Y motors: their step currents go to 0, which
          on this machine lets the gantry and the head move freely by hand
          (the drivers stay enabled and the rail is not touched). Idle or
          Alarm only, and never under an open armed window. X and Y lose
          their reference at once, since the head is about to move without
          the counters. Z is not released.
    $ME   energize them again: the hold currents. The position stays
          invalid until a home.

  While the motors are released every motion is refused, from every
  source, and nothing but the operator's own $ME or a manual $H energizes
  them: the lid is open and hands are on the gantry, and a stray jog from a
  sender, a pendant, or a bounced button must not snap the rotors to a
  detent under them. The lock is the core's own: the machine sits in the
  alarm state, where the core refuses every g-code line and every jog with
  an error, whoever sent it. What this module adds is that the lock cannot
  be picked: $X is refused while released, a soft reset leaves the alarm
  standing, a homing runner is refused, the realtime poll puts the alarm
  back if anything else ever clears it, and the current scheme itself writes
  0 to X and Y for as long as the release is held (glowforge_io.c), so not
  even the hold posture can energize them. A marker in the state directory
  carries the release across a controller restart: the new controller takes
  it over before it writes its first current.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "fflog.h"
#include "driver.h"
#include "glowforge_release.h"
#include "glowforge_homing.h"
#include "glowforge_io.h"
#include "glowforge_laser.h"
#include "stepper_stream.h"

#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define STATE_DIR_DEFAULT "/run/forgefirm"
#define MARKER_NAME "motors.released"
#define REPORT_EVERY_MS 2000
#define KERNEL_IDLE_WAIT_MS 3000

static bool released = false;
static bool alarm_before = false;       /* the machine was already in alarm at the release */
static uint32_t last_report;
static bool reported = false;
static status_message_ptr status_message_chain = NULL;
static on_report_handlers_init_ptr report_handlers_init_chain = NULL;

static const char *marker_path (void)
{
    static char path[160];
    if(path[0] == '\0') {
        const char *dir = getenv("GF_STATE_DIR");
        snprintf(path, sizeof(path), "%s/" MARKER_NAME, dir && *dir ? dir : STATE_DIR_DEFAULT);
    }
    return path;
}

static void marker_set (bool on)
{
    if(on) {
        FILE *f = fopen(marker_path(), "w");
        if(f) {
            fputs("released\n", f);
            fclose(f);
        }
    } else
        unlink(marker_path());
}

bool gfrelease_active (void)
{
    return released;
}

void gfrelease_report (void)
{
    uint32_t now = hal.get_elapsed_ticks();
    if(!reported || now - last_report >= REPORT_EVERY_MS) {
        reported = true;
        last_report = now;
        report_message("Motors released: nothing moves until $ME, or $H with homing_mode = manual",
                       Message_Warning);
    }
}

static status_code_t release_cmd (sys_state_t state, char *args)
{
    (void)args;

    if(released)
        return Status_OK;
    if(!(state == STATE_IDLE || state == STATE_ALARM))
        return Status_IdleError;
    if(gflaser_armed() || gflaser_arming()) {
        report_message("Motors not released: a laser job is armed", Message_Warning);
        return Status_IdleError;
    }
    /* A release sent right after a move meets the move's decel tail still
     * playing in the kernel, which refuses a release then. Wait for it. */
    if(!gfhome_wait_kernel_idle(KERNEL_IDLE_WAIT_MS))
        return Status_IdleError;

    /* The flag first: from here the current scheme writes nothing but 0. */
    gfio_xy_released_set(true);
    if(gfio_wr_attr("pic/x_step_current", "0") != 0 || gfio_wr_attr("pic/y_step_current", "0") != 0) {
        gfio_xy_released_set(false);
        gfio_currents_hold();
        report_message("Motors not released: the step currents could not be written", Message_Warning);
        return Status_IdleError;
    }

    released = true;
    marker_set(true);
    reported = false;
    alarm_before = state == STATE_ALARM;
    gfhome_invalidate_xy();
    if(!alarm_before)
        system_raise_alarm(Alarm_HomingRequired);
    report_message("Motors released: X and Y are free and their position is no longer known",
                   Message_Warning);
    fflog(LOG_NOTICE, "release: X and Y released");

    return Status_OK;
}

status_code_t gfrelease_energize (void)
{
    if(!released)
        return Status_OK;

    gfio_xy_released_set(false);
    gfio_currents_hold();

    released = false;
    marker_set(false);
    fflog(LOG_NOTICE, "release: X and Y energized");

    return Status_OK;
}

static status_code_t energize_cmd (sys_state_t state, char *args)
{
    (void)state;
    (void)args;

    if(!released)
        return Status_OK;

    status_code_t status = gfrelease_energize();
    if(status != Status_OK)
        return status;

    /* Back to where the machine was before the release. An alarm that was
     * already standing then is not this command's to clear. */
    if(!alarm_before) {
        sys.alarm = Alarm_None;
        state_set(STATE_IDLE);
    }
    report_message("Motors energized: the position is unknown until a home", Message_Warning);

    return Status_OK;
}

/* $X would unlock the alarm that holds a released machine still. */
static status_code_t unlock_guard (sys_state_t state, char *args)
{
    (void)state;
    (void)args;

    if(!released)
        return Status_Unhandled;        /* the core's own $X */

    gfrelease_report();
    return Status_SystemGClock;
}

void gfrelease_poll (void)
{
    if(released && state_get() == STATE_IDLE)
        system_raise_alarm(Alarm_HomingRequired);
}

/* A refused line gets the core's error; say why, once in a while. */
static status_code_t release_status_message (status_code_t status)
{
    if(released && (status == Status_SystemGClock || status == Status_IdleError))
        gfrelease_report();

    return status_message_chain(status);
}

static void release_report_handlers_init (void)
{
    if(report_handlers_init_chain)
        report_handlers_init_chain();

    status_message_chain = grbl.report.status_message;
    grbl.report.status_message = release_status_message;
}

static const sys_command_t release_command_list[] = {
    { "MD", release_cmd, { .noargs = On }, { .str = "release the X and Y motors" } },
    { "ME", energize_cmd, { .noargs = On }, { .str = "energize the X and Y motors" } },
    { "X", unlock_guard }
};

static sys_commands_t release_commands = {
    .n_commands = sizeof(release_command_list) / sizeof(sys_command_t),
    .commands = release_command_list
};

void gfrelease_adopt (void)
{
    /* A controller that starts over a released gantry (the one before it
     * died, or was restarted) takes the release over, and before the
     * stream engine's start writes its first hold current: hands may be on
     * the gantry. The poll raises the alarm as soon as the core is up. */
    if(access(marker_path(), F_OK) == 0) {
        released = true;
        gfio_xy_released_set(true);
    }
}

void gfrelease_init (void)
{
    if(released)
        fflog(LOG_NOTICE, "release: X and Y are released; taking that over");

    system_register_commands(&release_commands);

    report_handlers_init_chain = grbl.on_report_handlers_init;
    grbl.on_report_handlers_init = release_report_handlers_init;
    status_message_chain = grbl.report.status_message;
    grbl.report.status_message = release_status_message;
}
