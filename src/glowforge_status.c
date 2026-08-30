/*
  glowforge_status.c - publish the controller's state for the machine daemon

  Part of grblHAL-glowforge. forgectrl can never open the Grbl socket (a
  new connection displaces the sender - last connection wins), so the
  panel cannot see what a sender sees. This module publishes it the
  other way, mirroring the cooling verdict file forgectrl already
  publishes for this controller: two files under one tmpfs directory,
  written atomically (tmp + rename), that any local reader may poll.

    grbl.settings   the $$ view - every $N=value line, captured from the
                    core's own settings report so the bytes match what a
                    sender sees. Rewritten when a setting changes and
                    when the derived floor moves (a precompute loads
                    the floor key into $35 in RAM without a settings
                    write, so the poll watches the value).
    grbl.state      one JSON object: ts_mono (CLOCK_MONOTONIC, for age),
                    the machine state and alarm code, the sender session
                    (connected, generation, seconds connected, peer
                    address), the laser (armed window, arming wait, dose
                    model and its floor), the exact [GC:...] modal report
                    the sender would get, the feed and rapid override
                    percents, and the driver version. Rewritten on
                    change and on a slow heartbeat so age stays honest.

  Edges only, never a feed: position, buffer fill and live feed rate
  change per segment and stay with the kernel counters and the sender's
  own status polls. Everything here runs on the protocol thread (the
  realtime hook), and a write is a render into a small buffer, a
  compare, and an atomic rename only when something changed.

  GF_STATE_DIR overrides the directory (default /run/forgefirm) so the
  host harnesses can point it at a workdir.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "driver.h"
#include "glowforge_status.h"
#include "glowforge_laser.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/settings.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#define STATE_DIR_DEFAULT "/run/forgefirm"
#define HEARTBEAT_S 5.0

static char state_dir[128];
static bool active = false;
static bool settings_dirty = true;
static char last_state[640];        /* the state JSON minus ts_mono */
static double next_heartbeat;
static on_settings_changed_ptr settings_changed_chain;

static double mono_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

/* Capture writer: the core's report functions write through a
 * stream_write_ptr; pointed here they fill cap_buf instead. */
static char cap_buf[4096];
static size_t cap_len;

static void cap_write (const char *s)
{
    size_t n = strlen(s);
    if(cap_len + n >= sizeof(cap_buf))
        n = sizeof(cap_buf) - 1 - cap_len;
    memcpy(cap_buf + cap_len, s, n);
    cap_len += n;
    cap_buf[cap_len] = '\0';
}

/* Atomic publish: write whole, rename into place. A failure disables
 * the module for the session rather than retrying every tick. */
static void write_file (const char *name, const char *body, size_t len)
{
    char path[192], tmp[200];
    snprintf(path, sizeof(path), "%s/%s", state_dir, name);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if(f == NULL) {
        active = false;
        return;
    }
    bool ok = fwrite(body, 1, len, f) == len;
    ok = fclose(f) == 0 && ok;
    if(!ok || rename(tmp, path) != 0)
        active = false;
}

static const char *state_name (sys_state_t st)
{
    switch(st) {
        case STATE_IDLE:        return "Idle";
        case STATE_CYCLE:       return "Run";
        case STATE_HOLD:        return "Hold";
        case STATE_JOG:         return "Jog";
        case STATE_HOMING:      return "Home";
        case STATE_ESTOP:
        case STATE_ALARM:       return "Alarm";
        case STATE_CHECK_MODE:  return "Check";
        case STATE_SAFETY_DOOR: return "Door";
        case STATE_SLEEP:       return "Sleep";
        case STATE_TOOL_CHANGE: return "Tool";
        default:                return "unknown";
    }
}

/* The settings file: the $$ view through the core's own reporter. The
 * report writes through hal.stream.write; swap it for the capture and
 * restore (the same redirection fs_ram uses for its settings dump). */
static void publish_settings (void)
{
    stream_write_ptr saved = hal.stream.write;
    cap_len = 0;
    cap_buf[0] = '\0';
    hal.stream.write = cap_write;
    report_grbl_settings(true, NULL);
    hal.stream.write = saved;
    write_file("grbl.settings", cap_buf, cap_len);
}

/* The state JSON. Rendered without ts_mono first so an unchanged state
 * costs a compare, not a write; the heartbeat rewrites it with a fresh
 * timestamp so the reader's age stays meaningful. */
static void publish_state (bool force)
{
    char modals[256] = "";
    cap_len = 0;
    cap_buf[0] = '\0';
    report_gcode_modes(cap_write);
    /* One line, "[GC:...]" with the line ending stripped. */
    size_t n = strcspn(cap_buf, "\r\n");
    if(n >= sizeof(modals))
        n = sizeof(modals) - 1;
    memcpy(modals, cap_buf, n);
    modals[n] = '\0';

    sys_state_t st = state_get();
    char body[sizeof(last_state)];
    snprintf(body, sizeof(body),
        "\"state\":\"%s\",\"alarm\":%d,"
        "\"sender\":{\"connected\":%s,\"generation\":%u,\"for_s\":%.0f,\"peer\":\"%s\"},"
        "\"laser\":{\"armed\":%s,\"arming\":%s,\"model\":\"%s\",\"floor_pct\":%g},"
        "\"modals\":\"%s\","
        "\"overrides\":{\"feed\":%d,\"rapid\":%d},"
        "\"driver\":\"%s\"}",
        state_name(st), (int)sys.alarm,
        serial_client_connected() ? "true" : "false",
        serial_client_generation(),
        serial_client_connected() ? mono_s() - serial_client_since() : 0.0,
        serial_client_peer(),
        gflaser_armed() ? "true" : "false",
        gflaser_arming() ? "true" : "false",
        gflaser_density() ? "density" : "analog",
        (double)settings.pwm_spindle.pwm_min_value,
        modals,
        (int)sys.override.feed_rate, (int)sys.override.rapid_rate,
        hal.driver_version ? hal.driver_version : "");

    double now = mono_s();
    if(!force && strcmp(body, last_state) == 0 && now < next_heartbeat)
        return;
    strcpy(last_state, body);
    next_heartbeat = now + HEARTBEAT_S;

    char out[sizeof(body) + 48];
    int len = snprintf(out, sizeof(out), "{\"ts_mono\":%.3f,%s\n", now, body);
    write_file("grbl.state", out, (size_t)len);
}

static void onSettingsChanged (settings_t *stg, settings_changed_flags_t changed)
{
    settings_dirty = true;

    if(settings_changed_chain)
        settings_changed_chain(stg, changed);
}

void gfstatus_poll (void)
{
    if(!active)
        return;

    /* The derived floor changes without a settings write (the spindle
     * precompute derives it in RAM): watch the value. */
    static float last_floor = -1.0f;
    if(settings.pwm_spindle.pwm_min_value != last_floor) {
        last_floor = settings.pwm_spindle.pwm_min_value;
        settings_dirty = true;
    }

    if(settings_dirty) {
        settings_dirty = false;
        publish_settings();
    }
    publish_state(false);
}

void gfstatus_init (void)
{
    const char *dir = getenv("GF_STATE_DIR");
    snprintf(state_dir, sizeof(state_dir), "%s",
             dir && *dir ? dir : STATE_DIR_DEFAULT);
    /* The daemon owns the directory on the machine; create it when
     * running standalone (a host harness workdir already exists). */
    mkdir(state_dir, 0755);
    struct stat sb;
    active = stat(state_dir, &sb) == 0 && S_ISDIR(sb.st_mode);

    settings_changed_chain = grbl.on_settings_changed;
    grbl.on_settings_changed = onSettingsChanged;
}
