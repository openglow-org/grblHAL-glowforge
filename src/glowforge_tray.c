/*
  glowforge_tray.c - the crumb tray, in or out

  Part of grblHAL-glowforge. The crumb tray can be removed. With it in, Z
  is the focal point's height above the tray, as it has always been. With
  it out, the work sits on the floor of the cut area, tray_offset_mm lower,
  and Z is the focal height above that floor: the Z position and the Z
  envelope are both shifted up by the offset. Nothing moves; only the
  numbers change.

    M103 P0   the tray is in
    M103 P1   the tray is out

  Any other P, a missing P, or an axis word on the line is an error, and
  the mode stays as it was. The M-code is synchronized: the planner drains,
  and the switch waits for the kernel to finish playing the last move (the
  wait a package's M-code makes, glowforge_mcode.c) before the frame
  changes, so no step is counted in the wrong frame. The sender is told the
  mode that stands, [MSG:Tray in] or [MSG:Tray out], also when it did not
  change. The controller port's "tray in|out" runs the same line
  (ctlport.c).

  The shift is applied in whole lens half-steps, the offset times $102
  rounded. The edge, the envelope and the position all move by the same
  whole steps, so nothing drifts by rounding. The park after a camera home
  is counted from the edge, so the lens parks in the same physical place in
  both modes; only the Z it reports differs.

  The mode persists across a soft reset, a controller restart and a
  reboot: the marker tray.out beside the shared config says the tray is
  out. This controller is its only writer. The machine daemon reads it for
  the panel, and refuses a change of tray_offset_mm while it stands, so the
  offset in force never changes under a live frame.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "glowforge_tray.h"
#include "fflog.h"
#include "glowforge_homing.h"
#include "glowforge_io.h"
#include "stepper_stream.h"

#include "grbl/hal.h"
#include "grbl/gcode.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MARKER_NAME "tray.out"

/* How long the move before M103 may take to finish playing after the core
 * has handed its steps to the stream. */
#define DRAIN_S 5.0

static user_mcode_ptrs_t chained;

static bool out;            /* the mode in force */
static bool shift_known;    /* shift holds the offset of the tray-out frame in force */
static long shift;

static double now_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static const char *marker_path (void)
{
    static char path[256];
    if(path[0] == '\0' && gfio_conf_sibling(MARKER_NAME, path, sizeof(path)) != 0)
        path[0] = '\0';
    return path;
}

/* The directory entry is made durable too: a power cut right after a
 * switch must not bring back the other mode. */
static void sync_dir (const char *path)
{
    char dir[256];
    const char *slash = strrchr(path, '/');
    snprintf(dir, sizeof(dir), "%.*s", slash ? (int)(slash - path) : 1, slash ? path : ".");
    int fd = open(dir[0] ? dir : "/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if(fd >= 0) {
        fsync(fd);
        close(fd);
    }
}

static bool marker_set (bool on)
{
    const char *path = marker_path();
    if(path[0] == '\0')
        return false;
    if(on) {
        int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if(fd < 0)
            return false;
        bool ok = write(fd, "out\n", 4) == 4 && fsync(fd) == 0;
        close(fd);
        if(!ok) {
            unlink(path);
            return false;
        }
    } else if(unlink(path) != 0 && errno != ENOENT)
        return false;
    sync_dir(path);
    return true;
}

static long offset_steps (void)
{
    float key = gfio_conf_read_float("tray_offset_mm", GFTRAY_OFFSET_DEFAULT_MM);
    float mm = gftray_clamp_offset_mm(key);
    if(mm != key)
        fflog(LOG_WARNING, "tray: tray_offset_mm %g is out of range; using %g", (double)key, (double)mm);
    return gftray_offset_steps(mm, settings.axis[Z_AXIS].steps_per_mm);
}

bool gftray_out (void)
{
    return out;
}

long gftray_shift_steps (void)
{
    if(!out)
        return 0;
    if(!shift_known) {
        shift = offset_steps();
        shift_known = true;
    }
    return shift;
}

static void hold_job (const char *msg)
{
    report_message(msg, Message_Warning);
    fflog(LOG_WARNING, "tray: %s", msg);
    system_set_exec_state_flag(EXEC_FEED_HOLD);
}

/* The switch itself: the marker first, so a mode that could not be saved
 * is a mode that did not change, then the frame. */
static void tray_set (bool want)
{
    if(want != out) {
        long delta = want ? offset_steps() : -gftray_shift_steps();
        if(!marker_set(want)) {
            hold_job(want ? "Tray mode not changed: the tray out could not be saved"
                          : "Tray mode not changed: the tray in could not be saved");
            return;
        }
        out = want;
        shift = want ? delta : 0;
        shift_known = want;
        gfhome_tray_shift(delta);
        fflog(LOG_NOTICE, "tray: %s, Z shifted %+ld half-steps (%+.3f mm)", want ? "out" : "in", delta,
              (double)((float)delta / settings.axis[Z_AXIS].steps_per_mm));
    }
    report_message(out ? "Tray out" : "Tray in", Message_Plain);
}

static user_mcode_type_t mcodeCheck (user_mcode_t mcode)
{
    if(mcode == UserMCode_Generic3)
        return UserMCode_Normal;
    return chained.check ? chained.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcodeValidate (parser_block_t *gc_block)
{
    if(gc_block->user_mcode != UserMCode_Generic3)
        return chained.validate ? chained.validate(gc_block) : Status_Unhandled;
    if(!gc_block->words.p)
        return Status_GcodeValueWordMissing;
    if(!(gc_block->values.p == 0.0f || gc_block->values.p == 1.0f))
        return Status_GcodeValueOutOfRange;
    /* An axis word would be a move in the modal motion mode, on the same
     * line as the frame it moves in changing. */
    if(gc_block->words.x || gc_block->words.y || gc_block->words.z)
        return Status_GcodeAxisWordsExist;
    gc_block->words.p = Off;
    gc_block->user_mcode_sync = true;
    return Status_OK;
}

static void mcodeExecute (sys_state_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode != UserMCode_Generic3) {
        if(chained.execute)
            chained.execute(state, gc_block);
        return;
    }

    /* Every step is the stream's now; the frame changes once the kernel
     * has played them. Pumped as a dwell is: a reset or an alarm ends the
     * wait, and the mode is left as it was. */
    const struct timespec nap = { 0, 5 * 1000 * 1000 };
    double drained = now_s() + DRAIN_S;
    while(!gf_stream_kernel_idle() && now_s() < drained) {
        if(!protocol_execute_realtime() || (state_get() & (STATE_ALARM | STATE_ESTOP)))
            return;
        nanosleep(&nap, NULL);
    }
    if(!gf_stream_kernel_idle()) {
        hold_job("Tray mode not changed: the move before M103 did not finish");
        return;
    }
    tray_set(gc_block->values.p != 0.0f);
}

void gftray_init (void)
{
    const char *path = marker_path();
    out = path[0] != '\0' && access(path, F_OK) == 0;
    if(out)
        fflog(LOG_NOTICE, "tray: out (the Z frame is above the floor of the cut area)");

    chained = grbl.user_mcode;
    grbl.user_mcode.check = mcodeCheck;
    grbl.user_mcode.validate = mcodeValidate;
    grbl.user_mcode.execute = mcodeExecute;
}
