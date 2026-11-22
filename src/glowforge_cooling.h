/*
  glowforge_cooling.h - cooling-service client (see glowforge_cooling.c)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "grbl/hal.h"

// Client setup (report target, verdict cache). The cooling engine and
// the machine's thermal posture live in forgectrl; the contract is
// https://docs.forgefirm.org/technical/forgefirm/cooling-engine/. Called from driver_init().
void gfcool_init (void);

// The report secret the supervisor handed this process, "" when there is
// none. The homing runner reports job state in this client's place while
// a gfcloud $H holds the machine, and is handed it at its spawn.
const char *gfcool_report_secret (void);

// hal.coolant backends. Flood (M8/M9) drives the job-state reports;
// the engine applies the fan profiles.
void gfcool_coolant_set (coolant_state_t state);
coolant_state_t gfcool_coolant_get (void);

// Periodic work on the protocol thread (realtime hook): the ~1 Hz
// level-triggered job-state report, the verdict-file read, and the
// hold / resume / fallback enforcement.
void gfcool_poll (void);

// The gate of the window's opening: the verdict's own fire_ok while it
// is fresh and, once the window is reported to the engine, the engine's
// acknowledgment of it. The arm and the re-arm open the window on it
// (after the wait for the acknowledgment, gfcool_run_ack): the
// acknowledgment is the precondition of a window's first fire, never a
// gate inside the open window.
bool gfcool_fire_ok (void);

// The verdict's own fire_ok while it is fresh: false while the engine's
// verdict says so - coolant over the run ceiling, for one - and whenever
// the verdict is missing or stale (the engine being gone must look like
// a fault). The gate inside an open window: the per-tick fire gate and
// the per-segment laser update both read it, and the acknowledgment
// does not enter it. Read from the stepper producer thread as well -
// cached flag reads, no file IO on that path.
bool gfcool_verdict_fire_ok (void);

// True while the pause tier's first hold is still decelerating: the
// fire gate stays open through it, so the pause leaves no dark gap in
// the cut (the kernel's queue plays its lit bytes either way). The
// per-tick fire gate and the per-segment laser update both read it,
// so M3 and M4 cut alike to the stop. False once the head has stopped,
// for every hold taken again under the standing verdict, and whenever
// the verdict is stale: a stale verdict's hold runs dark.
bool gfcool_decel_lit (void);

// True once the engine has answered the armed window this client
// reported: a fresh verdict carrying the engine's own armed flag. The
// window opens before the engine has seen the report, and until it has,
// the verdict on file answers the idle session that preceded the arm -
// so the arm waits on this before it lets the gate above decide.
bool gfcool_run_ack (void);
// The acknowledgment is the window's own: every change of the reported
// armed bit clears it, and it is taken again only from a verdict read
// begun after the engine answered that window's report (the engine takes
// the acknowledgment off its file before it answers a report that closes
// a window or opens one).

// True when the engine has answered no report for 3 s: the reporter is
// stalled, or the engine is not taking reports. The laser module ends an
// open window on it (glowforge_laser.c), before the engine's own silence
// bound (5 s) lets the extension host read the window closed.
bool gfcool_report_stale (void);

// A failure waits for the operator's Acknowledge on the control panel: a
// fresh verdict whose ack_wait field is true, or whose name is ACK (the
// name the engine publishes when no other verdict stands; under another
// verdict only the field says it). fire_ok is false meanwhile and is the
// gate; this picks the refusal's words and keeps a held job held instead
// of resuming it.
bool gfcool_ack_waits (void);

// A prompt on the control panel has the machine's button: a fresh verdict
// whose button_reserved field is true (a hold-gated action's dialog, a
// package's check, a package's ask for a press). fire_ok is false meanwhile
// and is the gate;
// this makes a press seen in the arm's wait no consent, and picks the
// refusal's words. Protocol thread only.
bool gfcool_button_reserved (void);

// Armed-window hook from glowforge_laser.c: reported to the engine,
// which forces the run fan profile while armed,
// whatever the sender's M8/M9 state.
void gfcool_laser_armed (bool armed);

// A fresh verdict that blocks firing short of the fail tier: a pause by
// name (OVERTEMP, COLD, WARMUP, SENSOR, FANS, FLAME, BUMP, EXT, QUIET, BUTTON), or a fault
// that waits for its Acknowledge. It is the pause tier as a job with its
// window closed meets it: the resume of a held job is refused under it
// and the job stays held, where the fail tier (FIRE, CRASH, AIRFLOW,
// CRITICAL) and a stale verdict end the job. False for both of those,
// and for a verdict that lets the laser fire.
bool gfcool_hold_stands (void);

// The operator's feed hold (the sender's ! or 0x82, the button's pause):
// a hold this client took becomes the operator's, and for the rest of the
// cycle the client claims none (a poll that runs before the core acts on
// the hold included), so the verdict that clears resumes nothing (the job
// stays held until the operator's ~ or press).
void gfcool_operator_hold (void);
