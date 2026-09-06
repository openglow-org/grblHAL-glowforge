/*
  boards/glowforge.h - Glowforge factory-board machine constants

  Part of grblHAL-glowforge, the ForgeFIRM grblHAL driver for the stock
  Glowforge (Basic/Plus/Pro) i.MX6 control board.

  Every value below is measured/derived from the factory machine, not
  guessed. Sources: the pulse feeder contract
  (https://docs.forgefirm.org/technical/forgefirm/pulse-feeder-contract/),
  forgefirm/docs/BRINGUP.md (hardware facts bank), and the factory pulse
  streams analyzed with forgefirm/scripts/bench/puls_profile.py:
  - XY: 0.15 mm per full step at x8 microstepping -> 53.333 usteps/mm.
  - Z: 0.70612 mm per full step, driven in half-steps (0.3531 mm) -> 2.832 half-steps/mm, ~10.6 mm travel.
  - Travel moves peak 202 mm/s vector with ~700 mm/s2 ramps on v2.6.0
    factory firmware (header HAxr=132/HAyr=112/HAar=133 at ~5.3 mm/s2 per
    unit; the 2018 firmware ramped at ~1000, so 700/590 is conservative).
  - Z cadence <= ~16 half-steps/s observed -> 300 mm/min cap.

  This file is force-included (via my_machine.h) into every translation
  unit including the grblHAL core, so the core's #ifndef-guarded defaults
  in config.h pick these values up. Keep it to preprocessor defines only.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#define BOARD_GLOWFORGE

#define DEFAULT_X_STEPS_PER_MM 53.333f
#define DEFAULT_Y_STEPS_PER_MM 53.333f
// Z: the lens screw's half-steps per millimeter, 36 over the carriage's
// 12.32 mm of travel. Every head shares the screw, so the scale is a
// constant, not a per-head setting.
#define DEFAULT_Z_STEPS_PER_MM 2.922f

#define DEFAULT_X_MAX_RATE 12000.0f // mm/min (200 mm/s; factory travels at 197-202)
#define DEFAULT_Y_MAX_RATE 12000.0f
#define DEFAULT_Z_MAX_RATE 300.0f

#define DEFAULT_X_ACCELERATION 700.0f // mm/s^2
#define DEFAULT_Y_ACCELERATION 590.0f
#define DEFAULT_Z_ACCELERATION 50.0f

#define DEFAULT_X_MAX_TRAVEL 495.0f // mm
#define DEFAULT_Y_MAX_TRAVEL 279.0f
// Z is the focal point's height above the tray: Z 0 focuses on the bed,
// Z +3 focuses 3 mm above it, on the top of 3 mm material. The lens is a
// 2 in lens under a collimated beam, so the focal point moves 1:1 with
// the lens and +Z is lens up. The travel is the lens carriage's 0.485 in
// (the head drawing); its span in Z is the same whatever the scale.
#define DEFAULT_Z_MAX_TRAVEL 12.32f

// The lens is placed in that frame by one per-head number in
// /data/forgefirm.conf, measured and written by the commissioning focus
// card: lens_hall_edge_z_mm, the focal height above the tray when the
// lens sits on the hall sensor's rising edge, the one reference the head
// has (every head shares the screw and the travel; where along the
// travel the hall trips is the head's). A home leaves the lens on that
// edge, sets Z to the number, then moves the focus to lens_park_z_mm, a
// user setting. The defaults are the bench reference machine's.
#define DEFAULT_LENS_HALL_EDGE_Z_MM 3.35f
#define DEFAULT_LENS_PARK_Z_MM 3.0f
// The head's free travel from the edge, half-steps below and above it,
// as the focus card found its stops by the head accelerometer
// (lens_stop_below_steps, lens_stop_above_steps). Until the card has run,
// or when the stops could not be found on a head, the fallback window
// below clears the stops on any head whose edge sits within six
// half-steps of the bench reference machine's (18 below its bottom stop,
// 20 below its top, the 36 convention). The lens is never driven onto a
// stop on a user's machine: the park and the Z envelope keep the window.
#define LENS_WINDOW_DOWN 10
#define LENS_WINDOW_UP   12

// It is a laser: $32 on by default so senders' M3/M4 dynamic-power
// semantics work without a settings dance. Fire additionally requires
// the operator-armed window (glowforge_laser.c: button press unlocks
// the kernel laser latch) and the hardware safety chain.
#define DEFAULT_LASER_MODE On

// The floor of the laser's output range, as a percent of full. Under
// the shipped FIRE-density dose model this is a DENSITY floor: the
// bottom of the S range maps onto it, so a commanded 1 % lands at about
// 10 % density - measured as the lowest level that still marks, where
// below ~5 % the pulses fall too far apart for the discharge to
// re-strike at all. Under the analog fallback the same setting is a
// DUTY floor and wants ~16 instead, the duty this tube lases at; the
// wrong pairing is a dead band either way, so a machine switched to
// analog must raise it. Both are tube properties, commissioned per
// machine with the ladder drills in forgefirm scripts/bench.
#define DEFAULT_SPINDLE_PWM_MIN_VALUE 10.0f // Percent

// The machine has no limit or home switches; the operator selects the
// homing method at runtime (homing_mode in /data/forgefirm.conf, set
// from the forgectrl web UI - see glowforge_homing.c):
// - gfcloud: $H runs the Glowforge web-service homing session. The
//   cloud's camera homing ends with the head at the factory home
//   position = the BACK-LEFT corner - X min (left), Y min (+Y
//   physically moves the gantry toward the FRONT, operator-verified) -
//   which becomes machine origin (workspace all-positive); Z ends on
//   the hall sensor's edge, placed in the focal frame from the focus
//   card's numbers (never blind-drive Z).
// - switches: $H falls through to the core homing cycle. The core
//   defaults below keep it disabled until the physical switches exist
//   (the limit signals are stubbed in driver.c); the same home-corner
//   convention carries over.
#define DEFAULT_HOMING_ENABLE Off
