/*
  glowforge_homing.h - homing method dispatch (see glowforge_homing.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <math.h>

// Z is the focal point's height above the tray. After a home the lens
// sits on the hall's rising edge, whose focal height the focus card
// measured (lens_hall_edge_z_mm). The controller counts whole steps, so
// a height is placed on the step grid first: the stored position, the
// home position, and the reported Z then agree.
static inline long gfhome_z_steps (float z_mm, float z_spm)
{
    return lroundf(z_mm * z_spm);
}

static inline float gfhome_grid_z (float z_mm, float z_spm)
{
    return (float)gfhome_z_steps(z_mm, z_spm) / z_spm;
}

// The park after a home: the half-steps from the edge to the park
// height, kept inside the window every head reaches without touching a
// stop (`down` below the edge, `up` above it).
static inline int gfhome_park_steps (float edge_z_mm, float park_z_mm, float z_spm, int down, int up)
{
    long k = gfhome_z_steps(park_z_mm, z_spm) - gfhome_z_steps(edge_z_mm, z_spm);
    return k < -down ? -down : k > up ? up : (int)k;
}

// The lens is never moved without a reference. The Z soft limit is
// always on: until Z is referenced it holds Z where it is (a jog is
// refused, a program move raises the soft-limit alarm before it starts);
// a home references the lens on its hall edge and opens the envelope to
// the head's free travel; a commissioning card that referenced the lens
// itself says so with M103 Z<focal height at the edge> P<free half-steps
// below> Q<above> (P and Q optional: the settings, else the fallback).
// gfhome_reference_z applies that; gfhome_apply_z_limit re-applies the
// standing state (after a settings change, at the driver's start, and
// when the reference is dropped).
void gfhome_reference_z (float z_mm, int below, int above);
void gfhome_apply_z_limit (void);

// Register the "$H" system command (shadows the core's; dispatches on
// the homing_mode key in /data/forgefirm.conf, GFHOME_CONF overrides
// the path). Called from driver_init().
void gfhome_init (void);

// Drop the /run position anchor: the homed reference is no longer
// trustworthy (stream fault, position lost). Homing success rewrites it.
void gfhome_invalidate (void);
