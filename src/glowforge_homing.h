/*
  glowforge_homing.h - homing method dispatch (see glowforge_homing.c)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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

// The homing session's budget, seconds, held to 10..3600: a value out of
// range lands on the nearer end, and one that is not a number (NaN, an
// unparsable key) takes `dflt`. The session runs with the pulse device
// handed over, so it is always bounded.
#define GFHOME_TIMEOUT_MIN_S 10.0f
#define GFHOME_TIMEOUT_MAX_S 3600.0f

static inline float gfhome_clamp_timeout_s (float s, float dflt)
{
    if(!(s == s))
        return dflt;
    return s < GFHOME_TIMEOUT_MIN_S ? GFHOME_TIMEOUT_MIN_S
         : s > GFHOME_TIMEOUT_MAX_S ? GFHOME_TIMEOUT_MAX_S : s;
}

// The coordinate a home declares. Each homing provider has its own pair of
// keys and reads no other's.
//
// A manual home (manual_home_x, _y) declares what the stop blocks stand
// for: 0 to `travel` (the axis's travel, positive), never negative, since
// nothing is reachable behind the blocks.
//
// A camera home (gfcloud_home_x, _y) declares where the service leaves the
// head, and that may lie behind the origin the operator calibrated: it is
// held to `travel` either side of the origin.
//
// A value that is not a number is the origin in both.
static inline float gfhome_clamp_home_mm (float mm, float travel)
{
    if(!(mm == mm))
        return 0.0f;
    return mm < 0.0f ? 0.0f : mm > travel ? travel : mm;
}

static inline float gfhome_clamp_cloud_home_mm (float mm, float travel)
{
    if(!(mm == mm))
        return 0.0f;
    return mm < -travel ? -travel : mm > travel ? travel : mm;
}

// The lens is never moved without a reference. The Z soft limit is
// always on: until Z is referenced it holds Z where it is (a jog is
// refused, a program move raises the soft-limit alarm before it starts);
// a home references the lens on its hall edge and opens the envelope to
// the head's free travel. gfhome_reference_z applies a reference (the
// focal height at the edge, and the free half-steps below and above it:
// pass 0 for either to take the settings, else the fallback);
// gfhome_apply_z_limit re-applies the standing state (after a settings
// change, at the driver's start, and when the reference is dropped).
// The X and Y soft limits are the driver's as well: on after a
// successful home (the envelope is the bed), off while the position is
// not trusted. The core's $20 stays off ($22 is off by design).
void gfhome_reference_z (float z_mm, int below, int above);
void gfhome_apply_z_limit (void);

// Take the lens reference forgectrl left before this controller started
// (the lens stands on its hall edge, and the marker says so). Called once
// from the driver's settings-changed hook, where the step scale the
// reference needs is finally loaded. Silent when there is no marker: the
// lens then stays unreferenced and Z stays pinned where it stands.
void gfhome_startup_reference (void);

// Register the "$H" system command (shadows the core's; dispatches on
// the homing_mode key in /data/forgefirm/forgefirm.conf, GFHOME_CONF overrides
// the path). Called from driver_init().
void gfhome_init (void);

// Drop the /run position anchor: the homed reference is no longer
// trustworthy (stream fault, position lost). Homing success rewrites it.
void gfhome_invalidate (void);

// Wait, pumping the protocol, until the kernel has finished playing what a
// move left queued (its decel tail), or the time runs out. False on a
// timeout or an abort.
bool gfhome_wait_kernel_idle (uint32_t timeout_ms);

// Drop the X and Y reference alone: the head is about to be moved by hand
// (a motor release). The lens is not released, so Z keeps its reference
// and the anchor keeps carrying it.
void gfhome_invalidate_xy (void);
