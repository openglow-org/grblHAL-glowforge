/*
  glowforge_io.h - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

// Attribute paths are relative to /sys/glowforge/ (e.g. "cnc/state",
// "pic/x_step_current"). All return 0 on success, -1 on failure.
void gfio_set_hw (bool active);
// 0 on a complete write; GFIO_ENOATTR when the attribute does not exist
// (module not loaded / older kernel); GFIO_EREJECT when the store refused
// or truncated the value.
#define GFIO_ENOATTR (-2)
#define GFIO_EREJECT (-1)
int gfio_wr_attr (const char *attr, const char *val);
int gfio_rd_attr (const char *attr, char *buf, size_t len);

// Open the pulse device and take the exclusive flock (the kernel dead
// man's switch). Returns the fd, or -1. The _nb variant fails instead
// of blocking when another process still holds the lock. Under the
// forgectrl device broker (GF_PULSE_FD in the environment) both return
// the inherited fd instead of opening - see glowforge_io.c.
int gfio_open_pulse_dev (const char *path);
int gfio_open_pulse_dev_nb (const char *path);

// True when the pulse device came in from the broker: the device stays
// open across handovers (never close it; the rail never cycled, so no
// settle is needed on takeover).
bool gfio_pulse_inherited (void);

// Factory analog machine config (print-header ground truth), with the XY
// microstep mode in force in place of the factory's x8: mixed-decay mode
// (decay mode 1), every axis in the pulse stream, laser latched out, PIC
// hold currents. The kernel does not restore any of this after a module
// reload.
void gfio_analog_config (void);

// Factory run/idle current scheme: full torque only while motion plays.
void gfio_currents_run (void);
void gfio_currents_hold (void);

// The shared machine config /data/forgefirm/forgefirm.conf ("key = value" lines,
// '#' comments; GFHOME_CONF overrides the path). Written by the
// forgectrl web UI; consumers re-read at their natural boundaries
// (homing per $H, cooling per flood start). Returns 0 when the key
// exists; the float variant falls back on a missing or unparsable key.
int gfio_conf_read (const char *key, char *val, size_t len);
float gfio_conf_read_float (const char *key, float fallback);

// The XY microstep mode in force: xy_microsteps in the shared config (8,
// 16 or 32; unset or anything else reads as x8, logged once). Read at
// the first call and fixed for the session: the DRV8825 MODE pins are
// written from it at the controller's start, at idle, and $100/$101,
// the machine tick and the kernel stop ramp are derived from the same
// number (boards/glowforge.h). A change of the key takes a controller
// restart, which forgectrl does for an idle machine.
unsigned gfio_xy_microsteps (void);

// The derivations, pure, so the wiring and the host test share them.

// The mode a config value names, or 0 when it is not 8, 16 or 32.
static inline unsigned gfio_xy_mode_parse (const char *val)
{
    char *end;
    unsigned long v = strtoul(val, &end, 10);
    if(end == val || *end != '\0')
        return 0;
    return v == 8 || v == 16 || v == 32 ? (unsigned)v : 0;
}

// The mode over x8: 1, 2 or 4.
static inline unsigned gfio_xy_scale_of (unsigned mode)
{
    return mode / XY_MICROSTEPS_DEFAULT;
}

// $100/$101 for a mode, three decimals like the x8 default so a
// settings dump is stable per mode.
static inline float gfio_xy_steps_per_mm_of (unsigned mode)
{
    switch(mode) {
        case 16: return 106.667f;
        case 32: return 213.333f;
        default: return DEFAULT_X_STEPS_PER_MM;
    }
}

// The machine tick for a mode: the factory travel tick at x8, scaled so
// the ticks per step stay the same at every mode.
static inline unsigned gfio_xy_tick_hz_of (unsigned mode)
{
    return GF_TICK_X8_HZ * gfio_xy_scale_of(mode);
}

// The kernel stop ramp (Hz/s of tick frequency) that decelerates over
// the same distance at a given tick as the factory ramp does at its
// tick: the ramp scales with the tick, whatever set the tick.
static inline unsigned gfio_xy_ramp_for_tick (unsigned tick_hz)
{
    unsigned long long ramp = (unsigned long long)GF_RAMP_X8_HZ_PER_S * tick_hz / GF_TICK_X8_HZ;
    if(ramp < 10000)
        ramp = 10000;               /* the kernel's bounds */
    if(ramp > 500000)
        ramp = 500000;
    return (unsigned)ramp;
}

// The fastest feed (mm/min) a stream at tick_hz carries for an axis at
// steps_per_mm: one step per tick per axis.
static inline float gfio_xy_rate_ceiling (unsigned tick_hz, float steps_per_mm)
{
    return (float)tick_hz * 60.0f / steps_per_mm;
}
