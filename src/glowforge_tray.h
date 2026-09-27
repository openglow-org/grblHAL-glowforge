/*
  glowforge_tray.h - the crumb tray, in or out (see glowforge_tray.c)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <math.h>
#include <stdbool.h>

// The tray offset (tray_offset_mm): how far the floor of the cut area lies
// below the tray. The default is 1.35", and the setting is held to 13 to
// 60 mm: above the lens's travel, so the Z ranges of the two modes never
// overlap. A value out of range lands on the nearer end; one that is not
// a number takes the default.
#define GFTRAY_OFFSET_DEFAULT_MM 34.29f
#define GFTRAY_OFFSET_MIN_MM     13.0f
#define GFTRAY_OFFSET_MAX_MM     60.0f

static inline float gftray_clamp_offset_mm (float mm)
{
    if(!(mm == mm))
        return GFTRAY_OFFSET_DEFAULT_MM;
    return mm < GFTRAY_OFFSET_MIN_MM ? GFTRAY_OFFSET_MIN_MM
         : mm > GFTRAY_OFFSET_MAX_MM ? GFTRAY_OFFSET_MAX_MM : mm;
}

// The offset in whole lens half-steps (z_spm is $102).
static inline long gftray_offset_steps (float mm, float z_spm)
{
    return lroundf(gftray_clamp_offset_mm(mm) * z_spm);
}

// Reads the persisted mode (the marker beside the shared config) and
// chains M103 into the core's user M-code hooks. From driver_init().
void gftray_init (void);

// True while the tray is out.
bool gftray_out (void);

// The Z shift of the mode in force, in half-steps: 0 with the tray in, the
// offset on the step grid with it out. Every Z referenced from the hall
// edge (the start's lens reference, a camera home) adds it.
long gftray_shift_steps (void);
