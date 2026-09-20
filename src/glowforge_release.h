/*
  glowforge_release.h - X and Y motor release (see glowforge_release.c)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "grbl/errors.h"

// Take over a release a controller before this one left standing (the
// marker in the state directory): a controller that starts over a released
// gantry must not energize it. Called first in driver_init(), before the
// stream engine writes its first hold current.
void gfrelease_adopt (void);

// Register $MD, $ME and the $X guard. Called from driver_init().
void gfrelease_init (void);

// Keeps the released machine locked. Called from the realtime hook.
void gfrelease_poll (void);

// True while X and Y are released: nothing may move.
bool gfrelease_active (void);

// Energize X and Y if they are released. Status_OK when they are (or
// already were) energized. The position stays invalid: the caller is a
// home, or the operator's $ME.
status_code_t gfrelease_energize (void);

// Tell the sender why a request was refused (rate limited).
void gfrelease_report (void);
