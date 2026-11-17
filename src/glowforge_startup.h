/*
  glowforge_startup.h - the startup lines a sender stores ($N0, $N1)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>

// The core runs the stored startup lines at every controller start and
// every soft reset, with no sender connected and X and Y unreferenced,
// before the lens takes its reference. A startup line may therefore hold
// modal-state words only: the plane (G17, G18, G19), the units (G20,
// G21), the work coordinate system (G54 to G59, G59.1 to G59.3), the
// distance modes (G90, G91, and G91.1; the core refuses G90.1) and the
// feed mode (G93, G94).
// Nothing else is a startup line here: no motion, no axis word, no
// spindle, no offset write, no M-code.
//
// True when every word of every block of `line` (blocks split on '|')
// is one of those, in a G word with an optional decimal part, either
// case, with spaces, tabs and comments, "( )" and a ';' to the end,
// between words. An empty line passes: it clears the slot.
bool gfstartup_line_ok (const char *line);

// Shadows the core's $N0 and $N1: a line that is not modal-state words
// only is refused before the core runs it (the core validates a line by
// executing it, in Idle and in Alarm), and the core stores the rest. Called
// from driver_init().
void gfstartup_init (void);

// A stored line that holds more than modal state, whatever wrote it to the
// settings store (the store survives an update), is cleared before the core
// first runs the startup lines, and the log says what was cleared. Called
// from the driver's setup, after the settings load and before the protocol
// loop's first start.
void gfstartup_check (void);
