/*
  glowforge_mcode.h - M-codes an extension package answers (see glowforge_mcode.c)

  Part of grblHAL-glowforge.
  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include <stdbool.h>
#include <stddef.h>

// The numbers a package may answer, and how many at once.
#define GFMCODE_MIN 160
#define GFMCODE_MAX 179
#define GFMCODE_TABLE_MAX 20      // the whole range: each number is one package's

// How long a job waits for the answer. It stays under laser_disarm_s's
// default, so the armed window of a job waiting at one does not close.
#define GFMCODE_WAIT_S 30.0

// How long the last move before it may take to finish playing, after the
// core has handed its steps to the stream, before the M-code is announced.
#define GFMCODE_DRAIN_S 5.0

// The most an answer's words may be: they go to the sender in a [MSG:].
#define GFMCODE_TEXT_MAX 96

// Chains the core's user M-code hooks. From driver_init(), after
// gflaser_init() (M102 is the laser's, and stays so).
void gfmcode_init (void);

// The numbers something answers now, from the machine daemon: "-" for none,
// or up to GFMCODE_TABLE_MAX numbers in range, comma separated. 0, or -1
// for a list with no such form (the table is left as it was).
int gfmcode_set_table (const char *list);

// The answer to the M-code the job waits at: seq is the one state names,
// ok says whether it did its work, text is its words (printable, no
// brackets, at most GFMCODE_TEXT_MAX). 0, or -1 when no M-code waits under
// that seq or the text has no such form.
int gfmcode_answer (unsigned seq, bool ok, const char *text);

// True while a job waits at an M-code: the port jogs nothing meanwhile.
bool gfmcode_waiting (void);

// The M-code the job waits at, as JSON for the port's state: null, or
// {"seq":n,"code":160,"words":{"P":1}}.
void gfmcode_state_json (char *buf, size_t len);
