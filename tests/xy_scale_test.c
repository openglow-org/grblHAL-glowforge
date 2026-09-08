/*
  xy_scale_test.c - host unit test for the XY microstep scale

  Part of grblHAL-glowforge. The XY microstep mode (xy_microsteps in the
  shared config: 8, 16 or 32) is one number three coupled quantities are
  derived from, never typed (src/glowforge_io.h helpers):

    - $100/$101: 0.15 mm per full step, three decimals per mode
    - the machine tick: the factory travel tick at x8, scaled with the
      mode so the ticks per step and the top speed stay the same
    - the kernel stop ramp: Hz/s of tick frequency, scaled with the tick
      in force so a controlled stop covers the same distance, held to
      the kernel's bounds
    - the feed ceiling the stream carries: one step per tick per axis,
      which is what holds $110/$111 down when the bench lowers the tick

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "glowforge_io.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;

static void expect_parse (const char *val, unsigned want)
{
    unsigned got = gfio_xy_mode_parse(val);
    if (got != want) {
        printf("FAIL parse '%s': %u (want %u)\n", val, got, want);
        failures++;
    } else {
        printf("ok   parse '%s' = %u\n", val, got);
    }
}

static void expect_u (const char *what, unsigned got, unsigned want)
{
    if (got != want) {
        printf("FAIL %s: %u (want %u)\n", what, got, want);
        failures++;
    } else {
        printf("ok   %s = %u\n", what, got);
    }
}

static void expect_f (const char *what, float got, float want, float tol)
{
    if (fabsf(got - want) > tol) {
        printf("FAIL %s: %.4f (want %.4f)\n", what, got, want);
        failures++;
    } else {
        printf("ok   %s = %.3f\n", what, got);
    }
}

int main (void)
{
    /* The key names a mode or nothing: no rounding, no prefixes. */
    expect_parse("8", 8);
    expect_parse("16", 16);
    expect_parse("32", 32);
    expect_parse("1", 0);
    expect_parse("24", 0);
    expect_parse("64", 0);
    expect_parse("160", 0);
    expect_parse("16x", 0);
    expect_parse("", 0);
    expect_parse("abc", 0);

    /* $100/$101: the x8 literal is the board default; every mode is
     * 0.15 mm per full step to three decimals. */
    expect_f("$100 at x8", gfio_xy_steps_per_mm_of(8), 53.333f, 1e-4f);
    expect_f("$100 at x16", gfio_xy_steps_per_mm_of(16), 106.667f, 1e-4f);
    expect_f("$100 at x32", gfio_xy_steps_per_mm_of(32), 213.333f, 1e-4f);
    for (unsigned mode = 8; mode <= 32; mode *= 2)
        expect_f("steps/mm against 0.15 mm per full step", gfio_xy_steps_per_mm_of(mode),
                 (float)mode / XY_MM_PER_FULL_STEP, 1e-3f);

    /* The tick keeps the ticks per step at the factory's. */
    expect_u("scale at x8", gfio_xy_scale_of(8), 1);
    expect_u("scale at x16", gfio_xy_scale_of(16), 2);
    expect_u("scale at x32", gfio_xy_scale_of(32), 4);
    expect_u("tick at x8", gfio_xy_tick_hz_of(8), 28160);
    expect_u("tick at x16", gfio_xy_tick_hz_of(16), 56320);
    expect_u("tick at x32", gfio_xy_tick_hz_of(32), 112640);

    /* The ramp follows the tick, whatever set it, inside the kernel's
     * bounds: 32 at the full tick lands exactly on the maximum, and the
     * 3x fallback tick sits between. */
    expect_u("ramp at 28160 Hz", gfio_xy_ramp_for_tick(28160), 125000);
    expect_u("ramp at 56320 Hz", gfio_xy_ramp_for_tick(56320), 250000);
    expect_u("ramp at 112640 Hz", gfio_xy_ramp_for_tick(112640), 500000);
    expect_u("ramp at 84480 Hz", gfio_xy_ramp_for_tick(84480), 375000);
    expect_u("ramp at 1000 Hz (floor)", gfio_xy_ramp_for_tick(1000), 10000);
    expect_u("ramp at 165000 Hz (ceiling)", gfio_xy_ramp_for_tick(165000), 500000);

    /* The feed ceiling: the same 31680 mm/min at every mode on its own
     * tick, a quarter of that for x32 held at the x8 tick. */
    expect_f("ceiling x8 at 28160 Hz", gfio_xy_rate_ceiling(28160, gfio_xy_steps_per_mm_of(8)), 31680.0f, 0.5f);
    expect_f("ceiling x32 at 112640 Hz", gfio_xy_rate_ceiling(112640, gfio_xy_steps_per_mm_of(32)), 31680.0f, 0.5f);
    expect_f("ceiling x32 at 28160 Hz", gfio_xy_rate_ceiling(28160, gfio_xy_steps_per_mm_of(32)), 7920.0f, 0.5f);
    expect_f("ceiling x16 at 1000 Hz", gfio_xy_rate_ceiling(1000, gfio_xy_steps_per_mm_of(16)), 562.5f, 0.05f);

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS xy_scale_test\n");
    return EXIT_SUCCESS;
}
