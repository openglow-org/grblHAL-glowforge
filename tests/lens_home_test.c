/*
  lens_home_test.c - host unit test for the Z a home sets and the park after it

  Part of grblHAL-glowforge. The gfcloud homing path (src/glowforge_homing.h
  helpers) leaves the lens on the hall sensor's rising edge, whose focal
  height above the tray the focus card measured (lens_hall_edge_z_mm), and
  parks it at lens_park_z_mm. The controller counts whole steps, so:

    - a height is placed on the step grid first (gfhome_z_steps,
      gfhome_grid_z), and the step count the controller stores for the
      grid height reads back as that count: the stored position, the
      home position, and the reported Z agree
    - the park is a whole number of half-steps from the edge's grid step
      to the park height's (gfhome_park_steps), kept inside the window
      every head reaches without touching a stop

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "glowforge_homing.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SPM 2.922f      /* the lens screw: 36 half-steps over 12.32 mm */
#define DOWN 10
#define UP 12

static int failures = 0;

/* The grid height for `z_mm` must be `steps` over the scale and store
 * back as `steps`. */
static void expect_grid (float z_mm, long steps)
{
    float z = gfhome_grid_z(z_mm, SPM);
    float want = (float)steps / SPM;
    long stored = lroundf(z * SPM);

    if (fabsf(z - want) > 1e-5f || stored != steps || gfhome_z_steps(z_mm, SPM) != steps) {
        printf("FAIL Z %.3f: grid %.4f (want %.4f, %ld steps), stored %ld\n",
               z_mm, z, want, steps, stored);
        failures++;
    } else {
        printf("ok   Z %.3f: grid %.4f = %ld steps\n", z_mm, z, steps);
    }
}

/* The park from edge to park height must be `want` half-steps. */
static void expect_park (float edge_z, float park_z, int want)
{
    int k = gfhome_park_steps(edge_z, park_z, SPM, DOWN, UP);
    if (k != want) {
        printf("FAIL edge Z %.2f park Z %.2f: %d half-steps (want %d)\n", edge_z, park_z, k, want);
        failures++;
    } else {
        printf("ok   edge Z %.2f park Z %.2f: %+d half-steps\n", edge_z, park_z, k);
    }
}

int main (void)
{
    /* The bench reference machine: the edge at Z 3.35 is 9.8 half-steps,
     * ten on the grid, Z 3.4223. */
    expect_grid(3.35f, 10);
    expect_grid(0.0f, 0);
    expect_grid(3.0f, 9);
    expect_grid(-1.0f, -3);
    expect_grid(7.5f, 22);

    /* The default park (Z 3.0, nine steps) from the reference edge (ten
     * steps): one half-step down. */
    expect_park(3.35f, 3.0f, -1);
    expect_park(3.35f, 3.35f, 0);
    expect_park(3.35f, 5.0f, 5);
    expect_park(0.5f, 3.0f, 8);
    /* Beyond the window the park is clamped: the lens never meets a stop
     * on a home. */
    expect_park(3.35f, 12.0f, UP);
    expect_park(3.35f, -5.0f, -DOWN);
    expect_park(8.0f, 3.0f, -DOWN);

    /* Every height over the settings' range stores the count it was
     * placed on. */
    {
        long checked = 0, disagreed = 0;
        for (int hundredths = -2000; hundredths <= 2000; hundredths++) {
            float z = (float)hundredths / 100.0f;
            long want = gfhome_z_steps(z, SPM);
            if (lroundf(gfhome_grid_z(z, SPM) * SPM) != want)
                disagreed++;
            checked++;
        }
        if (disagreed) {
            printf("FAIL %ld of %ld heights store a count other than their grid step\n",
                   disagreed, checked);
            failures++;
        } else {
            printf("ok   %ld heights store their grid step\n", checked);
        }
    }

    /* Every park over both settings' ranges stays inside the window. */
    {
        long checked = 0, outside = 0;
        for (int e = -200; e <= 200; e++) {
            for (int p = -50; p <= 100; p++) {
                int k = gfhome_park_steps((float)e / 10.0f, (float)p / 10.0f, SPM, DOWN, UP);
                if (k < -DOWN || k > UP)
                    outside++;
                checked++;
            }
        }
        if (outside) {
            printf("FAIL %ld of %ld parks leave the window\n", outside, checked);
            failures++;
        } else {
            printf("ok   %ld parks stay inside the window\n", checked);
        }
    }

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS lens_home_test\n");
    return EXIT_SUCCESS;
}
