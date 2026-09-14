/*
  switches_test.c - host unit test for the lid cancel under a failed switch read

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  The switch poll keeps the previous state when a read fails, so a
  transient failure never fakes an edge. A cancel already in flight must
  not wait on that read: its reset and its park go on. This test includes
  the switch source, drives it through the file-backed test source, and
  proves that a cancel waiting for the kernel to drain sends its reset
  on a poll whose switch read fails, using the stream engine's reading
  of the kernel (idle without a device).
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

static int  resets_requested;
static int  cycle_starts;
static bool kernel_idle_val = true;
static char last_message[128];

bool gf_stream_kernel_idle (void) { return kernel_idle_val; }
void gflaser_disarm (void) {}
bool gflaser_arming (void) { return false; }
bool gflaser_resume_gate (void) { return false; }
int  gfio_conf_read (const char *k, char *v, size_t n) { (void)k; (void)v; (void)n; return -1; }
void fflog (int prio, const char *fmt, ...) { (void)prio; (void)fmt; }

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_switches.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
grbl_hal_t hal;
grbl_t grbl;
system_t sys;
static sys_state_t cur_state = STATE_IDLE;
sys_state_t state_get (void) { return cur_state; }
plan_block_t *plan_get_current_block (void) { return NULL; }
void system_convert_array_steps_to_mpos (float *pos, int32_t *steps)
{ for(int i = 0; i < N_AXIS; i++) pos[i] = (float)steps[i]; }
bool protocol_enqueue_realtime_command (uint8_t c)
{
    if(c == CMD_RESET) resets_requested++;
    if(c == CMD_CYCLE_START) cycle_starts++;
    return true;
}
void report_message (const char *msg, message_type_t type)
{ (void)type; snprintf(last_message, sizeof(last_message), "%s", msg ? msg : ""); }
static void control_cb (control_signals_t s) { (void)s; }
static int parks_enqueued;
static bool enqueue_gcode (char *cmd) { (void)cmd; parks_enqueued++; return true; }

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void write_word (const char *path, unsigned word)
{
    FILE *f = fopen(path, "w");
    if(f) {
        fprintf(f, "%u\n", word);
        fclose(f);
    }
}

int main (void)
{
    char path[] = "/tmp/switches_test.XXXXXX";
    int fd = mkstemp(path);
    if(fd < 0) {
        printf("FAIL: cannot create the switch file\n");
        return 1;
    }
    close(fd);
    write_word(path, 1u << SW_BIT_DOORS);           /* lid closed, loop closed, button up */
    setenv("GF_SWITCH_FILE", path, 1);
    hal.control.interrupt_callback = control_cb;

    gfsw_init();
    CHECK(gfsw_available(), "the file-backed switch source is available");

    printf("a cancel in flight under a failed switch read:\n");

    /* The job is canceled and waits for the kernel to drain the hold's
       tail; the kernel is idle (the stream engine's reading), so the
       next poll must send the reset. The switch file is gone by then. */
    cancel_state = Cancel_WaitDrain;
    door_hidden = true;
    park_at = wall_s();
    cur_state = STATE_SAFETY_DOOR;
    kernel_idle_val = true;
    unlink(path);                                   /* the read fails from here */
    uint8_t sw[SW_BYTES];
    CHECK(!gfsw_read_raw(sw), "the switch read fails with the file gone");
    gfsw_poll();
    CHECK(resets_requested == 1, "the cancel sent its reset on the poll whose switch read failed");
    CHECK(cancel_state == Cancel_ResetSent, "the cancel moved on to waiting for the reset");

    /* A failed read fakes no edge: the state the core sees is the last
       good one. */
    CHECK(gfsw_get_state().safety_door_ajar == Off, "the failed read left the door state as it was");

    /* The reset lands, the core is idle, and the park is enqueued: the
       cancel keeps running without a single successful read. */
    cur_state = STATE_IDLE;
    have_job_start = true;
    grbl.enqueue_gcode = enqueue_gcode;
    gfsw_poll();
    CHECK(parks_enqueued == 1 && cancel_state == Cancel_ParkQueued,
          "the return to the job start is enqueued on a poll whose switch read failed");

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: a lid cancel in flight reaches its reset and its park even when the "
                      "switch device stops answering\n", failures);
    return failures ? 1 : 0;
}
