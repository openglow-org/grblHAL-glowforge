/*
  switches_test.c - host unit test for the lid cancel under a failed switch read

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  The switch poll keeps the previous state for one or two failed reads
  in a row, so a transient failure never fakes an edge, and reads the
  door open from the third: the window closes, once, the core is shown
  the door where it shows a real one, and a read that succeeds ends it.
  A first read that fails at startup reads open too. A device that keeps
  failing is opened again once a second, and only a device that answers
  to the switches' name replaces it (open and ioctl are faked for that
  case). A cancel already in flight must not wait on a failed read: its
  reset and its park go on. This test includes the switch source,
  drives it through the file-backed test source, and proves that a
  cancel waiting for the kernel to drain sends its reset on a poll whose
  switch read fails, using the stream engine's reading of the kernel
  (idle without a device). The file is a switch source only in a
  null-sink run: with GFSINK set and no switch device, there is no
  switch source, and the laser arm has no button to read.

  A lid or loop that opens while the laser's window is open closes the
  window, at Idle too; a cancel is pending until its reset has landed.

  And a press in a hold that the laser module's resume gate takes: it
  says the job re-arms before it resumes only when the gate started a
  re-arm.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
/* The core's headers before glibc's fcntl.h, as the source orders them:
   its st_mtime macro would mangle the field of that name in vfs.h. */
#include "fflog.h"
#include "glowforge_switches.h"
#include "glowforge_switch_map.h"
#include "glowforge_io.h"
#include "glowforge_laser.h"
#include "stepper_stream.h"
#include "grbl/hal.h"
#include "grbl/planner.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include <fcntl.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <linux/input.h>

/* --- controllable + observable stubs (defined before the include so the
   driver source links against them) ------------------------------------ */

static int  resets_requested;
static int  cycle_starts;
static int  feed_holds;
static bool kernel_idle_val = true;
static char last_message[128];
static int  disarms;
static int  log_errs;               /* lines logged at LOG_ERR */

bool gf_stream_kernel_idle (void) { return kernel_idle_val; }
static bool armed_val;                  /* gflaser_armed(): the laser's window is open */
void gflaser_disarm (void) { disarms++; armed_val = false; }
bool gflaser_armed (void) { return armed_val; }

/* The switch device, faked for the reopen case: open() of FAKE_DEV gives a
   descriptor, EVIOCGNAME answers dev_name, and EVIOCGSW answers dev_word or
   fails while dev_reads_fail is set. Any other path does not open. */
#define FAKE_DEV "fake-switch-device"
static int  dev_opens, dev_name_checks;
static bool dev_reads_fail;
static unsigned dev_word;
static const char *dev_name = "switches";
static int test_open (const char *path, int flags, ...)
{
    (void)flags;
    if(strcmp(path, FAKE_DEV)) {
        errno = ENOENT;
        return -1;
    }
    dev_opens++;
    return dup(STDERR_FILENO);
}
static int test_ioctl (int fd, unsigned long req, ...)
{
    va_list ap;
    va_start(ap, req);
    char *buf = va_arg(ap, char *);
    va_end(ap);
    (void)fd;
    if(_IOC_NR(req) == _IOC_NR(EVIOCGNAME(0))) {
        dev_name_checks++;
        snprintf(buf, _IOC_SIZE(req), "%s", dev_name);
        return 0;
    }
    if(_IOC_NR(req) == _IOC_NR(EVIOCGSW(0))) {
        if(dev_reads_fail) {
            errno = ENODEV;
            return -1;
        }
        memset(buf, 0, _IOC_SIZE(req));
        buf[0] = (char)(dev_word & 0xff);
        buf[1] = (char)(dev_word >> 8);
        return 0;
    }
    errno = EINVAL;
    return -1;
}
#define open test_open
#define ioctl test_ioctl
/* The laser module's resume gate: whether it takes the resume
   (gate_takes), and whether taking it started a re-arm (gate_rearms),
   which gflaser_arming() then reports. */
static bool gate_takes, gate_rearms, arming_val;
bool gflaser_arming (void) { return arming_val; }
bool gflaser_resume_gate (void) { arming_val = gate_takes && gate_rearms; return gate_takes; }
static int operator_holds;              /* gfcool_operator_hold(): the operator's pause */
void gfcool_operator_hold (void) { operator_holds++; }
int  gfio_conf_read (const char *k, char *v, size_t n) { (void)k; (void)v; (void)n; return -1; }
void fflog (int prio, const char *fmt, ...) { if(prio == LOG_ERR) log_errs++; (void)fmt; }

/* --- driver source under test ---------------------------------------- */
#include "../src/glowforge_switches.c"
#undef open
#undef ioctl

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
    if(c == CMD_FEED_HOLD) feed_holds++;
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

    printf("the switch file that GF_SWITCH_FILE names, honored only in a null-sink run:\n");

    /* On a machine the supervisor sets GFSINK: with no switch device the
       file is no switch source. */
    setenv("GFSINK", "/dev/glowforge", 1);
    gfsw_init();
    CHECK(!gfsw_available(), "with GFSINK set, the switch file is no switch source");
    unsetenv("GFSINK");
    grbl.on_state_change = NULL;                    /* the next init chains afresh */

    gfsw_init();
    CHECK(gfsw_available(), "the file-backed switch source is available");

    printf("a press in a cycle is the operator's pause:\n");

    /* A press in a cycle is a pause, and the operator's: a cooling hold
       asked for in the same pass becomes theirs, and the verdict that
       clears resumes nothing. */
    feed_holds = operator_holds = 0;
    button_edge(STATE_CYCLE);
    CHECK(feed_holds == 1 && operator_holds == 1,
          "a press in a cycle holds the job, and the hold is the operator's");
    feed_holds = 0;

    printf("a press in a hold that the laser module's resume gate takes:\n");

    /* The gate took the resume and refused it (its own words say why):
       the press adds no words and no cycle start. The gate took it and
       started a re-arm: the press says the job re-arms first. */
    gate_takes = true;
    gate_rearms = false;
    last_message[0] = '\0';
    operator_holds = 0;
    button_edge(STATE_HOLD);
    CHECK(last_message[0] == '\0' && cycle_starts == 0,
          "a resume the gate refused gets no re-arm notice and no cycle start");
    CHECK(operator_holds == 0, "a press in a hold is a resume, never the operator's pause: the cooling hold stays the client's");
    gate_rearms = true;
    arming_val = false;
    button_edge(STATE_HOLD);
    CHECK(strstr(last_message, "re-arms before it resumes") != NULL && cycle_starts == 0,
          "a resume the gate re-arms says the job re-arms before it resumes");
    gate_takes = gate_rearms = arming_val = false;

    printf("a lid or loop that opens closes an open laser window, at Idle too:\n");

    /* The lid sets the hardware button latch, which only a press clears;
       the loop is the lockout engaged. Either closes the window, once; with
       the window closed an opening closes nothing. */
    cur_state = STATE_IDLE;
    {
        int d0 = disarms;
        armed_val = true;
        write_word(path, 0);                        /* the lid open */
        gfsw_poll();
        CHECK(disarms == d0 + 1 && strstr(last_message, "lid opened - the laser window closed") != NULL,
              "the lid opening closes the open window");
        write_word(path, 1u << SW_BIT_DOORS);
        gfsw_poll();
        armed_val = true;
        write_word(path, (1u << SW_BIT_DOORS) | (1u << SW_BIT_INTERLOCK));     /* the loop open, the lid closed */
        gfsw_poll();
        CHECK(disarms == d0 + 2 && strstr(last_message, "interlock open - the laser window closed") != NULL,
              "the loop opening closes the open window");
        armed_val = false;
        write_word(path, 0);
        gfsw_poll();
        CHECK(disarms == d0 + 2, "with the window closed an opening closes nothing");
        write_word(path, 1u << SW_BIT_DOORS);
        gfsw_poll();
        disarms = d0;                               /* the cases below count from here */
    }

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
    CHECK(gfsw_cancel_pending(), "a cancel waiting for its reset is still pending (a resume is dropped)");

    /* The reset lands, the core is idle, and the park is enqueued: the
       cancel keeps running without a single successful read. */
    cur_state = STATE_IDLE;
    have_job_start = true;
    grbl.enqueue_gcode = enqueue_gcode;
    gfsw_poll();
    CHECK(parks_enqueued == 1 && cancel_state == Cancel_ParkQueued,
          "the return to the job start is enqueued on a poll whose switch read failed");
    CHECK(!gfsw_cancel_pending(), "the return to the job start is no pending cancel");

    printf("a switch device that stops answering:\n");

    /* Two failed polls in a row kept the last reading (the lid closed)
       and closed nothing; the third reads the door open, closes the
       window and says so, once. The cancel above still hides the door
       from the core; in a job-time state the core is shown it. */
    CHECK(state.safety_door_ajar == Off && disarms == 0 && log_errs == 0,
          "two failed polls keep the last reading and close nothing");
    gfsw_poll();
    CHECK(state.safety_door_ajar == On && disarms == 1 && log_errs == 1,
          "the third failed poll reads the door open, closes the window, logs once");
    gfsw_poll();
    CHECK(disarms == 1 && log_errs == 1, "a fourth failed poll closes and logs nothing more");
    cancel_state = Cancel_None;
    door_hidden = false;
    cur_state = STATE_CYCLE;
    gfsw_poll();
    CHECK(gfsw_get_state().safety_door_ajar == On && delivered.safety_door_ajar == On,
          "a running job is shown the door, as a real one");
    write_word(path, 1u << SW_BIT_DOORS);           /* the switches read again, lid closed */
    gfsw_poll();
    CHECK(state.safety_door_ajar == Off && !device_lost && poll_fails == 0,
          "a read that succeeds ends it: the lid reads closed again");

    /* A failed read is no press: a button held across a lost device gives
       no press edge when the device reads again. */
    cur_state = STATE_IDLE;
    write_word(path, (1u << SW_BIT_DOORS) | (1u << SW_BIT_BUTTON));
    gfsw_poll();                                    /* held at idle, where a press means nothing */
    unlink(path);
    gfsw_poll();
    gfsw_poll();
    gfsw_poll();                                    /* lost */
    write_word(path, (1u << SW_BIT_DOORS) | (1u << SW_BIT_BUTTON));
    cur_state = STATE_CYCLE;
    feed_holds = 0;
    gfsw_poll();
    CHECK(!device_lost && feed_holds == 0,
          "a button held across a lost device is no press when the device reads again");
    write_word(path, 1u << SW_BIT_DOORS);
    gfsw_poll();

    /* A first read that fails at startup reads open, never closed, and
       counts as the first failed read: two failed polls after it lose the
       device. */
    unlink(path);
    grbl.on_state_change = NULL;
    gfsw_init();
    CHECK(state.safety_door_ajar == On, "a failed first read reads the lid open");
    int disarms0 = disarms;
    gfsw_poll();
    CHECK(!device_lost, "one failed poll after it keeps the open reading");
    gfsw_poll();
    CHECK(device_lost && disarms == disarms0 + 1, "the second loses the device: it counted as the first");
    write_word(path, 1u << SW_BIT_DOORS);
    gfsw_poll();
    CHECK(state.safety_door_ajar == Off, "and the first read that succeeds reads it closed");

    /* The device itself: opened again once a second while its reads fail,
       and replaced only by a device that answers to the switches' name. */
    unsetenv("GF_SWITCH_FILE");
    setenv("GF_SWITCH_DEV", FAKE_DEV, 1);
    grbl.on_state_change = NULL;
    fake_path = NULL;
    dev_word = 1u << SW_BIT_DOORS;
    gfsw_init();
    int first_fd = sw_fd;
    CHECK(first_fd >= 0 && dev_opens == 1 && dev_name_checks == 1 && state.safety_door_ajar == Off,
          "the device opens with its name checked and reads the lid closed");
    dev_reads_fail = true;
    gfsw_poll();
    gfsw_poll();
    CHECK(dev_opens == 1, "two failed polls open nothing");
    gfsw_poll();
    CHECK(dev_opens == 2 && dev_name_checks == 2 && sw_fd >= 0 && sw_fd != first_fd,
          "the third opens the device again, name checked, and takes it");
    int second_fd = sw_fd;
    gfsw_poll();
    CHECK(dev_opens == 2, "no second open inside the second");
    reopen_at = 0.0;                                /* the second has passed */
    dev_name = "other";
    gfsw_poll();
    CHECK(dev_opens == 3 && sw_fd == second_fd, "a device of another name is not taken");
    dev_reads_fail = false;
    gfsw_poll();
    CHECK(state.safety_door_ajar == Off && !device_lost, "the device reading again: the lid reads closed");
    unlink(path);

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: a failed switch read reads the lid open from the third poll, and a lid "
                      "cancel in flight reaches its reset and its park even when the switch device "
                      "stops answering\n", failures);
    return failures ? 1 : 0;
}
