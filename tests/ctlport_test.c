/*
  ctlport_test.c - host unit test for the controller port's outstanding line

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  The port injects one line at a time and reads nothing while its status
  is outstanding (ctlport.c). This test includes the port source, stands
  in for the stream's multiplexer (serial.c) and the core, and drives
  ctlport_poll() over the port's real Unix socket, so each step lands in
  the poll the test chooses: no timing decides an order.

  - An open envelope (the bed check's) holds the sender at the poll after
    it opened, before any port line is injected. A sender line that comes
    while a port line is outstanding, which the envelope block does not
    look at then, stays held when the port line's status comes back, so
    the core does not read it inside the open envelope; the next poll
    closes the envelope before it lets the line go.
  - A jog whose client hangs up before the jog's status comes back is
    canceled as it starts: the jog is a gone client's, and its dead-man
    is gone with it. The client that connected meanwhile gets no reply.
*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "ctlport.h"
#include "serial.h"
#include "glowforge_homing.h"
#include "glowforge_io.h"
#include "glowforge_laser.h"
#include "glowforge_mcode.h"
#include "glowforge_release.h"
#include "glowforge_tray.h"
#include "stepper_stream.h"
#include "grbl/hal.h"
#include "grbl/planner.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

/* --- the stream's multiplexer, as the port sees it (serial.h) ---------- */

static serial_inject_done_ptr inj_done_cb;      /* the port's status hook */
static char injected[200];                      /* the line handed over, "" when none */
static bool sender_held;                        /* serial_hold_sender() */
static bool sender_bytes;                       /* a sender line waits in the ring */

void serial_inject_init (serial_inject_done_ptr done) { inj_done_cb = done; }
bool serial_inject_line (const char *line)
{
    if(injected[0] || sender_bytes)
        return false;
    snprintf(injected, sizeof(injected), "%s", line);
    return true;
}
void serial_hold_sender (bool hold) { sender_held = hold; }
bool serial_sender_pending (void) { return sender_bytes; }
bool serial_sender_empty_lines_only (void) { return false; }
double serial_sender_last_line (void) { return 0.0; }
bool serial_sender_midline (void) { return false; }
bool serial_sender_line_open (void) { return false; }
bool serial_client_connected (void) { return true; }
void serial_keep_out (bool on) { (void)on; }
bool serial_keeping_out (void) { return false; }
bool serial_drop_sender (void) { return false; }

/* --- the rest of the driver ---------------------------------------------- */

static bool envelope_open;
int gfhome_envelope (bool open) { envelope_open = open; return 0; }
bool gfhome_envelope_is_open (void) { return envelope_open; }
void gfhome_envelope_close (void) { envelope_open = false; }
int gfio_conf_read (const char *key, char *val, size_t len) { (void)key; (void)val; (void)len; return -1; }
bool gflaser_armed (void) { return false; }
bool gflaser_arming (void) { return false; }
int gfmcode_set_table (const char *list) { (void)list; return 0; }
int gfmcode_answer (unsigned seq, bool ok, const char *text) { (void)seq; (void)ok; (void)text; return 0; }
bool gfmcode_waiting (void) { return false; }
void gfmcode_state_json (char *buf, size_t len) { snprintf(buf, len, "null"); }
bool gfrelease_active (void) { return false; }
bool gftray_out (void) { return false; }
bool gf_stream_kernel_idle (void) { return true; }

/* --- the core ------------------------------------------------------------ */

grbl_hal_t hal;
system_t sys;
settings_t settings;
static sys_state_t core_state = STATE_IDLE;
sys_state_t state_get (void) { return core_state; }
plan_block_t *plan_get_current_block (void) { return NULL; }
static void set_bits (volatile uint_fast16_t *value, uint_fast16_t bits) { *value |= bits; }
static void stream_write (const char *s) { (void)s; }

/* --- port source under test ------------------------------------------ */
#include "../src/ctlport.c"

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static char sock_path[108];

static int port_connect (void)
{
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", sock_path);
    if(fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
        return -1;
    ctlport_poll();                     /* the port accepts it */
    return fd;
}

static void port_send (int fd, const char *req)
{
    (void)!write(fd, req, strlen(req));
    ctlport_poll();
}

/* What the port has written to a client so far; "" when nothing. */
static const char *port_reply (int fd)
{
    static char buf[200];
    ssize_t n = recv(fd, buf, sizeof(buf) - 1, MSG_DONTWAIT);
    buf[n > 0 ? n : 0] = '\0';
    return buf;
}

/* The core reads the injected line, runs it and reports its status. */
static void core_runs_line (int status)
{
    injected[0] = '\0';
    inj_done_cb(status);
}

int main (void)
{
    char dir[] = "/tmp/ctlport_test.XXXXXX";
    if(!mkdtemp(dir)) {
        printf("FAIL: cannot create the state directory\n");
        return 1;
    }
    unsetenv("GFSINK");
    setenv("GF_STATE_DIR", dir, 1);
    snprintf(sock_path, sizeof(sock_path), "%s/" SOCK_NAME, dir);
    hal.stream.write = stream_write;
    hal.set_bits_atomic = set_bits;
    ctlport_init();
    if(port_listen_fd < 0 || !inj_done_cb) {
        printf("FAIL: the port did not start\n");
        return 1;
    }

    printf("A sender line behind an outstanding port line, inside an open envelope:\n");
    int c = port_connect();
    port_send(c, "envelope open\n");
    CHECK(envelope_open && !strcmp(port_reply(c), "ok\n"), "the envelope opens");
    ctlport_poll();
    CHECK(sender_held, "the poll after it holds the sender");
    port_send(c, "energize\n");
    CHECK(!strcmp(injected, "$ME"), "a port line is injected and outstanding");
    sender_bytes = true;                /* the sender's line comes behind it */
    ctlport_poll();
    core_runs_line(Status_OK);
    CHECK(sender_held && envelope_open,
          "when the port line's status is back the sender's line is still held: the core "
          "does not read it inside the open envelope");
    CHECK(!strcmp(port_reply(c), "ok\n"), "the port line is answered");
    ctlport_poll();
    CHECK(!envelope_open && !sender_held, "the next poll closes the envelope, then lets the line go");
    sender_bytes = false;
    close(c);
    ctlport_poll();

    printf("A jog whose client hangs up before its status:\n");
    int a = port_connect();
    port_send(a, "jog X5 F100\n");
    CHECK(!strcmp(injected, "$J=X5 F100"), "the jog is injected and outstanding");
    close(a);
    int b = port_connect();             /* the next client: the poll finds the hang-up */
    core_state = STATE_JOG;             /* the core plans the jog and starts it before its status */
    sys.rt_exec_state = 0;
    core_runs_line(Status_OK);
    CHECK(sys.rt_exec_state & EXEC_MOTION_CANCEL, "the gone client's jog is canceled as it starts");
    CHECK(!strcmp(port_reply(b), ""), "the client that connected meanwhile gets no reply");
    core_state = STATE_IDLE;
    ctlport_poll();
    close(b);

    unlink(sock_path);
    rmdir(dir);
    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: a sender line behind an outstanding port line waits out the open envelope, "
                      "and a gone client's jog is canceled as it starts\n", failures);
    return failures ? 1 : 0;
}
