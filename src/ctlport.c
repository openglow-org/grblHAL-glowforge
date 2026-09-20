/*
  ctlport.c - the controller port: a local, non-displacing second channel

  Part of grblHAL-glowforge. The Grbl socket is one session, and a new
  connection displaces the sender. The port is how the machine daemon jogs
  and reads state while a sender stays connected: a Unix stream socket,
  mode 0600, one client, served from the protocol thread's poll set. It
  adds no thread and no lock.

  Requests and replies are single lines:

    state            {"state":"Idle","sender":true,"port_jog":false,"released":false,
                      "mpos":[x,y,z],"homed":3}
    jog <words>      ok | error:<n> | busy:<why>     <words> is the tail of a $J= line
    cancel           ok

  and, for the machine daemon's own panel only (no package capability maps
  to them; the daemon decides which set a request may use before anything
  else sees it):

    release          ok | error:<n> | busy:<why>     $MD (glowforge_release.c)
    energize         ok | error:<n> | busy:<why>     $ME
    home             ok | error:<n> | busy:<why>     $H, only while homing_mode = manual:
                                                     it moves nothing. Every other $H is a
                                                     session that belongs to the sender.

  The port's only motion is the core's jog, $J=. A jog ships dark whatever
  the modal spindle state is, because the stream masks FIRE while the core
  is jogging; no other motion has that property, so the port never forms a
  line that does not start with $J=, and <words> is held to the characters
  a jog needs.

  The sender always wins. A port jog is refused while the sender is active,
  and a sender line that arrives while a port jog runs cancels the jog and
  waits, unread, until the core is idle again: the sender sees a short
  delay and its own status, never the error a line gets during a jog. The
  cancel is the motion-cancel flag, not the jog-cancel command: the core
  answers that command by flushing the input ring, which would discard the
  sender's waiting line without a status.

  An empty line is not the sender being active. LightBurn polls '?' with an
  end of line behind it about twice a second: the '?' is a realtime
  character, and the end of line is an empty line the core answers ok in
  any state. It neither refuses a port jog nor cancels one, and it is
  answered during a port jog like at any other time (serial.c).

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE     /* accept4 */
#endif

#include "ctlport.h"
#include "serial.h"
#include "glowforge_io.h"
#include "glowforge_release.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define STATE_DIR_DEFAULT "/run/forgefirm"
#define SOCK_NAME "grbl.ctl"

/* A port jog waits this long after the sender's last line. */
#define SENDER_QUIET_S 0.3

/* A line that found the sender's empty lines in the ring waits this long
 * for the core to read them, which takes it one pass of its loop. */
#define DEFER_S 0.1

static int port_listen_fd = -1;
static int port_fd = -1;
static char req[160];
static size_t req_len = 0;
static bool awaiting = false;       /* a line is injected, its status not yet back */
static bool awaiting_jog = false;   /* and that line is a jog */
static bool port_jog = false;       /* the jog in progress is the port's */
static bool cancel_sent = false;
static char deferred[sizeof(req) + 4];  /* a line waiting for the ring to empty, "" when none */
static bool deferred_jog;
static double deferred_until;

static double now_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void cancel_port_jog (void)
{
    if(port_jog && !cancel_sent && state_get() == STATE_JOG) {
        system_set_exec_state_flag(EXEC_MOTION_CANCEL);
        cancel_sent = true;
    }
}

static void drop_port_client (void)
{
    if(port_fd >= 0) {
        close(port_fd);
        port_fd = -1;
        req_len = 0;
        deferred[0] = '\0';
        cancel_port_jog();          /* the client is the dead-man */
    }
}

static void reply (const char *line)
{
    if(port_fd >= 0) {
        size_t len = strlen(line);
        if(write(port_fd, line, len) != (ssize_t)len)
            drop_port_client();     /* replies are one short line: a client that cannot take one is gone */
    }
}

/* The injected line's status, from the stream's status hook. */
static void line_done (int status)
{
    char buf[24];

    awaiting = false;
    if(status == Status_OK && !awaiting_jog)
        reply("ok\n");
    else if(status == Status_OK) {
        if(!port_jog)
            hal.stream.write("[MSG:Panel jog]" ASCII_EOL);
        port_jog = true;
        cancel_sent = false;
        serial_hold_sender(true);
        reply("ok\n");
    } else if(status < 0)
        reply("error:aborted\n");
    else {
        snprintf(buf, sizeof(buf), "error:%d\n", status);
        reply(buf);
    }
}

/* Hands a line to the multiplexer. The ring must be empty for that, and
 * the sender's empty lines, which the core is about to read, are no
 * reason to refuse: the line waits for them. */
static void inject (const char *line, bool jog)
{
    if(serial_inject_line(line)) {
        awaiting = true;
        awaiting_jog = jog;
    } else if(serial_sender_empty_lines_only()) {
        snprintf(deferred, sizeof(deferred), "%s", line);
        deferred_jog = jog;
        deferred_until = now_s() + DEFER_S;
    } else
        reply("busy:sender\n");     /* the sender is in the middle of a line */
}

static const char *state_name (void)
{
    switch(state_get()) {
        case STATE_IDLE:        return "Idle";
        case STATE_CYCLE:       return "Run";
        case STATE_HOLD:        return "Hold";
        case STATE_JOG:         return "Jog";
        case STATE_HOMING:      return "Home";
        case STATE_ALARM:       return "Alarm";
        case STATE_ESTOP:       return "Alarm";
        case STATE_CHECK_MODE:  return "Check";
        case STATE_SAFETY_DOOR: return "Door";
        case STATE_SLEEP:       return "Sleep";
        case STATE_TOOL_CHANGE: return "Tool";
        default:                return "Unknown";
    }
}

static void op_state (void)
{
    char buf[200];
    float mpos[3] = {0};

    for(int i = 0; i < 3 && i < N_AXIS; i++)
        mpos[i] = (float)sys.position[i] / settings.axis[i].steps_per_mm;

    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"sender\":%s,\"port_jog\":%s,\"released\":%s,"
             "\"mpos\":[%.3f,%.3f,%.3f],\"homed\":%u}\n",
             state_name(), serial_client_connected() ? "true" : "false",
             port_jog ? "true" : "false", gfrelease_active() ? "true" : "false",
             mpos[0], mpos[1], mpos[2], (unsigned)sys.homed.mask);
    reply(buf);
}

static void op_jog (const char *words)
{
    char line[sizeof(req) + 4];

    if(*words == '\0') {
        reply("error:invalid\n");
        return;
    }
    for(const char *p = words; *p; p++) {
        if(!((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9') ||
             *p == '.' || *p == '-' || *p == '+' || *p == ' ')) {
            reply("error:invalid\n");
            return;
        }
    }

    if(gfrelease_active()) {
        reply("busy:released\n");   /* the core would refuse it too; this says why */
        return;
    }

    sys_state_t state = state_get();
    if(!(state == STATE_IDLE || (state == STATE_JOG && port_jog))) {
        reply("busy:state\n");      /* a program, a hold, an alarm, or the sender's own jog */
        return;
    }
    if(serial_sender_pending() || (serial_sender_last_line() > 0 &&
                                   now_s() - serial_sender_last_line() < SENDER_QUIET_S)) {
        reply("busy:sender\n");
        return;
    }

    snprintf(line, sizeof(line), "$J=%s", words);
    inject(line, true);
}

/* The panel-only operations: a $ command that moves nothing. */
static void op_command (const char *line)
{
    sys_state_t state = state_get();

    if(!(state == STATE_IDLE || state == STATE_ALARM)) {
        reply("busy:state\n");
        return;
    }
    if(serial_sender_pending()) {
        reply("busy:sender\n");
        return;
    }
    inject(line, false);
}

static void op_home (void)
{
    char mode[24] = "";

    gfio_conf_read("homing_mode", mode, sizeof(mode));
    if(strcmp(mode, "manual"))
        reply("error:mode\n");      /* a homing session is the sender's to start */
    else
        op_command("$H");
}

static void handle_request (char *line)
{
    if(!strcmp(line, "state"))
        op_state();
    else if(!strncmp(line, "jog ", 4))
        op_jog(line + 4);
    else if(!strcmp(line, "cancel")) {
        cancel_port_jog();
        reply("ok\n");
    } else if(!strcmp(line, "release"))
        op_command("$MD");
    else if(!strcmp(line, "energize"))
        op_command("$ME");
    else if(!strcmp(line, "home"))
        op_home();
    else
        reply("error:unknown\n");
}

void ctlport_poll (void)
{
    if(port_listen_fd < 0)
        return;

    /* The port jog ends when the core leaves the jog state, and the
     * sender's held bytes go to the core then. A sender byte that arrives
     * before that cancels the jog: the sender always wins. */
    if(port_jog && !awaiting) {
        if(state_get() != STATE_JOG) {
            port_jog = false;
            serial_hold_sender(false);
        } else if(serial_sender_pending())
            cancel_port_jog();
    }

    /* One client. A second connection is refused, never displacing. A
     * client that closes and reconnects at once is not a second client,
     * though: its hang-up is looked for before the new connection is
     * judged, or the reconnect would lose to its own corpse. */
    int fd = accept4(port_listen_fd, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if(fd >= 0) {
        char peek;
        if(port_fd >= 0 && recv(port_fd, &peek, 1, MSG_PEEK | MSG_DONTWAIT) == 0)
            drop_port_client();
        if(port_fd >= 0)
            close(fd);
        else
            port_fd = fd;
    }

    if(deferred[0]) {
        if(serial_inject_line(deferred)) {
            awaiting = true;
            awaiting_jog = deferred_jog;
            deferred[0] = '\0';
        } else if(serial_sender_pending() || now_s() > deferred_until) {
            deferred[0] = '\0';
            reply("busy:sender\n");
        }
    }

    /* One request at a time: nothing is read while a line's status is
     * outstanding or a line waits to be injected, so replies keep the
     * order of the requests. */
    while(port_fd >= 0 && !awaiting && !deferred[0]) {
        char c;
        ssize_t n = read(port_fd, &c, 1);
        if(n == 1) {
            if(c == '\n') {
                req[req_len] = '\0';
                req_len = 0;
                handle_request(req);
            } else if(c != '\r') {
                if(req_len < sizeof(req) - 1)
                    req[req_len++] = c;
                else {
                    drop_port_client();     /* no request is this long */
                }
            }
        } else if(n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            drop_port_client();
        } else
            break;
    }
}

unsigned ctlport_pollfds (struct pollfd *fds)
{
    unsigned n = 0;

    if(port_listen_fd >= 0) {
        fds[n].fd = port_listen_fd;
        fds[n].events = POLLIN;
        n++;
        if(port_fd >= 0 && !awaiting && !deferred[0]) {
            fds[n].fd = port_fd;
            fds[n].events = POLLIN;
            n++;
        }
    }

    return n;
}

void ctlport_init (void)
{
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    const char *dir = getenv("GF_STATE_DIR");

    if(snprintf(sa.sun_path, sizeof(sa.sun_path), "%s/" SOCK_NAME,
                dir && *dir ? dir : STATE_DIR_DEFAULT) >= (int)sizeof(sa.sun_path))
        return;

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if(fd < 0)
        return;

    /* 0600 from the first instant: on Linux the bound name takes the mode
     * of the socket, so it is set before the bind and never through the
     * process-wide umask, which the stream threads share. */
    unlink(sa.sun_path);
    if(fchmod(fd, 0600) != 0 ||
        bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0 || listen(fd, 1) != 0) {
        close(fd);
        return;
    }

    port_listen_fd = fd;
    serial_inject_init(line_done);
}
