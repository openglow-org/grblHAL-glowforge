/*
  glowforge_cooling.c - cooling-service client for the Glowforge board

  Part of grblHAL-glowforge. The cooling engine - fan/pump/TEC/heater
  profiles, the over-temp policy - lives in
  forgectrl (the machine-services daemon); the shared contract is
  https://docs.forgefirm.org/technical/forgefirm/cooling-engine/. This client:

  - reports job state to the engine: POST /cool/state (127.0.0.1,
    FORGECTRL_PORT or 80) with mode=idle|run|cooldown and the armed
    flag, and with the secret the supervisor handed this process at its
    spawn (GF_REPORT_SECRET), which the route asks for: the channel is the
    running controller's alone, and a loopback peer is not proof of that.
    The secret is read once and taken out of the environment, so nothing
    this process starts inherits it but the homing runner, which reports
    in this client's place while a gfcloud $H holds the machine and is
    handed it at its spawn (gfcool_report_secret). Level-triggered, re-sent every ~1 s from gfcool_poll and
    immediately on every change, so a lost report self-heals. The
    effective run window is the sender's M8/M9 OR'd with the laser
    armed window: fire must never run without the cut airflow, which
    only lives inside a run session.

  - reads the engine's verdict from /run/forgefirm/cooling.state every
    500 ms and enforces it in-process on every poll from the cached
    flags: gfcool_verdict_fire_ok() gates the laser per tick and per
    segment (the stepper producer thread reads a cached flag with a
    monotonic freshness deadline - no file IO on that path), the
    window's opening also waits for the engine's acknowledgment
    (gfcool_run_ack, gfcool_fire_ok), a hold verdict takes a real feed hold in the
    same tick the fire gate closes (jogs are canceled instead - grblHAL
    never holds a jog), and resume_ok auto-resumes a hold this client
    took.
    The resume gate refuses the button's and the sender's resume under a
    verdict that blocks fire; a job that leaves the hold under the
    standing verdict all the same is held again within the next poll:
    fire is blocked either way, and a verdict with no resume is a reset,
    never a pause.
    A hold the core does not take stops the job with a reset: a job that
    cannot be held does not run on under the verdict. (The core refuses
    M53 P0, feed hold disabled, in laser mode; this is the backstop.)
    A missing or stale verdict (ts_mono older than 2 s) is treated as
    fire_ok=false, hold=true: the engine being gone must look exactly
    like a fault.

  - the verdict has two tiers inside an open armed window, keyed on its
    name. The fail tier (FIRE, CRASH, AIRFLOW, CRITICAL) ends the job:
    the window closes, the latch locks, the job is reset with ALARM:3,
    and nothing resumes it. Every other fire_ok=false verdict, and a
    stale one, is a pause: this client holds the job and keeps the
    window, and the stream engine masks FIRE on every tick and every
    segment from the same verdict from the moment the head has stopped
    (under a fresh verdict the first deceleration runs lit, in M3 and M4
    alike, as a feed hold's does: the segments already planned at speed
    would otherwise play dark and leave a gap in the cut, while the
    kernel's queue plays its 200 ms of lit bytes either way). A stale
    verdict vouches for nothing: fire is stripped at once and its
    deceleration runs dark. That lit deceleration is bounded: it ends when the head has
    stopped, when the core did not take the hold, or after the longest
    stop the settings allow, whichever comes first. A ~ or a press under
    the standing verdict is refused, the job held. resume_ok with fire_ok
    resumes the hold this client took once the head has stopped, lit
    from the first step, with no new button press while the armed window
    is still open (its grace counts from the verdict; past it the resume
    asks for the button). The pause never writes the latch: a lock sets
    the hardware button latch, which only a press clears.

  - EMERGENCY FALLBACK: if the verdict goes stale while the laser is
    armed, the engine is provably absent and nobody else will move the
    fans - so this client writes the factory run duties directly to
    sysfs once, holds, and stands down. This is the one sanctioned
    exception to the engine's single-writer ownership; the duties are
    compiled in (no config dependency - the same failure may take the
    config file).

  The hardware AND-gate (OK_2_FIRE) remains the safety boundary;
  everything here is equipment protection and defense in depth.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "fflog.h"
#include "glowforge_cooling.h"
#include "glowforge_laser.h"
#include "glowforge_io.h"

#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

/* After the grbl headers: glibc's stat.h (via fcntl.h) defines an
 * st_mtime macro that would otherwise mangle the field of that name in
 * vfs.h. */
#include <fcntl.h>

#define VERDICT_FILE   "/run/forgefirm/cooling.state"
/* The engine's verdict file. In a null-sink run a host harness or a host
 * test moves it (GF_VERDICT_FILE, a test hook) and publishes its own
 * verdicts. With GFSINK set the client reads the engine's file only. */
static const char *verdict_file = VERDICT_FILE;
/* Reader staleness window per the contract: a verdict older than this
 * is no verdict. The engine publishes at 1 Hz. */
#define VERDICT_MAX_AGE_MS 2000
/* Report cadence (level-triggered refresh). */
#define REPORT_PERIOD_MS   1000
/* Socket timeouts for the report POST: localhost, so anything slower
 * than this means forgectrl is wedged - drop the report and move on
 * (the level-triggered refresh retries in a second). */
#define REPORT_TIMEOUT_MS  250

/* Factory run fan duties for the emergency fallback (pulse-header
 * ground truth, mirrored in the contract doc). */
#define FB_AIR_ASSIST "1023"
#define FB_EXHAUST    "65535"
#define FB_INTAKE     "43278"

static coolant_state_t coolant_reported = {0};
/* Armed (glowforge_laser.c). Written on the protocol thread, read from
 * the stepper producer thread through the fire gate, so atomic. */
static _Atomic bool laser_on_window = false;
static bool was_run = false;            /* for the cooldown report */

/* Verdict cache. fire_ok is read from the stepper producer thread; the
 * deadline comparison makes a stalled poll thread read as fire-blocked.
 * Ordering: the flags are published BEFORE the deadline (release), and
 * readers take the deadline first (acquire) - so a reader can never
 * pair a refreshed deadline with the previous, possibly laxer flags. */
static _Atomic bool v_fire_ok = false;
static _Atomic bool v_hold = false;
static _Atomic bool v_resume_ok = false;
/* The engine's acknowledgment of our armed window. A verdict is only an
 * answer to the session it is read against: the window opens here
 * before the engine has seen the report, so an idle verdict - fire_ok
 * with nothing wrong at idle - would otherwise stay fresh across the arm
 * and authorize the first fire before the run airflow is applied. */
static _Atomic bool v_armed = false;
/* The deadline is CLOCK_MONOTONIC milliseconds in 64 bits: it never wraps,
 * so a deadline left behind stays stale however long the controller runs,
 * and the 0 it starts at is stale at any uptime. The stepper producer
 * reads it, at real-time priority, so it must be lock-free there: on the
 * i.MX6 (ARMv7-A) a 64-bit atomic is LDREXD/STREXD. */
static _Atomic uint64_t v_fresh_until_ms = 0;
_Static_assert(__atomic_always_lock_free(sizeof(uint64_t), 0),
               "the verdict deadline is read by a real-time thread: its atomic must be lock-free");
static char v_reason[112];
static char v_reason_shown[112];
static char v_name[16];                 /* the verdict's name (its tier) */
/* The verdict's ack_wait field: a fault waits for the operator's
 * Acknowledge, whatever the verdict's name is. Protocol thread only, as
 * the name is. */
static bool v_ack_wait;
/* The verdict's button_reserved field: a prompt on the control panel has
 * the machine's button (a hold-gated action's dialog, a package's check
 * or a package's ask for a press), so a press is no job's consent.
 * Protocol thread only. */
static bool v_button_reserved;
static bool pause_said;                 /* one message per pause-tier episode */
/* The pause tier's first hold decelerates lit: set when this client
 * takes the hold on a fresh verdict flip under a cut in progress, cleared
 * once the head has stopped (or the job left the hold another way, or the
 * core did not take the hold, or the time bound passed). Read by the laser
 * module's fire gate on the protocol thread; atomic for the same
 * reason the other flags are. */
static _Atomic bool decel_lit = false;
/* The lit deceleration's time bound (CLOCK_MONOTONIC milliseconds), set
 * with decel_lit. Protocol thread only. */
static uint64_t decel_until_ms;
/* The margin on that bound, over the longest stop the settings allow:
 * the poll's pace and the segments already prepared at speed. */
#define DECEL_MARGIN_MS 500

static bool hold_ours = false;
static bool op_held = false;            /* the operator held this cycle: the client claims no hold while it stands */
static bool cycle_ok = false;           /* this cycle has run under a verdict that allowed fire */
static bool hold_asked = false;         /* a feed hold was asked for in this cycle */
static uint8_t hold_misses = 0;         /* requests the core executed with the job still in its cycle */
static bool hold_refused = false;       /* the core did not take the hold: the job is being stopped */
/* How many such requests make a hold the core did not take (hold_take). */
#define HOLD_MISSES_MAX 2
static bool rehold_said = false;        /* one message per resume under a hold */
static bool jog_warned = false;
static bool fallback_done = false;
static bool stale_warned = false;

static bool verdict_fresh (void);

static uint32_t next_report_ms = 0;
static uint32_t next_verdict_ms = 0;
static int http_port = 80;
static char report_secret[33];      /* from the supervisor, at the spawn; empty when there is none */

/* The window's own acknowledgment. Every change of the reported armed bit
 * is a new window, numbered by win_gen (protocol thread), and the change
 * clears the cached acknowledgment. The reporter records the number of the
 * newest report the engine answered (rep_answered_gen). The engine takes
 * the acknowledgment off its file before it answers a report that closes
 * a window or opens one, so a verdict read that begins after the window's
 * own report was answered can carry that window's acknowledgment, and one
 * that began before cannot: verdict_read stores armed as false for it. */
static unsigned win_gen;
static _Atomic unsigned rep_answered_gen;
/* When the engine last answered a report (CLOCK_MONOTONIC ms): a window
 * whose reports go unanswered for REPORT_STALE_MS is ended by the laser
 * module (gfcool_report_stale), before the engine's own silence bound
 * (5 s) lets the extension host read the window closed. */
static _Atomic uint64_t rep_answered_ms;
#define REPORT_STALE_MS 3000
/* A host harness that publishes its own verdicts (the GF_VERDICT_FILE test
 * hook, honored only in a null-sink run) stands in for the engine, and
 * runs no listener for the reports unless a case needs one: there a
 * refused report counts as answered. With GFSINK set, as on a machine,
 * only the engine's 200 answers a report. */
static bool harness_engine;

static uint64_t mono_ms64 (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* The pacing clocks (the next report, the next read) are refreshed every
 * pass, so their 32-bit differences never span 2^31 ms. */
static uint32_t mono_ms (void)
{
    return (uint32_t)mono_ms64();
}

static void warn (const char *msg)
{
    report_message(msg, Message_Warning);
    fflog(LOG_WARNING, "gfcool: %s", msg);
}

/* The longest a feed hold can take to stop the head, from the settings in
 * force, plus the margin: a hold decelerates along the path at the
 * block's acceleration from at most the block's speed, and the planner
 * keeps both inside each moving axis's limits, so the stop is never
 * longer than the largest of the axes' own (highest rate over
 * acceleration; $110-$112 in mm/min over $120-$122, which the core keeps
 * in mm/min^2). The settings dispatch holds the rates at most and the
 * accelerations at least their bounds (gfio_apply_machine_pins), so that
 * is 0.68 s at most; the 60 s cap keeps a nonsense value a bound all the
 * same. */
static uint32_t decel_budget_ms (void)
{
    float worst_s = 0.0f;

    for(uint_fast8_t axis = 0; axis < N_AXIS; axis++) {
        float accel = settings.axis[axis].acceleration;
        float stop_s = accel > 0.0f ? settings.axis[axis].max_rate * 60.0f / accel : 0.0f;
        if(stop_s > worst_s)
            worst_s = stop_s;
    }
    if(!(worst_s < 60.0f))
        worst_s = 60.0f;                /* nan, inf, nonsense: a bound all the same */

    return (uint32_t)(worst_s * 1000.0f) + DECEL_MARGIN_MS;
}

/* Take the feed hold, or take it again. hold_ours already set with the
 * core back in Cycle means the job left the hold under a standing verdict
 * by a way the resume gate does not refuse: fire is blocked either way,
 * so the job would run dark to its end. It is held again, and told why
 * once per resume. A verdict with no resume is a reset, never a pause. */
static void hold_take (bool stale)
{
    /* Called only from a cycle. A request this client made in an earlier
     * poll that the core has since executed, with the job still in its
     * cycle, is a hold the core did not take: feed hold disabled (M53 P0,
     * which the core refuses in laser mode) or a motion cancel in
     * progress. One such request is not yet a refusal: this poll can run
     * inside the core's realtime pass, between its taking the request and
     * its acting on it (a status report written to a full transmit ring
     * pumps the hook), and the request made below is then still pending
     * when the first is acted on. Two are: the core takes the second only
     * in a later pass, after it has acted on the first. A job that cannot
     * be held is not left to run on: the lit deceleration ends here, and
     * the job is stopped the way a ^X stops it (the reset closes the
     * window and locks the latch, the kernel ramps the motion down,
     * ALARM:3). */
    if(hold_refused)
        return;                         /* the reset is on its way */
    if(hold_asked && !(sys.rt_exec_state & EXEC_FEED_HOLD) && ++hold_misses >= HOLD_MISSES_MAX) {
        hold_refused = true;
        decel_lit = false;
        warn(stale ? "cooling service lost and the hold was not taken - job stopped"
                   : "cooling hold: the hold was not taken - job stopped");
        protocol_enqueue_realtime_command(CMD_RESET);
        return;
    }
    hold_asked = true;

    if(hold_ours && !rehold_said && hold_misses == 0) {
        rehold_said = true;
        warn(stale ? "cooling service lost - job held again"
                   : "cooling hold stands - job held again; reset the job");
    }
    /* The first hold of the episode decelerates lit, where a fresh
     * verdict met a cut in progress: this cycle has run under a verdict
     * that allowed fire. A hold taken again under the standing verdict
     * (after a ~ or a press) does not, and neither does the hold of a
     * cycle that started under the verdict (the job sat at Idle, in a
     * dwell or between two lines, or in the sender's own hold, when the
     * verdict turned): there is no cut in progress to finish, and its
     * first move stays dark. Nor does the hold of a stale verdict: with
     * the engine gone nothing vouches for the airflow, so fire is
     * stripped at once and the deceleration runs dark. */
    /* The operator held this cycle (gfcool_operator_hold): the hold is
     * theirs. The client asks for it all the same, and claims nothing: no
     * lit deceleration of its own, no resume at the clear. A poll that runs
     * before the core has acted on either hold (a write to a full transmit
     * ring runs the real-time hook inside the core's own pass) cannot take
     * the hold back. */
    if(op_held) {
        protocol_enqueue_realtime_command(CMD_FEED_HOLD);
        return;
    }
    if(!hold_ours && cycle_ok && !stale) {
        decel_lit = true;
        decel_until_ms = mono_ms64() + decel_budget_ms();
    }
    hold_ours = true;
    protocol_enqueue_realtime_command(CMD_FEED_HOLD);
}

void gfcool_operator_hold (void)
{
    op_held = true;
    if(!hold_ours)
        return;
    hold_ours = false;
    report_message("feed hold - the job stays held when the cooling verdict clears; resume with ~",
                   Message_Info);
    fflog(LOG_INFO, "gfcool: the operator's feed hold - the clean verdict resumes nothing");
}

/* The lit deceleration ends when the head has stopped, when the job has
 * left the cycle and the hold by another way, or when its time bound has
 * passed, whichever comes first: the gate it holds open never waits on a
 * hold that does not complete. A hold the core did not take ends it in
 * hold_take. */
static void decel_track (sys_state_t st)
{
    if(st != STATE_CYCLE) {
        hold_asked = hold_refused = false;
        hold_misses = 0;
        op_held = false;            /* held now, or a cycle no longer: the next cycle is a new episode */
    }
    if(!decel_lit)
        return;
    if(st == STATE_HOLD && sys.holding_state == Hold_Complete)
        decel_lit = false;
    else if(!(st & (STATE_CYCLE | STATE_HOLD)))
        decel_lit = false;
    else if(mono_ms64() >= decel_until_ms)
        decel_lit = false;
}

/* Read on the stepper producer thread too, between two polls: a verdict
 * that goes stale there ends the lit deceleration on its own clock, as
 * it closes gfcool_verdict_fire_ok(), and for good: a reader that finds
 * the verdict stale clears the flag, and so does every poll that finds the
 * cache expired before it reads the file (gfcool_poll), so no verdict read
 * after the expiry relights it, even one the producer saw stale while the
 * read was under way. A new lit deceleration is begun only by hold_take,
 * under a fresh verdict. */
bool gfcool_decel_lit (void)
{
    if(!verdict_fresh()) {
        atomic_store_explicit(&decel_lit, false, memory_order_relaxed);
        return false;
    }
    return atomic_load_explicit(&decel_lit, memory_order_relaxed);
}

/* ------------------------------------------------------- job reports */

/* Whether a report was answered: the engine's status line says 200 (it
 * answers only once it has taken the report, its withdrawal of the
 * acknowledgment included). A refused connection is an answer only to a
 * harness that stands in for the engine with no listener (harness_engine). */
static bool report_answered (bool refused, const char *resp, ssize_t n)
{
    if(refused)
        return harness_engine;
    return n >= 12 && !strncmp(resp, "HTTP/1.", 7) && !strncmp(resp + 8, " 200", 4);
}

/* One POST, on the reporter thread only. localhost:
 * connection refused returns immediately when forgectrl is down. The
 * connect is non-blocking and bounded by poll - SO_SNDTIMEO does not
 * cover connect(), and a wedged listener (full accept backlog) blocks
 * a plain connect through minutes of SYN retries. True when the engine
 * answered it (report_answered). */
static bool report_send (const char *mode, bool armed, bool density)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if(fd < 0)
        return false;

    struct timeval tv = { 0, REPORT_TIMEOUT_MS * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)http_port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    bool up = connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0;
    bool refused = !up && errno == ECONNREFUSED;
    bool answered = false;
    if(!up && errno == EINPROGRESS) {
        struct pollfd pfd = { .fd = fd, .events = POLLOUT };
        int err = 0;
        socklen_t el = sizeof(err);
        up = poll(&pfd, 1, REPORT_TIMEOUT_MS) == 1 &&
              getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el) == 0 && err == 0;
        refused = !up && err == ECONNREFUSED;
    }
    if(refused)
        answered = report_answered(true, NULL, 0);
    if(up) {
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
        char req[288], who[64] = "";
        if(report_secret[0])
            snprintf(who, sizeof(who), "X-ForgeFIRM-Report: %s\r\n", report_secret);
        int n = snprintf(req, sizeof(req),
            "POST /cool/state?mode=%s&armed=%d&model=%s HTTP/1.1\r\n"
            "Host: 127.0.0.1\r\nConnection: close\r\n%s"
            "Content-Length: 0\r\n\r\n", mode, armed ? 1 : 0,
            density ? "density" : "analog", who);
        if(n > 0 && n < (int)sizeof(req) && write(fd, req, (size_t)n) == n) {
            char resp[128];
            ssize_t got = read(fd, resp, sizeof(resp));     /* the engine's answer */
            answered = report_answered(false, resp, got);
        }
    }
    close(fd);
    return answered;
}

/* Reporter thread: report_send can still block for the socket timeouts,
 * which is far past the ~10 ms the protocol thread can afford inside a
 * cycle (a stalled protocol thread starves the segment lookahead while
 * the shipper keeps cutting). Level-triggered hand-off: the protocol
 * thread deposits the latest mode+armed and kicks; intermediate states
 * may be skipped, only the newest matters. */
static pthread_mutex_t rep_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t rep_cv = PTHREAD_COND_INITIALIZER;
static char rep_mode[12] = "idle";
static bool rep_armed = false;
static bool rep_density = true;     /* the dose model in force at the report */
static unsigned rep_gen = 0;        /* the window the report speaks for (win_gen) */
static unsigned rep_seq = 0;

static void *reporter_thread (void *arg)
{
    (void)arg;

    unsigned seen = 0, gen;
    char mode[12];
    bool armed, density;

    pthread_mutex_lock(&rep_mx);
    for(;;) {
        while(rep_seq == seen)
            pthread_cond_wait(&rep_cv, &rep_mx);
        seen = rep_seq;
        strcpy(mode, rep_mode);
        armed = rep_armed;
        density = rep_density;
        gen = rep_gen;
        pthread_mutex_unlock(&rep_mx);

        if(report_send(mode, armed, density)) {
            atomic_store(&rep_answered_gen, gen);
            atomic_store(&rep_answered_ms, mono_ms64());
        }

        pthread_mutex_lock(&rep_mx);
    }

    return NULL;
}

static const char *report_mode (void)
{
    if(coolant_reported.flood || laser_on_window)
        return "run";
    /* After a run: the engine owns the cooldown phases; "cooldown"
     * tells it the job just ended vs. a machine that was never
     * running. Cleared once grbl is back at idle. */
    if(was_run && (state_get() & (STATE_CYCLE | STATE_HOLD)))
        return "cooldown";
    return "idle";
}

static void report_now (void)
{
    const char *mode = report_mode();
    if(!strcmp(mode, "idle"))
        was_run = false;

    pthread_mutex_lock(&rep_mx);
    strncpy(rep_mode, mode, sizeof(rep_mode) - 1);
    rep_mode[sizeof(rep_mode) - 1] = '\0';
    rep_armed = laser_on_window;
    rep_density = gflaser_density();
    rep_gen = win_gen;
    rep_seq++;
    pthread_cond_signal(&rep_cv);
    pthread_mutex_unlock(&rep_mx);

    next_report_ms = mono_ms() + REPORT_PERIOD_MS;
}

/* ---------------------------------------------------------- verdict */

/* Minimal field extraction from the engine's fixed JSON (we own the
 * writer; keys are matched, order is not assumed). A key that is absent
 * takes the caller's default, which is always the fail-safe direction:
 * fire blocked, hold on, resume refused. */
static bool json_bool (const char *body, const char *key, bool dflt)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(body, pat);
    if(!p)
        return dflt;
    p += strlen(pat);
    if(!strncmp(p, "true", 4))
        return true;
    if(!strncmp(p, "false", 5))
        return false;
    return dflt;
}

static bool json_num (const char *body, const char *key, double *out)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(body, pat);
    if(!p)
        return false;
    *out = atof(p + strlen(pat));
    return true;
}

static void json_str (const char *body, const char *key, char *out, size_t len)
{
    char pat[24];
    out[0] = '\0';
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(body, pat);
    if(!p)
        return;
    p += strlen(pat);
    const char *e = strchr(p, '"');
    size_t n = e ? (size_t)(e - p) : 0;
    if(n >= len)
        n = len - 1;
    memcpy(out, p, n);
    out[n] = '\0';
}

/* Read + parse the verdict file; refresh the cached flags. A missing
 * or unparsable file, or a stale ts_mono, leaves the cache to expire:
 * fire blocked, hold on. */
static void verdict_read (void)
{
    char body[1024];
    /* Taken before the file is opened: a read that began before the
     * window's own report was answered carries no acknowledgment for it.
     * The window changes on this thread only, so it cannot change under
     * the read. */
    bool own_window = atomic_load(&rep_answered_gen) == win_gen;
    FILE *f = fopen(verdict_file, "r");
    if(!f)
        return;
    size_t n = fread(body, 1, sizeof(body) - 1, f);
    fclose(f);
    body[n] = '\0';

    /* Only a complete document is trusted: the publisher writes the
     * whole object and renames it into place, so a body without its
     * closing brace is a torn or oversized read, not a verdict. */
    if(n == 0 || memchr(body, '}', n) == NULL)
        return;

    double ts_mono;
    if(!json_num(body, "ts_mono", &ts_mono))
        return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double age = (double)ts.tv_sec + ts.tv_nsec / 1e9 - ts_mono;
    if(age < 0.0 || age > VERDICT_MAX_AGE_MS / 1000.0)
        return;     /* stale publisher; let the cache expire */

    /* Flags first, deadline last (release): see the cache comment. */
    atomic_store_explicit(&v_hold, json_bool(body, "hold", true), memory_order_relaxed);
    atomic_store_explicit(&v_resume_ok, json_bool(body, "resume_ok", false), memory_order_relaxed);
    atomic_store_explicit(&v_fire_ok, json_bool(body, "fire_ok", false), memory_order_relaxed);
    atomic_store_explicit(&v_armed, own_window && json_bool(body, "armed", false), memory_order_relaxed);
    json_str(body, "reason", v_reason, sizeof(v_reason));
    json_str(body, "verdict", v_name, sizeof(v_name));
    /* Absent reads as false, as ack_wait does: fire_ok carries the block. */
    v_button_reserved = json_bool(body, "button_reserved", false);
    /* Absent reads as false: the field gates nothing by itself (fire_ok
     * carries the block), and a verdict named ACK says the same thing. */
    v_ack_wait = json_bool(body, "ack_wait", false);
    atomic_store_explicit(&v_fresh_until_ms,
        mono_ms64() + (uint64_t)(VERDICT_MAX_AGE_MS - (uint32_t)(age * 1000.0)),
        memory_order_release);
}

static bool verdict_fresh (void)
{
    return mono_ms64() < atomic_load_explicit(&v_fresh_until_ms, memory_order_acquire);
}

/* The fail tier: the verdicts a job cannot continue from. Everything
 * else with fire_ok=false, and a stale verdict, is a pause. */
static bool verdict_fail_tier (const char *name)
{
    return !strcmp(name, "FIRE") || !strcmp(name, "CRASH") || !strcmp(name, "EMISSION") ||
           !strcmp(name, "AIRFLOW") || !strcmp(name, "CRITICAL");
}

/* Stale verdict while the laser is armed: the engine is gone with the
 * laser hot. Gate + hold happen through the normal paths (freshness);
 * the fans need the one direct write nobody else can make now. */
static void fallback_fans (void)
{
    gfio_wr_attr("head/air_assist_pwm", FB_AIR_ASSIST);
    gfio_wr_attr("thermal/exhaust_pwm", FB_EXHAUST);
    gfio_wr_attr("thermal/intake_pwm", FB_INTAKE);
}

/* ---------------------------------------------------------------- api */

void gfcool_init (void)
{
    const char *v = getenv("FORGECTRL_PORT");
    if(v && atoi(v) > 0 && atoi(v) < 65536)
        http_port = atoi(v);
    const char *vf = gfio_test_hook("GF_VERDICT_FILE");
    harness_engine = vf && *vf;
    if(harness_engine)
        verdict_file = vf;
    atomic_store(&rep_answered_ms, mono_ms64());
    /* The report secret: 32 hex digits or nothing (it goes into a header
     * as it is). Out of the environment once read: nothing this process
     * starts inherits it, and the homing runner, which reports in this
     * client's place, is handed it at its spawn. */
    const char *rs = getenv("GF_REPORT_SECRET");
    if(rs && strlen(rs) == sizeof(report_secret) - 1 &&
       strspn(rs, "0123456789abcdef") == sizeof(report_secret) - 1)
        strcpy(report_secret, rs);
    unsetenv("GF_REPORT_SECRET");
    next_report_ms = mono_ms();
    next_verdict_ms = mono_ms();

    /* The reporter runs detached for the process lifetime. If it cannot
     * start, reports never send and the engine treats the silence as a
     * stand-down - the failure direction is safe. */
    pthread_t tid;
    pthread_attr_t a;
    pthread_attr_init(&a);
    pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
    if(pthread_create(&tid, &a, reporter_thread, NULL) != 0)
        fflog(LOG_ERR, "gfcool: cannot start the reporter thread");
    pthread_attr_destroy(&a);
}

const char *gfcool_report_secret (void)
{
    return report_secret;
}

void gfcool_coolant_set (coolant_state_t state)
{
    bool was = coolant_reported.flood || laser_on_window;
    coolant_reported = state;
    if((state.flood || laser_on_window) != was) {
        if(!was)
            was_run = true;
        report_now();
    }
}

coolant_state_t gfcool_coolant_get (void)
{
    return coolant_reported;
}

void gfcool_laser_armed (bool armed)
{
    if(laser_on_window == armed)
        return;
    bool was = coolant_reported.flood || laser_on_window;
    laser_on_window = armed;
    /* A new window: the acknowledgment read for the last one is not its
     * own, and only a verdict read begun after this window's report was
     * answered can give it one (verdict_read). */
    win_gen++;
    atomic_store_explicit(&v_armed, false, memory_order_relaxed);
    if(!was && (coolant_reported.flood || armed))
        was_run = true;
    /* Every change of the armed bit is reported at once, whatever the
     * sender's M8/M9 state: the engine's crash watch and its
     * acknowledgment both follow the reported bit. */
    report_now();
}

bool gfcool_verdict_fire_ok (void)
{
    return verdict_fresh() &&
           atomic_load_explicit(&v_fire_ok, memory_order_relaxed);
}

bool gfcool_fire_ok (void)
{
    /* Freshness (acquire) first: the flags read below are then at least
     * as new as the verdict that set the deadline. */
    if(!gfcool_verdict_fire_ok())
        return false;

    /* The window's opening: once the window is reported the verdict
     * must be one the engine computed with that window in effect, or it
     * is an answer to the previous session. Before it is reported there
     * is nothing to acknowledge, so the pre-arm gate reads the verdict as
     * it stands. The engine re-reports at 1 Hz and on every change, so
     * the wait here is bounded by one report and one engine tick. The
     * per-tick and per-segment gates inside the open window read the
     * verdict alone (gfcool_verdict_fire_ok). */
    if(atomic_load_explicit(&laser_on_window, memory_order_relaxed) &&
        !atomic_load_explicit(&v_armed, memory_order_relaxed))
        return false;

    return true;
}

bool gfcool_ack_waits (void)
{
    return verdict_fresh() && (v_ack_wait || !strcmp(v_name, "ACK"));
}

bool gfcool_button_reserved (void)
{
    return verdict_fresh() && v_button_reserved;
}

bool gfcool_run_ack (void)
{
    return verdict_fresh() &&
            atomic_load_explicit(&v_armed, memory_order_relaxed);
}

bool gfcool_report_stale (void)
{
    return mono_ms64() - atomic_load(&rep_answered_ms) > REPORT_STALE_MS;
}

bool gfcool_hold_stands (void)
{
    return verdict_fresh() && !verdict_fail_tier(v_name) &&
           (!atomic_load_explicit(&v_fire_ok, memory_order_relaxed) || gfcool_ack_waits());
}

void gfcool_poll (void)
{
    uint32_t now = mono_ms();
    bool read_now = (int32_t)(now - next_verdict_ms) >= 0;

    /* The file is read at half the staleness window, so no gap opens
     * between reads. The verdict is enforced on EVERY poll from the
     * cached flags: a cache that expires between two reads closes the
     * fire gate on its own clock, and the hold must land in that same
     * tick, or the head cuts dark until the next read. A cache that
     * expired since the last poll ends a lit deceleration before the read,
     * so the read that brings back a fresh, still blocking verdict does
     * not relight it. */
    if(!verdict_fresh())
        decel_lit = false;
    if(read_now) {
        next_verdict_ms = now + 500;
        verdict_read();
    }

    {
        bool fresh = verdict_fresh();
        sys_state_t st = state_get();
        bool blocked = !gfcool_verdict_fire_ok();
        bool armed = gflaser_armed();

        /* Whether this cycle has run under a verdict that allowed fire
         * (hold_take: only such a cycle's hold decelerates lit). */
        if(st != STATE_CYCLE)
            cycle_ok = false;
        else if(!blocked)
            cycle_ok = true;

        /* The tiers, inside an open window. The fail tier ends the job;
         * the pause tier (a named pause, or a stale verdict) is the hold
         * below with the stream's fire gate masking the beam, said once
         * per episode. */
        if(armed && blocked) {
            if(fresh && verdict_fail_tier(v_name))
                gflaser_verdict_fail(v_name);
            else if(!pause_said) {
                pause_said = true;
                warn("cooling verdict blocks fire - fire masked, job held");
            }
        } else {
            pause_said = false;
            decel_lit = false;
        }
        decel_track(st);

        if(!fresh) {
            /* No verdict: fire is blocked (freshness), motion holds,
             * and an armed window gets the fallback airflow (a lit
             * deceleration ended at the top of this poll). */
            if(laser_on_window || coolant_reported.flood) {
                if(!stale_warned) {
                    stale_warned = true;
                    warn("cooling service lost - fire blocked, holding");
                    /* The engine only runs the check heater inside this
                     * window; if it died mid-check the heater is still
                     * on, and like the fallback fans this is a write
                     * nobody else will make now. */
                    gfio_wr_attr("thermal/heater_pwm", "0");
                }
                if(st == STATE_CYCLE)
                    hold_take(true);
                else if(st & STATE_HOLD)
                    rehold_said = false;
                if(laser_on_window && !fallback_done) {
                    fallback_done = true;
                    fallback_fans();
                }
            }
        } else {
            if(stale_warned) {
                stale_warned = false;
                fallback_done = false;
                report_message("cooling service restored", Message_Info);
                fflog(LOG_INFO, "gfcool: cooling service restored");
            }

            /* Relay the engine's reason to the sender once per change. */
            if(read_now) {
                if(v_reason[0] && strcmp(v_reason, v_reason_shown))
                    warn(v_reason);
                strcpy(v_reason_shown, v_reason);
            }

            /* A verdict that blocks fire inside the window holds the
             * job whether or not it asks for the hold itself. */
            if(v_hold || (armed && blocked)) {
                if(st == STATE_CYCLE) {
                    hold_take(false);
                } else if(st & STATE_HOLD) {
                    rehold_said = false;
                } else if(st == STATE_JOG) {
                    /* grblHAL never holds a jog; cancel it instead. */
                    protocol_enqueue_realtime_command(CMD_JOG_CANCEL);
                    if(!jog_warned) {
                        jog_warned = true;
                        warn("jog canceled on the cooling verdict");
                    }
                }
            } else {
                jog_warned = false;
                rehold_said = false;
                if(hold_ours) {
                    if(!(st & STATE_HOLD))
                        hold_ours = false;  /* operator resumed or reset */
                    /* Not while a fault waits for its Acknowledge: the job
                     * stays held until the panel's Acknowledge, then resumes
                     * through the re-arm's press, instead of a resume the
                     * resume gate would refuse, the hold no longer this
                     * client's to resume. Nor before the head has stopped:
                     * the core drops a cycle start while the hold still
                     * decelerates, so the hold stays this client's until
                     * it completes, and is resumed then. */
                    else if(v_resume_ok && !gfcool_ack_waits() &&
                            sys.holding_state == Hold_Complete) {
                        hold_ours = false;
                        if(!gflaser_resume_gate()) {
                            report_message("cooling verdict clean - resuming",
                                           Message_Info);
                            fflog(LOG_INFO, "gfcool: resuming");
                            protocol_enqueue_realtime_command(CMD_CYCLE_START);
                        }
                    }
                }
            }
        }
    }

    /* One reporter per machine (the cooling-engine contract): a gfcloud homing session
     * hands the machine to gfhome, which reports its own job state -
     * this client's idle reports would fight it, resetting the engine's
     * per-job profile and thrashing the flood state per motion. */
    if((int32_t)(now - next_report_ms) >= 0 && state_get() != STATE_HOMING)
        report_now();
}
