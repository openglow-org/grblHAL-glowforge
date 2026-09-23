/*
  glowforge_mcode.c - M-codes an extension package answers

  Part of grblHAL-glowforge. M160 to M179 belong to extension packages: a
  package that holds mcode:<n> answers M<n> in a job. The machine daemon
  tells this controller which numbers something answers now (the port's
  mcodes op), and a number nothing answers is an unsupported command where
  the line is parsed, as any other would be, so a job never reaches one
  that cannot be answered.

  An M-code a package answers is a synchronized barrier. The core drains
  the planner before it runs the M-code, which means every step has been
  handed to the stream, not that the head has stopped: the kernel is still
  playing the last move's tail out of its ring. So the M-code waits for the
  kernel to go idle before it is announced, and from then on the head is
  still and the stream carries no fire: the laser is dark for as long as
  the job waits, by the stream's own rule for a starved planner. A move
  that does not finish within GFMCODE_DRAIN_S holds the job instead. The
  M-code is then put in the port's state; the daemon hands it to the
  package and brings the answer back (the port's mcode_result op).
  Throughout, the protocol is pumped as a dwell pumps it: the status
  reports, a feed hold, the door, and a soft reset all work, and a reset
  or an alarm ends the wait at once.

  The wait is bounded by GFMCODE_WAIT_S, which stays under laser_disarm_s's
  default so that a job's armed window does not close under it. An answer
  that says the work was done lets the job go on. One that says it was not,
  or no answer in time, holds the job with a [MSG:] saying why: the operator
  resumes it (and the job goes on without what the M-code was for) or stops
  it. Nothing here writes the laser or moves the head.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "glowforge_mcode.h"
#include "fflog.h"
#include "stepper_stream.h"

#include "grbl/hal.h"
#include "grbl/gcode.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static user_mcode_ptrs_t chained;

static int table[GFMCODE_TABLE_MAX];
static int table_n;

static struct {
    bool active;                /* a job waits at an M-code */
    bool answered;
    bool ok;
    unsigned seq;
    int code;
    bool has_p, has_q, has_r;
    float p, q, r;
    char text[GFMCODE_TEXT_MAX + 1];
} wait;
static unsigned seq_next = 1;

static double now_s (void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static bool ours (int code)
{
    return code >= GFMCODE_MIN && code <= GFMCODE_MAX;
}

static bool answered_now (int code)
{
    for(int i = 0; i < table_n; i++)
        if(table[i] == code)
            return true;
    return false;
}

int gfmcode_set_table (const char *list)
{
    int codes[GFMCODE_TABLE_MAX], n = 0;

    if(!list)
        return -1;
    if(strcmp(list, "-")) {
        const char *p = list;
        while(*p) {
            if(n == GFMCODE_TABLE_MAX || *p < '0' || *p > '9')
                return -1;
            char *end;
            long v = strtol(p, &end, 10);
            if(end - p > 3 || !ours((int)v))
                return -1;
            for(int i = 0; i < n; i++)
                if(codes[i] == v)
                    return -1;
            codes[n++] = (int)v;
            if(*end == ',' && end[1])
                p = end + 1;
            else if(*end)
                return -1;
            else
                p = end;
        }
        if(n == 0)
            return -1;
    }
    bool same = n == table_n && !memcmp(codes, table, sizeof(int) * (size_t)n);
    memcpy(table, codes, sizeof(int) * (size_t)n);
    table_n = n;
    if(!same)
        fflog(LOG_INFO, "mcode: answered now: %s", n ? list : "none");
    return 0;
}

int gfmcode_answer (unsigned seq, bool ok, const char *text)
{
    if(!wait.active || wait.answered || seq != wait.seq)
        return -1;
    size_t len = text ? strlen(text) : 0;
    if(len > GFMCODE_TEXT_MAX)
        return -1;
    for(size_t i = 0; i < len; i++)
        if(text[i] < 0x20 || text[i] > 0x7e || text[i] == '[' || text[i] == ']')
            return -1;
    memcpy(wait.text, text ? text : "", len);
    wait.text[len] = '\0';
    wait.ok = ok;
    wait.answered = true;
    return 0;
}

bool gfmcode_waiting (void)
{
    return wait.active;
}

void gfmcode_state_json (char *buf, size_t len)
{
    if(!wait.active || wait.answered) {
        snprintf(buf, len, "null");
        return;
    }
    char words[96] = "";
    size_t w = 0;
    if(wait.has_p)
        w += (size_t)snprintf(words + w, sizeof(words) - w, "\"P\":%.6g", (double)wait.p);
    if(wait.has_q && w < sizeof(words))
        w += (size_t)snprintf(words + w, sizeof(words) - w, "%s\"Q\":%.6g", w ? "," : "", (double)wait.q);
    if(wait.has_r && w < sizeof(words))
        snprintf(words + w, sizeof(words) - w, "%s\"R\":%.6g", w ? "," : "", (double)wait.r);
    snprintf(buf, len, "{\"seq\":%u,\"code\":%d,\"words\":{%s}}", wait.seq, wait.code, words);
}

static void hold_job (const char *msg)
{
    report_message(msg, Message_Warning);
    fflog(LOG_WARNING, "mcode: %s", msg);
    system_set_exec_state_flag(EXEC_FEED_HOLD);
}

static user_mcode_type_t mcodeCheck (user_mcode_t mcode)
{
    if(ours((int)mcode))
        return answered_now((int)mcode) ? UserMCode_Normal : UserMCode_Unsupported;
    return chained.check ? chained.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcodeValidate (parser_block_t *gc_block)
{
    if(!ours((int)gc_block->user_mcode))
        return chained.validate ? chained.validate(gc_block) : Status_Unhandled;
    /* P, Q and R are the M-code's own words; any other stays the block's,
     * as for any M-code, and one the block cannot use is the core's error. */
    if((gc_block->words.p && !isfinite(gc_block->values.p)) || (gc_block->words.q && !isfinite(gc_block->values.q)) ||
       (gc_block->words.r && !isfinite(gc_block->values.r)))
        return Status_BadNumberFormat;
    gc_block->words.p = gc_block->words.q = gc_block->words.r = Off;
    gc_block->user_mcode_sync = true;
    return Status_OK;
}

static void mcodeExecute (sys_state_t state, parser_block_t *gc_block)
{
    int code = (int)gc_block->user_mcode;
    char msg[160];

    if(!ours(code)) {
        if(chained.execute)
            chained.execute(state, gc_block);
        return;
    }
    if(!answered_now(code)) {       /* its package went away between the parse and here */
        snprintf(msg, sizeof(msg), "M%d has no extension to answer it now: the job is held", code);
        hold_job(msg);
        return;
    }

    /* Every step is the stream's now; the head stops when the kernel has
     * played them. Pumped as the wait below is. */
    const struct timespec nap = { 0, 5 * 1000 * 1000 };
    double drained = now_s() + GFMCODE_DRAIN_S;
    while(!gf_stream_kernel_idle() && now_s() < drained) {
        if(!protocol_execute_realtime() || (state_get() & (STATE_ALARM | STATE_ESTOP)))
            return;
        nanosleep(&nap, NULL);
    }
    if(!gf_stream_kernel_idle()) {
        snprintf(msg, sizeof(msg), "M%d: the move before it did not finish in %.0f s: the job is held", code,
                 GFMCODE_DRAIN_S);
        hold_job(msg);
        return;
    }

    wait.seq = seq_next++;
    if(seq_next == 0)
        seq_next = 1;
    wait.code = code;
    wait.has_p = gc_block->words.p;
    wait.has_q = gc_block->words.q;
    wait.has_r = gc_block->words.r;
    wait.p = gc_block->values.p;
    wait.q = gc_block->values.q;
    wait.r = gc_block->values.r;
    wait.answered = false;
    wait.text[0] = '\0';
    wait.active = true;
    snprintf(msg, sizeof(msg), "M%d waits for its extension", code);
    report_message(msg, Message_Info);
    fflog(LOG_INFO, "mcode: M%d (seq %u) waits for its extension", code, wait.seq);

    /* Pumped as a dwell is: the port's answer arrives through the realtime
     * hook, and a reset or an alarm ends the wait. */
    double end = now_s() + GFMCODE_WAIT_S;
    while(!wait.answered && now_s() < end) {
        if(!protocol_execute_realtime() || (state_get() & (STATE_ALARM | STATE_ESTOP)))
            break;
        nanosleep(&nap, NULL);
    }
    bool answered = wait.answered, ok = wait.ok;
    wait.active = false;

    if(sys.abort || (state_get() & (STATE_ALARM | STATE_ESTOP))) {
        fflog(LOG_NOTICE, "mcode: M%d (seq %u): the job ended while it waited", code, wait.seq);
        return;
    }
    if(answered && ok) {
        snprintf(msg, sizeof(msg), "M%d: done%s%s", code, wait.text[0] ? ": " : "", wait.text);
        report_message(msg, Message_Info);
        fflog(LOG_INFO, "mcode: %s", msg);
    } else if(answered) {
        snprintf(msg, sizeof(msg), "M%d: %s: the job is held", code, wait.text[0] ? wait.text : "its extension did not do it");
        hold_job(msg);
    } else {
        snprintf(msg, sizeof(msg), "M%d had no answer from its extension in %.0f s: the job is held", code, GFMCODE_WAIT_S);
        hold_job(msg);
    }
}

void gfmcode_init (void)
{
    chained = grbl.user_mcode;
    grbl.user_mcode.check = mcodeCheck;
    grbl.user_mcode.validate = mcodeValidate;
    grbl.user_mcode.execute = mcodeExecute;
}
