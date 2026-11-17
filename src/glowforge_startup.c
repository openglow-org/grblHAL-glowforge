/*
  glowforge_startup.c - the startup lines a sender stores ($N0, $N1)

  Part of grblHAL-glowforge. The core runs the stored startup lines at every
  controller start and every soft reset: after a reboot, a respawn, a mode
  switch, the restart that follows a cooling verdict's fail tier, with no
  sender connected, X and Y unreferenced, and the lens not yet referenced.
  It also validates a line by executing it when it is stored, in Idle and in
  Alarm, so a motion stored while the motors are released is planned and
  plays when they are energized. A startup line here is therefore modal
  state only (glowforge_startup.h names the words): $N0 and $N1 refuse
  anything else before the core sees it, and a stored line that holds more
  than modal state, whatever wrote it to the settings store, is cleared
  before the core first runs the lines.

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "fflog.h"
#include "driver.h"
#include "glowforge_startup.h"

#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/settings.h"
#include "grbl/system.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

/* A G word's number in tenths: G21 is 210, G59.1 is 591. */
static bool word_ok (unsigned long tenths)
{
    switch(tenths) {
        case 170: case 180: case 190:                       /* plane */
        case 200: case 210:                                 /* units */
        case 540: case 550: case 560: case 570: case 580: case 590:
        case 591: case 592: case 593:                       /* work coordinate system */
        case 900: case 910: case 911:                       /* distance modes */
        case 930: case 940:                                 /* feed mode */
            return true;
        default:
            return false;
    }
}

bool gfstartup_line_ok (const char *line)
{
    const char *p = line;

    for(;;) {
        char c = *p;

        if(c == '\0' || c == ';')
            return true;                    /* the end, or a comment to it */
        if(c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '|') {
            p++;
            continue;
        }
        if(c == '(') {                      /* a comment to its ')' */
            while(*p != '\0' && *p != ')')
                p++;
            if(*p == '\0')
                return false;
            p++;
            continue;
        }
        if(c != 'G' && c != 'g')
            return false;

        p++;
        while(*p == ' ' || *p == '\t')
            p++;
        if(!isdigit((unsigned char)*p))
            return false;
        unsigned long whole = 0;
        int digits = 0;
        while(isdigit((unsigned char)*p)) {
            if(++digits > 3)
                return false;
            whole = whole * 10 + (unsigned long)(*p++ - '0');
        }
        unsigned long tenth = 0;
        if(*p == '.') {
            p++;
            for(int n = 0; isdigit((unsigned char)*p); n++, p++) {
                if(n == 0)
                    tenth = (unsigned long)(*p - '0');
                else if(*p != '0')
                    return false;
            }
        }
        if(!word_ok(whole * 10 + tenth))
            return false;
    }
}

static const char refused[] = "Startup lines take only G17-G19, G20, G21, G54-G59.3, G90, G91, "
                              "G91.1, G93 and G94";

static status_code_t store_line (char *args, unsigned n)
{
    if(args == NULL || gfstartup_line_ok(args))
        return Status_Unhandled;            /* the core's own $N0/$N1 stores it */

    report_message(refused, Message_Warning);
    fflog(LOG_WARNING, "startup line $N%u refused: %s", n, args);
    return Status_GcodeUnsupportedCommand;
}

static status_code_t store_line0 (sys_state_t state, char *args)
{
    (void)state;
    return store_line(args, 0);
}

static status_code_t store_line1 (sys_state_t state, char *args)
{
    (void)state;
    return store_line(args, 1);
}

void gfstartup_check (void)
{
    char line[sizeof(stored_line_t)];

    for(uint_fast8_t n = 0; n < N_STARTUP_LINE; n++) {
        if(!settings_read_startup_line(n, line))
            continue;                       /* unreadable: the core has cleared it */
        line[sizeof(line) - 1] = '\0';
        if(gfstartup_line_ok(line))
            continue;
        fflog(LOG_WARNING, "startup line $N%u cleared, it held more than modal state: %s",
              (unsigned)n, line);
        line[0] = '\0';
        settings_write_startup_line(n, line);
    }
}

static const sys_command_t startup_command_list[] = {
    { "N0", store_line0 },
    { "N1", store_line1 }
};

static sys_commands_t startup_commands = {
    .n_commands = sizeof(startup_command_list) / sizeof(sys_command_t),
    .commands = startup_command_list
};

void gfstartup_init (void)
{
    system_register_commands(&startup_commands);
}
