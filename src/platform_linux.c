/*
  platform_linux.c - linux specific functions with generic cross-platform interface

  Part of Grbl Simulator

  Copyright (c) 2014 Adam Shelly

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include "platform.h"


// Saved STDIN state so it can be restored on exit. The terminal is switched to
// raw mode (and STDIN made non-blocking) once at startup instead of on every
// poll, so platform_poll_stdin() can be a single non-blocking read().
static struct termios orig_termios;
static int termios_saved = 0;
static int orig_stdin_flags = 0;
static int stdin_flags_saved = 0;

//restore STDIN to the state it had before platform_init()
static void platform_restore_terminal(void)
{
    if (termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        termios_saved = 0;
    }
    if (stdin_flags_saved) {
        fcntl(STDIN_FILENO, F_SETFL, orig_stdin_flags);
        stdin_flags_saved = 0;
    }
}

void platform_init()
{
    // Make STDIN non-blocking once so polling is a single read() with no
    // per-poll select()/tcsetattr() dance. Save the original flags so they can
    // be restored on exit (important when STDIN is a shared tty).
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) {
        orig_stdin_flags = flags;
        stdin_flags_saved = 1;
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    // If STDIN is a terminal, switch it to raw (non-canonical, no echo) mode
    // once. For a pipe/redirected file tcgetattr() fails and we leave it alone.
    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig_termios) == 0) {
        struct termios raw = orig_termios;
        raw.c_lflag &= ~(ICANON);
        raw.c_lflag &= ~(ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        termios_saved = 1;
    }

    // Restore the terminal even on abnormal exit.
    atexit(platform_restore_terminal);
}

uint8_t platform_poll_stdin()
{
    uint8_t char_in = 0;

    // STDIN was put in non-blocking mode once in platform_init(), so polling
    // is a single read().
    ssize_t n = read(STDIN_FILENO, &char_in, 1);

    if (n == 1)
        return char_in;   // byte available
    if (n == 0)
        return 0xFF;      // EOF: 0xFF signals end-of-input to the caller

    return 0;             // no data (EAGAIN/EWOULDBLOCK) or error -> nothing
}
