/*
  glowforge_io.c - Glowforge kernel-driver sysfs / pulse-device access

  Part of grblHAL-glowforge.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

/* The project headers first: glibc's stat.h (via fcntl.h) defines an
 * st_mtime macro that would otherwise mangle the field of that name in
 * the core's vfs.h, which the settings header below reaches. */
#include "glowforge_io.h"
#include "fflog.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

#define GF_SYSFS "/sys/glowforge/"
/* The attribute tree's root: a host test that includes this source defines
 * GFIO_SYSFS_ROOT() as a tree of its own (it ends with '/'). */
#ifndef GFIO_SYSFS_ROOT
#define GFIO_SYSFS_ROOT() GF_SYSFS
#endif

/* PIC stepper currents from the factory print header (XSrc/XShc, YSrc/YShc;
 * identical across every captured job, 2018 and 2026 firmware alike). The
 * X/Y scales differ by design - these are the factory's own DAC values. */
#define X_CURRENT_RUN  "135"
#define X_CURRENT_HOLD "33"
#define Y_CURRENT_RUN  "22"
#define Y_CURRENT_HOLD "5"

/* Null-sink instances must never touch the machine: on a dev host the
 * sysfs opens simply fail, but on the board a "no hardware I/O" test
 * process would otherwise drive the real laser latch and analog config.
 * Writes are gated until the instance declares hardware ownership. */
static bool gfio_hw = false;

void gfio_set_hw (bool active)
{
    gfio_hw = active;
}

int gfio_wr_attr (const char *attr, const char *val)
{
    char path[128];
    int fd, ret = 0;
    size_t len = strlen(val);
    if(!gfio_hw) {
        const char *log = getenv("GFSINK_ATTR_LOG");
        if(log != NULL && *log != '\0') {
            FILE *f = fopen(log, "a");
            if(f) {
                fprintf(f, "%s %s\n", attr, val);
                fclose(f);
            }
        }
        return 0;
    }
    snprintf(path, sizeof(path), "%s%s", GFIO_SYSFS_ROOT(), attr);
    if((fd = open(path, O_WRONLY | O_CLOEXEC)) < 0)
        return errno == ENOENT ? GFIO_ENOATTR : GFIO_EREJECT;
    /* A sysfs store consumes the whole buffer or rejects it; anything
     * short of a full write is a rejection, and EINTR is retried. */
    for(;;) {
        ssize_t w = write(fd, val, len);
        if(w == (ssize_t)len)
            break;
        if(w < 0 && errno == EINTR)
            continue;
        ret = GFIO_EREJECT;
        break;
    }
    close(fd);
    return ret;
}

int gfio_rd_attr (const char *attr, char *buf, size_t len)
{
    char path[128];
    int fd;
    ssize_t n;
    snprintf(path, sizeof(path), "%s%s", GFIO_SYSFS_ROOT(), attr);
    if((fd = open(path, O_RDONLY | O_CLOEXEC)) < 0)
        return -1;
    n = read(fd, buf, len - 1);
    close(fd);
    if(n < 0)
        return -1;
    while(n > 0 && buf[n - 1] == '\n')
        n--;
    buf[n] = '\0';
    return 0;
}

int gfio_rd_bin (const char *attr, void *buf, size_t len)
{
    char path[128];
    int fd;
    ssize_t n;
    snprintf(path, sizeof(path), "%s%s", GFIO_SYSFS_ROOT(), attr);
    if((fd = open(path, O_RDONLY | O_CLOEXEC)) < 0)
        return -1;
    do
        n = read(fd, buf, len);
    while(n < 0 && errno == EINTR);
    close(fd);
    return n < 0 ? -1 : (int)n;
}

/* Standalone: the device is this process's alone and must never reach
 * a child (the homing runner gets it through the broker's inherited fd
 * only, and standalone the suspend closes it first). */
static int open_pulse_dev (const char *path, int lock_flags)
{
    int fd;

    if((fd = open(path, O_WRONLY | O_CLOEXEC)) < 0)
        return -1;

    if(flock(fd, lock_flags) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

/* Broker mode: the forgectrl supervisor holds /dev/glowforge open for
 * its lifetime (the device is exclusive-open) and hands the writer a
 * dup at spawn, named by GF_PULSE_FD. The flock rides the shared open
 * file description - the relock below is a no-op on it - and closing
 * this fd is never the final close while the supervisor lives, so
 * handovers stop cycling the 40 V rail. The supervisor is the
 * dead-man for its writers; the kernel dead-man still backstops the
 * whole description. */
static int inherited_pulse_fd (void)
{
    const char *v = getenv("GF_PULSE_FD");
    if(v == NULL || *v == '\0')
        return -1;
    int fd = atoi(v);
    if(fd < 0 || fcntl(fd, F_GETFL) < 0)
        return -1;
    return fd;
}

bool gfio_pulse_inherited (void)
{
    return inherited_pulse_fd() >= 0;
}

/* The inherited fd rides the broker's open file description, which
 * already carries the lock, so the relock is a no-op there; one that
 * fails anyway is said, never hidden. The fd stays inheritable: the
 * supervisor cleared its close-on-exec for this child, and the homing
 * runner needs it the same way. */
static int inherited_locked (int fd)
{
    if(flock(fd, LOCK_EX) != 0)
        fflog(LOG_WARNING, "gfio: flock on the inherited pulse fd failed: %s", strerror(errno));
    return fd;
}

int gfio_open_pulse_dev (const char *path)
{
    int fd = inherited_pulse_fd();
    if(fd >= 0)
        return inherited_locked(fd);
    return open_pulse_dev(path, LOCK_EX);
}

int gfio_open_pulse_dev_nb (const char *path)
{
    int fd = inherited_pulse_fd();
    if(fd >= 0)
        return inherited_locked(fd);
    return open_pulse_dev(path, LOCK_EX | LOCK_NB);
}

/* One write of the stepper drivers' configuration, named when it fails. */
static bool config_write (const char *attr, const char *val)
{
    int rc = gfio_wr_attr(attr, val);
    if(rc != 0)
        fflog(LOG_ERR, "gfio: %s=%s did not take (%s)", attr, val,
              rc == GFIO_ENOATTR ? "no such attribute" : "refused");
    return rc == 0;
}

bool gfio_analog_config (void)
{
    char mode[12];
    bool ok = true;
    snprintf(mode, sizeof(mode), "%u", gfio_xy_microsteps());
    ok = config_write("cnc/x_mode", mode) && ok;
    ok = config_write("cnc/y_mode", mode) && ok;
    ok = config_write("cnc/x_decay", "1") && ok;
    ok = config_write("cnc/y_decay", "1") && ok;
    /* Every axis in the pulse path, the lens included: Z is the focal
     * height and a job's Z moves the lens. The lens motor steps in
     * half-steps ($102) and rises only at its drive current, which the
     * run posture sets and the hold posture takes back. */
    ok = config_write("cnc/motor_lock", "0") && ok;
    /* The head's own: named when they fail, and no refusal, so a machine
     * without its head still moves X and Y (no lens moves unreferenced). */
    config_write("head/z_mode", "1");
    config_write("head/z_enable", "0");
    gfio_currents_hold();
    return ok;
}

/* Written on the protocol thread, read by the shipper's hold drop. */
static _Atomic bool xy_released = false;

void gfio_xy_released_set (bool released)
{
    xy_released = released;
}

bool gfio_xy_released (void)
{
    return xy_released;
}

/* A failed current write is named once per episode, X and Y (the PIC) and
 * the lens (the head) apart; a write that takes ends its episode. Only the
 * protocol thread writes the currents. */
static bool xy_current_failed, z_current_failed;

static bool currents_write (const char *x, const char *y, const char *z)
{
    int rx = gfio_wr_attr("pic/x_step_current", x);
    int ry = gfio_wr_attr("pic/y_step_current", y);
    int rz = gfio_wr_attr("head/z_current", z);
    bool xy_ok = rx == 0 && ry == 0;

    if(!xy_ok && !xy_current_failed)
        fflog(LOG_ERR, "gfio: the X and Y step currents did not take (x %d, y %d)", rx, ry);
    else if(xy_ok && xy_current_failed)
        fflog(LOG_NOTICE, "gfio: the X and Y step currents take again");
    xy_current_failed = !xy_ok;
    if(rz != 0 && !z_current_failed)
        fflog(LOG_ERR, "gfio: the lens current did not take (%d)", rz);
    else if(rz == 0 && z_current_failed)
        fflog(LOG_NOTICE, "gfio: the lens current takes again");
    z_current_failed = rz != 0;
    return xy_ok;
}

bool gfio_currents_run (void)
{
    return currents_write(xy_released ? "0" : X_CURRENT_RUN, xy_released ? "0" : Y_CURRENT_RUN, "0");
}

bool gfio_currents_hold (void)
{
    return currents_write(xy_released ? "0" : X_CURRENT_HOLD, xy_released ? "0" : Y_CURRENT_HOLD, "1");
}

/* --- the shared machine config ("key = value", '#' comments) --------- */

#define CONF_DEFAULT "/data/forgefirm/forgefirm.conf"

static const char *conf_path (void)
{
    const char *p = getenv("GFHOME_CONF");
    return (p && *p) ? p : CONF_DEFAULT;
}

/* A value too long for its reader is refused, never read as its first
 * characters: a gate value cut short reads as another number. Named once
 * for each key (a short table; past it, every refusal is named). */
#define CONF_REFUSED_KEYS 8
static char conf_refused[CONF_REFUSED_KEYS][48];

static void conf_refuse (const char *key, size_t have, size_t room)
{
    for(int i = 0; i < CONF_REFUSED_KEYS; i++) {
        if(!strcmp(conf_refused[i], key))
            return;
        if(conf_refused[i][0] == '\0') {
            snprintf(conf_refused[i], sizeof(conf_refused[i]), "%s", key);
            break;
        }
    }
    fflog(LOG_WARNING, "gfio: the setting %s is %zu characters, over the %zu it is read in; "
          "its default stands", key, have, room);
}

int gfio_conf_read (const char *key, char *val, size_t len)
{
    FILE *f = fopen(conf_path(), "r");
    if(f == NULL)
        return -1;

    char line[256];
    int found = -1;
    while(fgets(line, sizeof(line), f)) {
        /* A line longer than the buffer is no setting: the rest of it is
         * skipped, never read as a line of its own, and a key it names
         * reads as refused. */
        size_t n = strlen(line);
        bool whole = n > 0 && (line[n - 1] == '\n' || feof(f));
        if(!whole) {
            int c;
            while((c = fgetc(f)) != EOF && c != '\n')
                ;
        }
        char *p = line;
        while(isspace((unsigned char)*p))
            p++;
        if(*p == '#' || *p == '\0')
            continue;
        char *eq = strchr(p, '=');
        if(eq == NULL)
            continue;
        char *k_end = eq;
        while(k_end > p && isspace((unsigned char)k_end[-1]))
            k_end--;
        *k_end = '\0';
        if(strcmp(p, key))
            continue;
        char *v = eq + 1;
        while(isspace((unsigned char)*v))
            v++;
        char *v_end = v + strlen(v);
        while(v_end > v && isspace((unsigned char)v_end[-1]))
            v_end--;
        *v_end = '\0';
        /* Keep scanning: the last occurrence wins, a refused one too. */
        size_t vlen = (size_t)(v_end - v);
        if(!whole || vlen >= len) {
            conf_refuse(key, whole ? vlen : sizeof(line) - 1, len - 1);
            found = -1;
            continue;
        }
        memcpy(val, v, vlen + 1);
        found = 0;
    }
    fclose(f);
    return found;
}

float gfio_conf_read_float (const char *key, float fallback)
{
    char val[32], *end;
    if(gfio_conf_read(key, val, sizeof(val)) != 0)
        return fallback;
    /* The whole value is the number, or the fallback stands: never a
     * number read off its front. */
    float f = strtof(val, &end);
    return end == val || *end != '\0' ? fallback : f;
}

int gfio_conf_sibling (const char *name, char *buf, size_t len)
{
    const char *conf = conf_path();
    const char *slash = strrchr(conf, '/');
    int n = slash ? snprintf(buf, len, "%.*s/%s", (int)(slash - conf), conf, name)
                  : snprintf(buf, len, "%s", name);
    return n < 0 || (size_t)n >= len ? -1 : 0;
}

unsigned gfio_xy_microsteps (void)
{
    static unsigned mode = 0;

    if(mode == 0) {
        char val[32];
        if(gfio_conf_read("xy_microsteps", val, sizeof(val)) == 0) {
            mode = gfio_xy_mode_parse(val);
            if(mode == 0)
                fflog(LOG_WARNING, "xy_microsteps '%s' is not 8, 16 or 32; running at x%u",
                      val, XY_MICROSTEPS_DEFAULT);
        }
        if(mode == 0)
            mode = XY_MICROSTEPS_DEFAULT;
    }
    return mode;
}
