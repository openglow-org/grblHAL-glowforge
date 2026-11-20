/*
  xy_scale_test.c - host unit test for the XY microstep scale

  Part of grblHAL-glowforge. The XY microstep mode (xy_microsteps in the
  shared config: 8, 16 or 32) is one number three coupled quantities are
  derived from, never typed (src/glowforge_io.h helpers):

    - $100/$101: 0.15 mm per full step, three decimals per mode
    - the machine tick: the factory travel tick at x8, scaled with the
      mode so the ticks per step and the top speed stay the same
    - the kernel stop ramp: Hz/s of tick frequency, scaled with the tick
      in force so a controlled stop covers the same distance, held to
      the kernel's bounds
    - the feed ceiling the stream carries: one step per tick per axis,
      which is what holds $110/$111 down when the bench lowers the tick

  And the bounds on $110-$112 (at most the board's defaults) and $120-$122
  (at least half of them): a typed value past one is refused
  (gfio_motion_setting_refused), and a stored one is held at it.

  And the pins the settings dispatch re-asserts (gfio_apply_machine_pins):
  $100/$101 from the mode, $110/$111 under the ceiling, $102 the lens
  screw's constant (a typed 0 or 999 is overwritten), $32 laser mode (a
  typed 0 is overwritten), $63 at 3, the laser off in a hold and restored
  on resume (a typed $63=0 or $63=7 is overwritten), $34 at 0, the
  laser's off value (a typed 10 is overwritten), the $22 bits of the
  core's homing cycle off (a typed $22 with every bit set loses those six
  and keeps the rest), $341 at 4, M6 ignored (a typed 1 is overwritten),
  $673 at 0, no coolant on-delay (a typed 3 s is overwritten), and
  $44-$46, the homing cycle masks (a typed
  $44=1, or all three typed 0, is overwritten with the driver's cycles,
  whose union holds Z: the core keeps an axis homed only while a cycle
  names it, and the lens hold rests on the Z homed bit).

  And glowforge_io.c's shared config reader and current writes (see
  conf_reader and current_writes below).

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "glowforge_io.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* The shared config's reader and the current writes are glowforge_io.c's
   own, included here: a value too long for its reader is refused, never
   read as its first characters, and a float is the whole value or the
   fallback; a failed current write is named once an episode. */
static char test_sysfs[64];             /* the attribute tree, made by current_writes */
#define GFIO_SYSFS_ROOT() test_sysfs
#include "../src/glowforge_io.c"
#include <sys/stat.h>                   /* after the core headers: its macros clash with vfs.h */

static int log_lines;                   /* fflog lines at warning or worse */
void fflog (int prio, const char *fmt, ...) { if(prio <= LOG_WARNING) log_lines++; (void)fmt; }

static int failures = 0;

static void expect_parse (const char *val, unsigned want)
{
    unsigned got = gfio_xy_mode_parse(val);
    if (got != want) {
        printf("FAIL parse '%s': %u (want %u)\n", val, got, want);
        failures++;
    } else {
        printf("ok   parse '%s' = %u\n", val, got);
    }
}

static void expect_u (const char *what, unsigned got, unsigned want)
{
    if (got != want) {
        printf("FAIL %s: %u (want %u)\n", what, got, want);
        failures++;
    } else {
        printf("ok   %s = %u\n", what, got);
    }
}

static void expect_f (const char *what, float got, float want, float tol)
{
    if (!(fabsf(got - want) <= tol)) {        /* a NaN fails */
        printf("FAIL %s: %.4f (want %.4f)\n", what, got, want);
        failures++;
    } else {
        printf("ok   %s = %.3f\n", what, got);
    }
}

/* The shared config's reader: a value longer than the reader's buffer is
   refused and the default stands, named once per key; a line longer than
   the reader's line is skipped whole, so nothing in its tail reads as a
   setting; a float is the whole value or the fallback. */
static void conf_reader (void)
{
    char path[] = "/tmp/xy_scale_test.XXXXXX";
    int fd = mkstemp(path);
    FILE *f = fd >= 0 ? fdopen(fd, "w") : NULL;
    if (f == NULL) {
        printf("FAIL cannot create the config file\n");
        failures++;
        return;
    }
    fprintf(f, "cool_fan_grace_s = 0000000000000000000000000000000688\n");   /* 34 characters */
    fprintf(f, "laser_disarm_s = 12abc\n");
    fputc('#', f);                      /* a comment longer than the reader's 255 characters, */
    for (int i = 0; i < 254; i++)       /* with a setting in its tail */
        fputc('x', f);
    fprintf(f, "rail_settle_s = 9\n");
    fprintf(f, "laser_floor_density = 12.5\n");
    fprintf(f, "lid_policy = hold\n");
    fclose(f);
    setenv("GFHOME_CONF", path, 1);

    char v[16];
    int lines0 = log_lines;
    expect_f("a value longer than the reader's 31 characters: the fallback",
             gfio_conf_read_float("cool_fan_grace_s", 15.0f), 15.0f, 1e-6f);
    gfio_conf_read_float("cool_fan_grace_s", 15.0f);
    expect_u("the refusal is named once for its key", (unsigned)(log_lines - lines0), 1);
    expect_f("a value that is not wholly a number: the fallback",
             gfio_conf_read_float("laser_disarm_s", 60.0f), 60.0f, 1e-6f);
    expect_f("a setting in the tail of a line longer than the reader's is none",
             gfio_conf_read_float("rail_settle_s", 2.5f), 2.5f, 1e-6f);
    expect_f("the line after it reads whole", gfio_conf_read_float("laser_floor_density", 10.0f), 12.5f, 1e-6f);
    expect_u("a value that fits reads", (unsigned)(gfio_conf_read("lid_policy", v, sizeof(v)) == 0 &&
                                                 !strcmp(v, "hold")), 1);
    expect_u("a value as long as the buffer is refused", (unsigned)(gfio_conf_read("lid_policy", v, 4) != 0), 1);
    unsetenv("GFHOME_CONF");
    unlink(path);
}

/* The attribute tree the writes land in (test_sysfs): every attribute
   present, or every one but `gone`, whose write then fails as a missing
   attribute does. */
static const char *const tree_attrs[] = {
    "cnc/x_mode", "cnc/y_mode", "cnc/x_decay", "cnc/y_decay", "cnc/motor_lock",
    "head/z_mode", "head/z_enable", "head/z_current", "pic/x_step_current", "pic/y_step_current" };

static void tree (const char *gone)
{
    char path[160];
    for (size_t i = 0; i < sizeof(tree_attrs) / sizeof(tree_attrs[0]); i++) {
        snprintf(path, sizeof(path), "%s%s", test_sysfs, tree_attrs[i]);
        if (gone && !strcmp(tree_attrs[i], gone)) {
            unlink(path);
            continue;
        }
        FILE *f = fopen(path, "w");
        if (f)
            fclose(f);
    }
}

/* The current writes and the drivers' configuration, one attribute
   failing at a time: an X or Y current that does not take is false and
   named once for its episode, the lens's named apart and refusing
   nothing; each of the X and Y modes and decays and the motor lock that
   does not take makes the configuration false, the head's refuse nothing;
   a write that takes ends an episode. */
static void current_writes (void)
{
    char tmpl[] = "/tmp/xy_scale_sys.XXXXXX";
    if (mkdtemp(tmpl) == NULL) {
        printf("FAIL cannot create the attribute tree\n");
        failures++;
        return;
    }
    snprintf(test_sysfs, sizeof(test_sysfs), "%s/", tmpl);
    char dir[96];
    const char *const dirs[] = { "cnc", "head", "pic" };
    for (int i = 0; i < 3; i++) {
        snprintf(dir, sizeof(dir), "%s%s", test_sysfs, dirs[i]);
        mkdir(dir, 0755);
    }
    gfio_set_hw(true);

    tree(NULL);
    int lines0 = log_lines;
    expect_u("every attribute present: the run currents take", (unsigned)gfio_currents_run(), 1);
    expect_u("and the configuration takes", (unsigned)gfio_analog_config(), 1);
    expect_u("and nothing is named", (unsigned)(log_lines - lines0), 0);

    tree("pic/y_step_current");
    expect_u("the Y run current alone does not take: false", (unsigned)gfio_currents_run(), 0);
    gfio_currents_hold();
    expect_u("named once for its episode", (unsigned)(log_lines - lines0), 1);
    tree(NULL);
    gfio_currents_run();
    lines0 = log_lines;
    tree("pic/x_step_current");
    expect_u("the X run current alone does not take: false", (unsigned)gfio_currents_run(), 0);
    expect_u("after a write that took, a new episode, named again", (unsigned)(log_lines - lines0), 1);
    tree("head/z_current");
    lines0 = log_lines;
    expect_u("the lens current alone does not take: still true", (unsigned)gfio_currents_run(), 1);
    expect_u("and it is named", (unsigned)(log_lines - lines0) >= 1, 1);

    static const char *const config_attrs[] = {
        "cnc/x_mode", "cnc/y_mode", "cnc/x_decay", "cnc/y_decay", "cnc/motor_lock" };
    for (size_t i = 0; i < sizeof(config_attrs) / sizeof(config_attrs[0]); i++) {
        char what[96];
        tree(config_attrs[i]);
        snprintf(what, sizeof(what), "%s alone does not take: the configuration is false", config_attrs[i]);
        expect_u(what, (unsigned)gfio_analog_config(), 0);
    }
    tree("head/z_mode");
    expect_u("the head's Z mode alone does not take: the configuration still takes",
             (unsigned)gfio_analog_config(), 1);
    tree("head/z_enable");
    expect_u("the head's Z enable alone: the same", (unsigned)gfio_analog_config(), 1);

    gfio_set_hw(false);
    for (size_t i = 0; i < sizeof(tree_attrs) / sizeof(tree_attrs[0]); i++) {
        char path[160];
        snprintf(path, sizeof(path), "%s%s", test_sysfs, tree_attrs[i]);
        unlink(path);
    }
    for (int i = 0; i < 3; i++) {
        snprintf(dir, sizeof(dir), "%s%s", test_sysfs, dirs[i]);
        rmdir(dir);
    }
    rmdir(tmpl);
}

int main (void)
{
    conf_reader();
    current_writes();

    /* The key names a mode or nothing: no rounding, no prefixes. */
    expect_parse("8", 8);
    expect_parse("16", 16);
    expect_parse("32", 32);
    expect_parse("1", 0);
    expect_parse("24", 0);
    expect_parse("16x", 0);
    expect_parse("", 0);
    expect_parse("abc", 0);

    /* $100/$101: the x8 literal is the board's scale base; every mode is
     * 0.15 mm per full step to three decimals. */
    expect_f("$100 at x8", gfio_xy_steps_per_mm_of(8), 53.333f, 1e-4f);
    expect_f("$100 at x16", gfio_xy_steps_per_mm_of(16), 106.667f, 1e-4f);
    expect_f("$100 at x32", gfio_xy_steps_per_mm_of(32), 213.333f, 1e-4f);

    /* The tick keeps the ticks per step at the factory's. */
    expect_u("scale at x8", gfio_xy_scale_of(8), 1);
    expect_u("scale at x16", gfio_xy_scale_of(16), 2);
    expect_u("scale at x32", gfio_xy_scale_of(32), 4);
    expect_u("tick at x8", gfio_xy_tick_hz_of(8), 28160);
    expect_u("tick at x16", gfio_xy_tick_hz_of(16), 56320);
    expect_u("tick at x32", gfio_xy_tick_hz_of(32), 112640);

    /* The ramp follows the tick, whatever set it, inside the kernel's
     * bounds: 32 at the full tick lands exactly on the maximum, and the
     * 3x fallback tick sits between. */
    expect_u("ramp at 28160 Hz", gfio_xy_ramp_for_tick(28160), 125000);
    expect_u("ramp at 56320 Hz", gfio_xy_ramp_for_tick(56320), 250000);
    expect_u("ramp at 112640 Hz", gfio_xy_ramp_for_tick(112640), 500000);
    expect_u("ramp at 84480 Hz", gfio_xy_ramp_for_tick(84480), 375000);
    expect_u("ramp at 1000 Hz (floor)", gfio_xy_ramp_for_tick(1000), 10000);
    expect_u("ramp at 165000 Hz (ceiling)", gfio_xy_ramp_for_tick(165000), 500000);

    /* The feed ceiling: the same 31680 mm/min at every mode on its own
     * tick, a quarter of that for x32 held at the x8 tick. */
    expect_f("ceiling x8 at 28160 Hz", gfio_xy_rate_ceiling(28160, gfio_xy_steps_per_mm_of(8)), 31680.0f, 0.5f);
    expect_f("ceiling x32 at 112640 Hz", gfio_xy_rate_ceiling(112640, gfio_xy_steps_per_mm_of(32)), 31680.0f, 0.5f);
    expect_f("ceiling x32 at 28160 Hz", gfio_xy_rate_ceiling(28160, gfio_xy_steps_per_mm_of(32)), 7920.0f, 0.5f);
    expect_f("ceiling x16 at 1000 Hz", gfio_xy_rate_ceiling(1000, gfio_xy_steps_per_mm_of(16)), 562.5f, 0.05f);

    /* The pins: whatever a sender typed, the dispatch puts the machine's
     * values back. $102 at 0 (the NaN envelope) or 999, and $32 off, are
     * the typed values that matter. */
    settings_t s;
    memset(&s, 0, sizeof(s));
    s.axis[X_AXIS].steps_per_mm = 1.0f;
    s.axis[Y_AXIS].steps_per_mm = 1.0f;
    s.axis[X_AXIS].max_rate = 99999.0f;
    s.axis[Y_AXIS].max_rate = 6000.0f;
    s.axis[Z_AXIS].steps_per_mm = 0.0f;
    s.mode = Mode_Standard;
    uint8_t held = gfio_apply_machine_pins(&s, 8, 28160);
    expect_f("$100 pinned from the mode", s.axis[X_AXIS].steps_per_mm, 53.333f, 1e-4f);
    expect_f("$101 pinned from the mode", s.axis[Y_AXIS].steps_per_mm, 53.333f, 1e-4f);
    expect_f("$110 stored past its cap is held at it", s.axis[X_AXIS].max_rate, 12000.0f, 1e-3f);
    expect_f("$111 under its cap is left alone", s.axis[Y_AXIS].max_rate, 6000.0f, 1e-3f);
    expect_u("the cap is no ceiling hold", held, 0);
    /* The accelerations, stored at 0 (memset): each raised to its floor,
       half the board's value, in the core's mm/min^2. */
    expect_f("$120 stored under its floor is raised to 350", s.axis[X_AXIS].acceleration, 350.0f * 3600.0f, 1.0f);
    expect_f("$121 stored under its floor is raised to 295", s.axis[Y_AXIS].acceleration, 295.0f * 3600.0f, 1.0f);
    expect_f("$122 stored under its floor is raised to 25", s.axis[Z_AXIS].acceleration, 25.0f * 3600.0f, 1.0f);
    s.axis[Z_AXIS].max_rate = 301.0f;
    s.axis[X_AXIS].acceleration = 700.0f * 3600.0f;
    gfio_apply_machine_pins(&s, 8, 28160);
    expect_f("$112 stored past its cap is held at 300", s.axis[Z_AXIS].max_rate, 300.0f, 1e-3f);
    expect_f("$120 above its floor is left alone", s.axis[X_AXIS].acceleration, 700.0f * 3600.0f, 1.0f);
    /* A stored NaN (a corrupt store) is held at the bound like any value
       past it. */
    s.axis[Z_AXIS].max_rate = NAN;
    s.axis[Z_AXIS].acceleration = NAN;
    gfio_apply_machine_pins(&s, 8, 28160);
    expect_f("$112 stored as NaN is held at 300", s.axis[Z_AXIS].max_rate, 300.0f, 1e-3f);
    expect_f("$122 stored as NaN is raised to 25", s.axis[Z_AXIS].acceleration, 25.0f * 3600.0f, 1.0f);
    /* The tick's ceiling under the cap (the bench's lowered tick): held,
       and reported. */
    held = gfio_apply_machine_pins(&s, 8, 7040);
    expect_f("$110 held under a lowered tick's ceiling", s.axis[X_AXIS].max_rate, 7920.0f, 0.5f);
    expect_f("$111 under it is left alone", s.axis[Y_AXIS].max_rate, 6000.0f, 1e-3f);
    expect_u("the held axes are reported", held, 1u << X_AXIS);

    /* A typed $110-$112 or $120-$122 past its bound is refused before it
       is stored; at the bound it is taken. */
    expect_u("$120=349.9 refused", gfio_motion_setting_refused(120, 349.9f), 1);
    expect_u("$120=350 taken", gfio_motion_setting_refused(120, 350.0f), 0);
    expect_u("$121=294.9 refused", gfio_motion_setting_refused(121, 294.9f), 1);
    expect_u("$121=295 taken", gfio_motion_setting_refused(121, 295.0f), 0);
    expect_u("$122=24.9 refused", gfio_motion_setting_refused(122, 24.9f), 1);
    expect_u("$122=25 taken", gfio_motion_setting_refused(122, 25.0f), 0);
    expect_u("$110=12000.1 refused", gfio_motion_setting_refused(110, 12000.1f), 1);
    expect_u("$110=12000 taken", gfio_motion_setting_refused(110, 12000.0f), 0);
    expect_u("$111=12000.1 refused", gfio_motion_setting_refused(111, 12000.1f), 1);
    expect_u("$111=12000 taken", gfio_motion_setting_refused(111, 12000.0f), 0);
    expect_u("$112=300.1 refused", gfio_motion_setting_refused(112, 300.1f), 1);
    expect_u("$112=300 taken", gfio_motion_setting_refused(112, 300.0f), 0);
    expect_u("$120=nan refused", gfio_motion_setting_refused(120, NAN), 1);
    expect_u("$130 is not this check's", gfio_motion_setting_refused(130, 0.0f), 0);
    expect_f("$102 typed to 0 is re-asserted", s.axis[Z_AXIS].steps_per_mm, DEFAULT_Z_STEPS_PER_MM, 1e-5f);
    expect_u("$32 typed to 0 is re-asserted", (unsigned)s.mode, (unsigned)Mode_Laser);
    /* $63 at 3, the laser off in a hold and restored on resume: a typed
     * $63=0 is put back, and so is a typed $63=7, whose bit 2 would leave
     * the laser on at its floor through the hold. */
    expect_u("$63 typed to 0 is re-asserted as 3",
             (unsigned)(s.flags.disable_laser_during_hold | s.flags.restore_after_feed_hold << 1 |
                        s.flags.set_rpm_0_during_hold << 2), 3);
    s.flags.disable_laser_during_hold = 1;
    s.flags.restore_after_feed_hold = 1;
    s.flags.set_rpm_0_during_hold = 1;
    /* $34, the laser's off value: a typed 10 is put back to 0. */
    s.pwm_spindle.pwm_off_value = 10.0f;
    /* $22: a sender's every bit set. The six bits of the core's homing
     * cycle go; the rest of the word stays as typed. */
    s.homing.flags.value = 0x7FF;
    /* $341: a sender's manual tool change (1) goes back to "ignore M6";
     * $673: a typed 3 s coolant on-delay goes back to none. */
    s.tool_change.mode = ToolChange_Manual;
    s.coolant.on_delay = 3000;
    s.axis[Z_AXIS].steps_per_mm = 999.0f;
    s.mode = Mode_Lathe;
    s.axis[X_AXIS].max_rate = 12000.0f;
    held = gfio_apply_machine_pins(&s, 32, 112640);
    expect_u("$63 typed to 7 is re-asserted as 3",
             (unsigned)(s.flags.disable_laser_during_hold | s.flags.restore_after_feed_hold << 1 |
                        s.flags.set_rpm_0_during_hold << 2), 3);
    expect_f("$34 typed to 10 is re-asserted", s.pwm_spindle.pwm_off_value, 0.0f, 0.0f);
    expect_u("$22 typed to 0x7FF keeps only the bits outside the homing cycle",
             s.homing.flags.value, 0x7FF & ~0x6Fu);
    expect_u("$341 typed to 1 is re-asserted as 4", (unsigned)s.tool_change.mode,
             (unsigned)ToolChange_Ignore);
    expect_u("$673 typed to 3 is re-asserted as 0", s.coolant.on_delay, 0);
    expect_f("$102 typed to 999 is re-asserted", s.axis[Z_AXIS].steps_per_mm, DEFAULT_Z_STEPS_PER_MM, 1e-5f);
    expect_u("$32 is re-asserted again", (unsigned)s.mode, (unsigned)Mode_Laser);
    expect_f("$100 follows the mode", s.axis[X_AXIS].steps_per_mm, 213.333f, 1e-4f);
    expect_u("nothing held at the mode's own tick", held, 0);

    /* The homing cycle masks: a $44 write leaves the Z hold. The core
     * recomputes the homing mask from $44-$46 after every write of one
     * and keeps an axis homed only while the union names it, so a typed
     * $44=1 (X alone), or all three typed 0, would drop the Z homed bit
     * that the lens hold rests on. The dispatch puts the driver's cycles
     * back before the core recomputes. */
    s.homing.cycle[0].mask = X_AXIS_BIT;
    s.homing.cycle[1].mask = 0;
    s.homing.cycle[2].mask = 0;
    gfio_apply_machine_pins(&s, 8, 28160);
    expect_u("$44 typed to 1 is re-asserted", s.homing.cycle[0].mask, DEFAULT_HOMING_CYCLE_0);
    expect_u("$45 typed to 0 is re-asserted", s.homing.cycle[1].mask, DEFAULT_HOMING_CYCLE_1);
    expect_u("$46 is re-asserted", s.homing.cycle[2].mask, DEFAULT_HOMING_CYCLE_2);
    expect_u("Z is in the union of the pinned cycles",
             (s.homing.cycle[0].mask | s.homing.cycle[1].mask | s.homing.cycle[2].mask) & Z_AXIS_BIT,
             Z_AXIS_BIT);
    s.homing.cycle[0].mask = 0;
    gfio_apply_machine_pins(&s, 32, 112640);
    expect_u("$44 typed to 0 is re-asserted", s.homing.cycle[0].mask, DEFAULT_HOMING_CYCLE_0);

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return EXIT_FAILURE;
    }
    printf("PASS xy_scale_test\n");
    return EXIT_SUCCESS;
}
