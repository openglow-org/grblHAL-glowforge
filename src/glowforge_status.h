/*
  glowforge_status.h - publish the controller's state for the machine
  daemon (see glowforge_status.c)

  Part of grblHAL-glowforge.
  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

// Register the settings-changed hook and resolve the publish directory
// (GF_STATE_DIR, default /run/forgefirm). Called from driver_init().
void gfstatus_init (void);

// Periodic work on the protocol thread (realtime hook): publish
// grbl.settings when a setting changed and grbl.state on a state,
// sender, laser or modal edge, plus a slow heartbeat.
void gfstatus_poll (void);
