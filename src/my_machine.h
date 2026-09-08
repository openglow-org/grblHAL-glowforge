/*
  my_machine.h - build-time machine configuration

  Part of grblHAL-glowforge. Force-included into every translation unit
  (driver and grblHAL core alike) by CMake, so defines here override the
  core's #ifndef-guarded defaults in grbl/config.h.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "boards/glowforge.h"
