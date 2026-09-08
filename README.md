# grblHAL-glowforge

A [grblHAL](https://github.com/grblHAL) driver for the **stock Glowforge
(Basic, Plus, Pro) control board**: the factory NXP i.MX6 SOM running Linux.
It is the GRBL-mode controller of
[ForgeFIRM](https://github.com/openglow-org/forgefirm), which replaces the
cloud-dependent factory firmware with an open, locally controlled image, with
no hardware modification.

The unmodified grblHAL core (a git submodule at `src/grbl`) runs as a Linux
userspace process. Steps are not fired from a GPIO interrupt handler: the
driver streams **pulse bytes**, one byte per machine tick, into the kernel
module's SDMA and EPIT playback engine. That is the same jitter-free hardware
step generator the factory firmware used, fed live from grblHAL's planner
instead of from a cloud-generated file.

The machine constants (steps per millimeter, maximum rates, accelerations) are
measured from the factory machine and its own pulse streams;
`src/boards/glowforge.h` names the sources.

## Documentation

Everything is on **<https://docs.forgefirm.org/>**. This README is an index
card.

| Subject | Page |
|---|---|
| This driver: the threads, G-code to pulse bytes, the laser path, the armed window, the lid and button, faults, the cooling client | [The grblHAL driver](https://docs.forgefirm.org/technical/forgefirm/grblhal-driver/) |
| The byte format and the playback engine | [The step engine](https://docs.forgefirm.org/technical/machine/step-engine/) |
| What a feeder must obey | [Pulse feeder contract](https://docs.forgefirm.org/technical/forgefirm/pulse-feeder-contract/) |
| The laser hardware and the tube's own thresholds | [The laser](https://docs.forgefirm.org/technical/machine/laser/) |
| Geometry, speeds, limits, the lens | [Motion hardware](https://docs.forgefirm.org/technical/machine/motion-hardware/) |
| Building it, and every environment variable | [Build](https://docs.forgefirm.org/developers/building/) |
| Connecting a sender | [GRBL mode](https://docs.forgefirm.org/usage/grbl-mode/), [LightBurn](https://docs.forgefirm.org/usage/lightburn/) |

## Build and test

```sh
cmake -B build && cmake --build build
./build/switch_map_test
./build/laser_arm_test
```

On a host without `GFSINK` the driver runs the real core, planner and stream
code against a **null sink**, which is what makes the laser path testable
without hardware. Two harnesses drive that build over TCP and live in the
`forgefirm` repository, under `scripts/bench/`:

```sh
python3 laser_stream_test.py build/grblHAL_glowforge
python3 laser_lifecycle_test.py build/grblHAL_glowforge
```

The board binary is a cross-build with the i.MX6 toolchain:
`forgefirm/scripts/bench/build-glowforge.sh`.

## Run it on the board

Under the ForgeFIRM image the driver is a supervised child of `forgectrl`,
which hands it the pulse device. You do not start it yourself. For bench work
it runs standalone once the supervisor has released the device:

```sh
GFSINK=/dev/glowforge grblHAL_glowforge -p 23 -e /data/forgefirm/EEPROM-glowforge.DAT
```

## Safety

The first laser-on of a job requires the **operator's physical button press**.
The kernel laser latch stays locked until then and relocks on disarm, alarm
and reset; fire only ever rides the motion segments of laser blocks; and an
armed underrun fails safe. The hardware safety chain is authoritative
regardless of anything this driver does.

## Contributing

[AGENTS.md](AGENTS.md) carries the rules for this repository and for the
project. They apply to human contributors too.

## Lineage and license

Derived from the [grblHAL Simulator](https://github.com/grblHAL/Simulator):
the platform layer, and the shape of the stream and NVS code.
GPL-3.0-or-later; see `COPYING`. The grblHAL core is copyright Terje Io and
contributors; the Simulator platform code is copyright Jens Geisler and Adam
Shelly; the Glowforge driver is copyright 514 LLC d/b/a OpenGlow.
