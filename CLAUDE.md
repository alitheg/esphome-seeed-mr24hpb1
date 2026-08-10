# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An ESPHome **external component** for the Seeed MR24HPB1 24GHz mmWave presence radar, communicating over UART. It is a library packaged for consumption via `external_components:` in a user's ESPHome config — there is no standalone application here.

## Commands

There is no unit test suite. Verification is done by running ESPHome against `test.yaml`, which mirrors what CI (`.github/workflows/ci.yaml`) does:

```bash
pip install esphome          # one-time setup
esphome config test.yaml     # validate YAML + run Python codegen (fast, no compiler)
esphome compile test.yaml    # full C++ firmware build for esp32-s3 (slow; the real check)
```

`esphome config` exercises the platform schemas (`__init__.py` and the per-platform `*.py`); `esphome compile` additionally builds the C++ in `seeed_mr24hpb1.cpp/.h`. Run `config` for quick iteration on the schema, `compile` to catch C++ errors.

Note: `esphome compile` exits non-zero at the very end unless the PlatformIO CLI (`pio`) is on PATH. The `export_compdb.py` post-build hook — wired in via `test.yaml`'s `platformio_options` for SonarCloud — shells out to `pio` and dies with `command not found`. The firmware itself builds fine just before that ("Successfully created ESP32-S3 image"); only the compile_commands.json export is lost.

Releases are tag-driven: pushing a `v*` tag triggers `.github/workflows/release.yaml` (GitHub release only — nothing is published to a package registry).

## Architecture

The component is a **hub plus sub-platforms**. `seeed_mr24hpb1:` declares the hub (UART + polling); each ESPHome platform (`binary_sensor`, `sensor`, `text_sensor`, `select`, `number`, `button`) declares only the entities you want, and the component **creates** them. The user does not wire in their own `template` sensors — that was the v1 model, replaced in v2 (see the README's "Migrating from v1"). `test.yaml` is the source of truth for a valid config.

Two layers that must stay in sync:

1. **Python codegen** — `__init__.py` defines the hub schema and the `MR24HPB1` class binding; one file per platform (`binary_sensor.py`, `sensor.py`, `text_sensor.py`, `select.py`, `number.py`, `button.py`) lists that platform's entities as `cv.Optional` keys. Each `to_code()` creates every present entity (e.g. `text_sensor.new_text_sensor(...)`) and hands it to the hub via a `set_*` setter. Everything is optional — omit any block or key for entities you don't want.
2. **C++ runtime** (`seeed_mr24hpb1.cpp` / `.h`) — `MR24HPB1` extends `PollingComponent` + `uart::UARTDevice`. `loop()` reassembles UART frames; `parse_frame_()` decodes them and publishes to whichever entities were registered (each `*_{nullptr}` member is null-checked before publish). `update()` polls the module for scene / sensitivity / versions.

Adding a **read** entity touches both layers: a `CONF_*` key + `new_*` + `set_*` call in the platform `.py`, and a matching `set_*_sensor()` setter + member + `parse_frame_()` branch in the C++. A **control** entity (select/number/button) instead adds a `write_*`/action method plus a small `Parented<MR24HPB1>` subclass in the `.h` whose `control()`/`press_action()` calls it.

### UART frame protocol

Frames begin with `0x55`; bytes `[1]`/`[2]` are a little-endian length covering everything after that header. `loop()` buffers until a full frame arrives, discarding on a declared length > 256 and resyncing on > 128 bytes buffered (clearing outright after 5 consecutive overruns). `parse_frame_()` dispatches on the `(addr1, addr2)` pair at `bytes[4]`/`bytes[5]` and ignores the function code, so a passive (`0x03`) or proactive (`0x04`) report of the same address lands in the same branch:

| addr1 / addr2 | Meaning | Published to |
|---------------|---------|--------------|
| `0x03` / `0x05` | presence + motion flags | presence, motion binary_sensors |
| `0x03` / `0x06` | movement % (IEEE-754 float at bytes[6..9]) | movement_pct, movement_class |
| `0x03` / `0x07` | approach/away direction (code at bytes[8]) | movement_direction |
| `0x01` / `0x01` | device ID (ASCII) | device_id |
| `0x01` / `0x02` | software version (ASCII) | software_version |
| `0x01` / `0x03` | hardware version (ASCII) | hardware_version |
| `0x04` / `0x0C` | threshold gear | sensitivity number |
| `0x04` / `0x10` | scene mode (enum index) | scene_mode select |
| `0x05` / `0x01` | heartbeat | (swallowed, VERBOSE log only) |

`movement_class` is derived from the percentage in C++ (unoccupied / resting / micro-movement / walking / running). `movement_direction` maps the direction byte to none / approach / away / sustained-approach / sustained-away — the module's only directional cue, useful for telling someone walking in from someone crossing the doorway.

Writes go the other way: `write_scene_mode()` / `write_sensitivity()` / `reboot()` build frames via `send_command()` with a CRC16/Modbus checksum (`crc16()`, reflected poly `0xA001`). The control entities (`SceneSelect`, `SensitivityNumber`, `RebootButton`, defined in the `.h`) call these from their `control()`/`press_action()`.

### Tooling note

`export_compdb.py` is a PlatformIO post-build hook (referenced from `test.yaml`) that emits `compile_commands.json` for SonarCloud C/C++ analysis; it needs `pio` on PATH (see Commands). The Sonar scan steps in `ci.yaml` are currently commented out.
