# Settings read/write via hub + platform model (v2.0.0)

- Date: 2026-05-21
- Status: Approved (design)
- Target release: v2.0.0 (breaking change)

## Summary

Rework the `seeed_mr24hpb1` ESPHome component from its current "bring your own
`template` sensor and wire its ID" model into a standard hub + platform
component. The hub owns the UART, frame parsing and command sending; entities are
created by sub-platforms (`binary_sensor`, `sensor`, `text_sensor`, `select`,
`number`, `button`). This adds the ability to *set* device settings (scene mode,
sensitivity, reboot) from Home Assistant via real two-way control entities, and
makes everything appear by declaring the platform blocks rather than hand-wiring
template sensors.

## Background / current state

- Reads work: the device proactively reports presence, motion and movement, and
  responds to read queries (`0x01`) for scene mode, threshold gear (sensitivity),
  device ID and software/hardware versions. All confirmed on-device.
- The component does not create its own entities. The user declares `template`
  sensors and passes their IDs in via `*_sensor` keys (optional since v1.0.1).
- There is no write capability. `from esphome import automation` is imported but
  unused. `send_command(fn, addr1, addr2, data)` already builds CRC-framed
  commands, so issuing writes is straightforward.

## Goals

- Set scene mode, sensitivity and reboot from Home Assistant.
- Two-way controls: the `select`/`number` show the device's current value and set
  it.
- Entities appear by declaring `platform: seeed_mr24hpb1` blocks - no template
  sensors, no ID wiring, no user lambdas required for normal use.
- Auto-populate read-only and control values without the user writing an
  `interval:` query block.

## Non-goals

- Backward compatibility with the v1 config schema (clean break; sole user).
- Forced-unoccupied timer control (dropped as niche).
- OTA-over-protocol, approach/away status, heartbeat handling (out of scope).

## Architecture

`seeed_mr24hpb1:` is the hub: a `PollingComponent` + `uart::UARTDevice` that owns
frame parsing, command sending, periodic polling, and pointers to its registered
entities. Each entity is created by a sub-platform whose `to_code` instantiates
the entity and registers it with the hub.

## Entities

| Entity | Platform type | Direction | Notes |
|--------|---------------|-----------|-------|
| Presence | binary_sensor | read | |
| Motion | binary_sensor | read | |
| Movement % | sensor | read | |
| Movement Class | text_sensor | read | |
| Device ID | text_sensor | read | `unset` when blank (all `0xFF`) |
| Software Version | text_sensor | read | |
| Hardware Version | text_sensor | read | |
| Scene Mode | select | read + write | options = 7 scenes; option index = protocol byte |
| Sensitivity | number | read + write | min 1, max 3, step 1, slider mode |
| Reboot | button | write | sends reboot command |

Scene mode and sensitivity are no longer separate read-only entities - the
`select`/`number` both display the current value and set it.

## Config (new shape)

```yaml
seeed_mr24hpb1:
  id: mr24hpb1
  uart_id: uart_id
  update_interval: 60s   # re-polls scene + sensitivity; versions queried until received

binary_sensor:
  - platform: seeed_mr24hpb1
    presence: { name: "Radar Presence" }
    motion:   { name: "Radar Motion" }

sensor:
  - platform: seeed_mr24hpb1
    movement_pct: { name: "Radar Movement %" }

text_sensor:
  - platform: seeed_mr24hpb1
    movement_class:   { name: "Radar Movement Class" }
    device_id:        { name: "Radar Device ID" }
    software_version: { name: "Radar Software Version" }
    hardware_version: { name: "Radar Hardware Version" }

select:
  - platform: seeed_mr24hpb1
    scene_mode: { name: "Radar Scene Mode" }

number:
  - platform: seeed_mr24hpb1
    sensitivity: { name: "Radar Sensitivity" }

button:
  - platform: seeed_mr24hpb1
    reboot: { name: "Radar Reboot" }
```

Every entity is optional - declare only the platforms and keys you want. Each
platform resolves the hub automatically when there is one instance, or via an
explicit `seeed_mr24hpb1_id:` when there are several.

## Migration (v1 -> v2)

The v1 pattern (declare `template` sensors, then list their IDs under
`seeed_mr24hpb1:`) is removed. Before:

```yaml
text_sensor:
  - platform: template
    id: radar_scene_mode
seeed_mr24hpb1:
  scene_mode_sensor: radar_scene_mode
```

After: the `scene_mode` becomes a `select` under `platform: seeed_mr24hpb1` (see
config above). README and release notes carry the full before/after example.
`office-presence.yaml` is rewritten to the new shape as part of rollout, and its
`interval:` query block is removed (the hub polls automatically).

## Data flow

- **Read**: device report -> hub `parse_frame_` -> `publish_state` on the
  registered entity, if present. The all-`0xFF` device ID still maps to `unset`.
- **Write**: HA changes a control -> the entity's `control()` /
  `press_action()` calls a hub method -> the hub builds an `0x02` (or reboot)
  frame via `send_command`. The device then reports the new value, which updates
  the entity, closing the loop.
- **Poll**: the hub is a `PollingComponent`. Each `update()` queries scene mode
  and sensitivity. Software version, hardware version and device ID are queried
  on each `update()` until each has been received once, then dropped. This
  removes the need for a user `interval:` block. The existing public
  `send_*_query()` methods stay available for manual use.

## Protocol commands

| Action | fn | addr1 | addr2 | data |
|--------|----|-------|-------|------|
| Scene write | `0x02` | `0x04` | `0x10` | scene byte `0x00`-`0x06` |
| Sensitivity write | `0x02` | `0x04` | `0x0C` | gear byte `0x01`-`0x03` |
| Reboot | `0x02` | `0x05` | `0x04` | none |

Scene byte mapping (option index = byte): `0` Default, `1` Area Detection,
`2` Bathroom, `3` Bedroom, `4` Living Room, `5` Office, `6` Hotel. This single
canonical list is shared between the select's options and the report decoder so
they cannot drift.

## C++ class design

- `MR24HPB1` (hub): `public PollingComponent, public uart::UARTDevice`.
  - Lifecycle: `setup()`, `loop()` (existing UART read/parse), `update()`
    (polling), `dump_config()`.
  - Registration setters, one per entity, called from the platforms'
    `to_code` (e.g. `set_presence_binary_sensor`, `set_scene_select`,
    `set_sensitivity_number`, ...).
  - Write helpers: `write_scene_mode(uint8_t mode)`,
    `write_sensitivity(uint8_t gear)`, `reboot()`.
  - Keeps the existing `send_*_query()` methods and `send_command`/`crc16`.
- `SceneSelect : public select::Select, public Parented<MR24HPB1>` -
  `control(const std::string &value)` maps the option to its index and calls
  `parent_->write_scene_mode(index)`.
- `SensitivityNumber : public number::Number, public Parented<MR24HPB1>` -
  `control(float value)` calls `parent_->write_sensitivity()` with the rounded
  value. Registered with traits min 1, max 3, step 1 and `mode: slider`.
- `RebootButton : public button::Button, public Parented<MR24HPB1>` -
  `press_action()` calls `parent_->reboot()`.

On report, the hub publishes back to whichever control is registered:
scene report -> `scene_select_->publish_state(name)`, gear report ->
`sensitivity_number_->publish_state(gear)`.

## File structure

```
components/seeed_mr24hpb1/
  __init__.py              # hub schema + to_code
  seeed_mr24hpb1.{h,cpp}   # hub: parse, send, poll, entity pointers, write methods
  binary_sensor.py
  sensor.py
  text_sensor.py
  select.py
  number.py
  button.py
  scene_select.{h,cpp}
  sensitivity_number.{h,cpp}
  reboot_button.{h,cpp}
```

## Versioning

Release as `v2.0.0`. No compatibility shim. README install pin and example
updated; release notes include the migration example.

## Testing

- `test.yaml` rewritten to the new platform config (all platforms exercised).
- `esphome config test.yaml` validates.
- `esphome compile test.yaml` builds clean (firmware image produced).
- On-device: scene mode and sensitivity read back after boot; changing the
  `select`/`number` in HA changes the radar and the entity reflects it; reboot
  button resets the module.
