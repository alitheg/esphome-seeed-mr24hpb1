# Seeed MR24HPB1 mmWave Radar Sensor for ESPHome

An ESPHome external component for the Seeed MR24HPB1 mmWave radar. It exposes presence and motion detection, movement classification, two-way control of scene mode and sensitivity, and a reboot button, all over UART.

## Supported Hardware

- [Seeed MR24HPB1 mmWave Presence Radar](https://www.seeedstudio.com/24GHz-mmWave-Radar-Sensor-Human-Static-Presence-Module-Lite-p-5524.html)

## Requirements

- ESPHome 2023.5.0 or higher
- ESP32 board with UART support

## Installation

Add the following to your ESPHome configuration:

```yaml
external_components:
  - source: github://alitheg/esphome-seeed-mr24hpb1@v2.0.0
```

## Configuration

The component is a hub plus sub-platforms. Declare the hub, then declare only the entities you actually want under each platform - everything is optional.

```yaml
uart:
  id: uart_id
  tx_pin: GPIO43
  rx_pin: GPIO44
  baud_rate: 9600
  parity: NONE
  stop_bits: 1
  data_bits: 8

seeed_mr24hpb1:
  id: mr24hpb1
  uart_id: uart_id
  update_interval: 60s

binary_sensor:
  - platform: seeed_mr24hpb1
    presence:
      name: "Radar Presence"
    motion:
      name: "Radar Motion"

sensor:
  - platform: seeed_mr24hpb1
    movement_pct:
      name: "Radar Movement Percentage"

text_sensor:
  - platform: seeed_mr24hpb1
    movement_class:
      name: "Radar Movement Class"
    device_id:
      name: "Radar Device ID"
    software_version:
      name: "Radar Software Version"
    hardware_version:
      name: "Radar Hardware Version"

select:
  - platform: seeed_mr24hpb1
    scene_mode:
      name: "Radar Scene Mode"

number:
  - platform: seeed_mr24hpb1
    sensitivity:
      name: "Radar Sensitivity"

button:
  - platform: seeed_mr24hpb1
    reboot:
      name: "Radar Reboot"
```

`update_interval` controls how often the hub re-polls the device for the current scene mode and sensitivity. Versions and the device ID are queried once at startup.

## Available Entities

### Binary sensors
- **presence**: someone is present in the detection area
- **motion**: motion is detected

### Sensor
- **movement_pct**: intensity of detected movement (0-100%)

### Text sensors
- **movement_class**: classification derived from the percentage - unoccupied, resting, micro-movement, walking, running
- **device_id**: the module's device ID (reads `unset` if none is programmed)
- **software_version**: firmware version reported by the module
- **hardware_version**: hardware revision reported by the module

### Select
- **scene_mode** (read + write): Default, Area Detection, Bathroom, Bedroom, Living Room, Office, Hotel. Changing it writes the new scene to the device, which reports it back to confirm.

### Number
- **sensitivity** (read + write): 1-3, slider. The radar's internal threshold gear - higher is more sensitive.

### Button
- **reboot**: restarts the radar module.

## Migrating from v1

v2 is a breaking change to the YAML schema. v1 made you declare `template` sensors and wire their IDs into the component; v2 declares its own entities via sub-platforms.

Before (v1):

```yaml
text_sensor:
  - platform: template
    name: "Radar Scene Mode"
    id: radar_scene_mode

seeed_mr24hpb1:
  id: mr24hpb1
  uart_id: uart_id
  scene_mode_sensor: radar_scene_mode
```

After (v2):

```yaml
seeed_mr24hpb1:
  id: mr24hpb1
  uart_id: uart_id
  update_interval: 60s

select:
  - platform: seeed_mr24hpb1
    scene_mode:
      name: "Radar Scene Mode"
```

Notes on the migration:
- The v1 read-only `threshold_gear_sensor` and `scene_mode_sensor` are gone. Sensitivity is now a two-way `number`, scene mode a two-way `select`.
- If you had an `interval:` block calling `send_*_query()` lambdas to populate values, drop it - the hub polls automatically on `update_interval`.
- All entities are optional; omit any sub-platform block or key for entities you don't want exposed.

## Hardware Setup

Connect the MR24HPB1 radar sensor to your ESP32:
- RX pin → ESP32 TX pin (GPIO43 in example)
- TX pin → ESP32 RX pin (GPIO44 in example)
- VCC → 5V
- GND → GND

## Troubleshooting

If you're not getting any readings:
1. Check the UART wiring (TX/RX might need to be swapped)
2. Verify the baud rate matches your radar module's settings (default is 9600)
3. Ensure the radar module is powered correctly

## Acknowledgements

- Thanks to the [ESPHome](https://esphome.io/) team for their excellent framework
- Project structure inspired by other ESPHome external components in the community, especially [j5lien's Idasen desk controller](https://github.com/j5lien/esphome-idasen-desk-controller)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
