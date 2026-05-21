# Seeed MR24HPB1 mmWave Radar Sensor for ESPHome

This is an ESPHome external component for the Seeed MR24HPB1 mmWave radar sensor. The sensor provides presence detection, motion detection, and movement classification over UART.

## Supported Hardware

- [Seeed MR24HPB1 mmWave Presence Radar](https://www.seeedstudio.com/24GHz-mmWave-Radar-Sensor-Human-Static-Presence-Module-Lite-p-5524.html)

## Requirements

- ESPHome 2023.5.0 or higher
- ESP32 board with UART support

## Installation

### Using external_components in ESPHome configuration

Add the following to your ESPHome configuration:

```yaml
external_components:
  - source: github://alitheg/esphome-seeed-mr24hpb1@v1.0.1
```

## Configuration

Example configuration. All sensors are optional - wire up only the ones you want exposed:

```yaml
uart:
  id: uart_id
  tx_pin: GPIO43
  rx_pin: GPIO44
  baud_rate: 9600
  parity: NONE
  stop_bits: 1
  data_bits: 8
    
binary_sensor:
  - platform: template
    name: "Radar Presence"
    id: radar_presence
    device_class: occupancy
  - platform: template
    name: "Radar Motion"
    id: radar_motion
    device_class: motion

sensor:
  - platform: template
    name: "Radar Movement Percentage"
    id: radar_movement_pct
    unit_of_measurement: "%"
    accuracy_decimals: 1
  - platform: template
    name: "Radar Threshold Gear"
    id: radar_threshold_gear
    accuracy_decimals: 0
    entity_category: "diagnostic"

text_sensor:
  - platform: template
    name: "Radar Movement Class"
    id: radar_movement_class
  - platform: template
    name: "Radar Device ID"
    id: radar_device_id
    entity_category: "diagnostic"
  - platform: template
    name: "Radar Software Version"
    id: radar_software_version
    entity_category: "diagnostic"
  - platform: template
    name: "Radar Hardware Version"
    id: radar_hardware_version
    entity_category: "diagnostic"
  - platform: template
    name: "Radar Scene Mode"
    id: radar_scene_mode
    entity_category: "diagnostic"

seeed_mr24hpb1:
  id: mr24hpb1
  uart_id: uart_id
  presence_sensor: radar_presence
  motion_sensor: radar_motion
  movement_pct_sensor: radar_movement_pct
  threshold_gear_sensor: radar_threshold_gear
  movement_class_sensor: radar_movement_class
  device_id_sensor: radar_device_id
  software_version_sensor: radar_software_version
  hardware_version_sensor: radar_hardware_version
  scene_mode_sensor: radar_scene_mode
```

## Available Sensors

### Binary Sensors
- **Radar Presence**: Indicates if a person is present in the detection area
- **Radar Motion**: Indicates if motion is detected

### Sensors
- **Movement Percentage**: Shows the intensity of movement (0-100%)
- **Threshold Gear**: Current sensitivity setting of the radar

### Text Sensors
- **Movement Class**: Classification of detected movement (unoccupied, resting, micro-movement, walking, running)
- **Device ID**: Hardware identifier of the radar module (reads "unset" if the module has none programmed)
- **Software Version**: Firmware version of the radar module
- **Hardware Version**: Hardware revision of the radar module
- **Scene Mode**: Current scene setting (Default, Area Detection, Bathroom, Bedroom, Living Room, Office, Hotel)

## Hardware Setup

1. Connect the MR24HPB1 radar sensor to your ESP32:
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