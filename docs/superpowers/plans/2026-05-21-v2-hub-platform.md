# v2 Hub + Platform Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the `seeed_mr24hpb1` ESPHome component to a hub + platform design with two-way `select`/`number`/`button` controls for scene mode, sensitivity and reboot.

**Architecture:** A hub component (`PollingComponent` + `uart::UARTDevice`) owns UART parsing, command sending and polling, and holds pointers to entities created by sub-platforms (`binary_sensor`, `sensor`, `text_sensor`, `select`, `number`, `button`). Controls call hub write methods; the hub publishes device reports back to the entities.

**Tech Stack:** ESPHome external component (Python codegen + C++), ESP32-S3, UART.

**Spec:** `docs/superpowers/specs/2026-05-21-settings-read-write-hub-platform-design.md`

**Branch:** `feat/v2-hub-platform` (already checked out).

**Testing note (read first):** This is an ESPHome component; there is no unit-test harness. The "test" for each task is `esphome config` (schema/codegen) and, where C++ changed, `esphome compile` (the firmware build). Use the project venv: `.venv-test/bin/esphome`. For compile, prefix `PATH="$PWD/.venv-test/bin:$PATH"` so the `pio` post-step resolves. On-device behaviour is verified by the user at the end.

**Implementation note:** All C++ classes (hub + the three control classes) live in the single header `seeed_mr24hpb1.h`. This deviates from the spec's separate-file list on purpose: ESPHome reliably includes a component's primary `<name>.h`, so co-locating the small control classes there avoids multi-header include resolution. Behaviour is unchanged.

---

### Task 1: Rewrite the hub C++ header

**Files:**
- Modify (replace whole file): `components/seeed_mr24hpb1/seeed_mr24hpb1.h`

- [ ] **Step 1: Replace the header with the v2 hub + control classes**

```cpp
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"  // Parented
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"

#include <vector>

namespace esphome {
namespace seeed_mr24hpb1 {

class MR24HPB1 : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  // Read-entity registration (called from platforms)
  void set_presence_binary_sensor(binary_sensor::BinarySensor *s) { presence_binary_sensor_ = s; }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *s) { motion_binary_sensor_ = s; }
  void set_movement_pct_sensor(sensor::Sensor *s) { movement_pct_sensor_ = s; }
  void set_movement_class_text_sensor(text_sensor::TextSensor *s) { movement_class_text_sensor_ = s; }
  void set_device_id_text_sensor(text_sensor::TextSensor *s) { device_id_text_sensor_ = s; }
  void set_software_version_text_sensor(text_sensor::TextSensor *s) { software_version_text_sensor_ = s; }
  void set_hardware_version_text_sensor(text_sensor::TextSensor *s) { hardware_version_text_sensor_ = s; }

  // Control-entity registration
  void set_scene_select(select::Select *s) { scene_select_ = s; }
  void set_sensitivity_number(number::Number *n) { sensitivity_number_ = n; }

  // Writes (called from control entities)
  void write_scene_mode(uint8_t mode);
  void write_sensitivity(uint8_t gear);
  void reboot();

  // Read queries (kept for manual use / polling)
  void send_scene_query();
  void send_threshold_gear_query();
  void send_software_version_query();
  void send_hardware_version_query();
  void send_device_id_query();

 protected:
  void parse_frame_(std::vector<uint8_t> &bytes);
  void send_command(uint8_t fn, uint8_t addr1, uint8_t addr2);
  void send_command(uint8_t fn, uint8_t addr1, uint8_t addr2, const std::vector<uint8_t> &data);
  uint16_t crc16(const uint8_t *data, size_t length);

  std::vector<uint8_t> buffer_;
  uint16_t expected_length_{0};

  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
  sensor::Sensor *movement_pct_sensor_{nullptr};
  text_sensor::TextSensor *movement_class_text_sensor_{nullptr};
  text_sensor::TextSensor *device_id_text_sensor_{nullptr};
  text_sensor::TextSensor *software_version_text_sensor_{nullptr};
  text_sensor::TextSensor *hardware_version_text_sensor_{nullptr};
  select::Select *scene_select_{nullptr};
  number::Number *sensitivity_number_{nullptr};

  bool got_software_version_{false};
  bool got_hardware_version_{false};
  bool got_device_id_{false};
};

class SceneSelect : public select::Select, public Parented<MR24HPB1> {
 protected:
  void control(const std::string &value) override {
    auto idx = this->index_of(value);
    if (idx.has_value())
      this->parent_->write_scene_mode(static_cast<uint8_t>(idx.value()));
  }
};

class SensitivityNumber : public number::Number, public Parented<MR24HPB1> {
 protected:
  void control(float value) override {
    this->parent_->write_sensitivity(static_cast<uint8_t>(lroundf(value)));
  }
};

class RebootButton : public button::Button, public Parented<MR24HPB1> {
 protected:
  void press_action() override { this->parent_->reboot(); }
};

}  // namespace seeed_mr24hpb1
}  // namespace esphome
```

- [ ] **Step 2: Commit**

```bash
git add components/seeed_mr24hpb1/seeed_mr24hpb1.h
git commit -m "Rewrite hub header for hub+platform model"
```

Note: this does not compile until Task 2 and Task 3 land (the .cpp and Python still reference the old API). Verification happens at Task 4.

---

### Task 2: Rewrite the hub C++ implementation

**Files:**
- Modify (replace whole file): `components/seeed_mr24hpb1/seeed_mr24hpb1.cpp`

- [ ] **Step 1: Replace the implementation**

```cpp
#include "esphome/core/log.h"
#include "seeed_mr24hpb1.h"
#include <cmath>

namespace esphome {
namespace seeed_mr24hpb1 {

static const char *const TAG = "seeed_mr24hpb1";

void MR24HPB1::setup() {
  ESP_LOGI(TAG, "Setup started");
  this->buffer_.clear();
  // Populate versions / device id / scene / sensitivity shortly after boot
  // instead of waiting a full update_interval.
  this->set_timeout(3000, [this]() { this->update(); });
}

void MR24HPB1::update() {
  // Stagger queries so the module is not flooded in one tick.
  uint32_t delay = 0;
  this->set_timeout(delay, [this]() { this->send_scene_query(); });
  delay += 200;
  this->set_timeout(delay, [this]() { this->send_threshold_gear_query(); });
  delay += 200;
  if (!this->got_software_version_) {
    this->set_timeout(delay, [this]() { this->send_software_version_query(); });
    delay += 200;
  }
  if (!this->got_hardware_version_) {
    this->set_timeout(delay, [this]() { this->send_hardware_version_query(); });
    delay += 200;
  }
  if (!this->got_device_id_) {
    this->set_timeout(delay, [this]() { this->send_device_id_query(); });
  }
}

void MR24HPB1::loop() {
  static int overrun_count = 0;

  while (available()) {
    uint8_t byte;
    read_byte(&byte);

    if (buffer_.empty() && byte != 0x55)
      continue;

    buffer_.push_back(byte);

    if (buffer_.size() == 3) {
      // Length field covers everything except the 0x55 header (manual 8.1.2),
      // so the full frame is that value plus the one header byte.
      expected_length_ = static_cast<size_t>(buffer_[1]) +
                         (static_cast<size_t>(buffer_[2]) << 8) + 1;
      if (expected_length_ > 256) {
        ESP_LOGW(TAG, "Payload length too large (%d) - discarding", expected_length_);
        buffer_.clear();
        expected_length_ = 0;
        continue;
      }
      ESP_LOGD(TAG, "Expecting frame of length %d", expected_length_);
    }

    if (expected_length_ > 0 && buffer_.size() >= expected_length_) {
      std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + expected_length_);
      parse_frame_(frame);
      buffer_.erase(buffer_.begin(), buffer_.begin() + expected_length_);
      expected_length_ = 0;
      overrun_count = 0;
    }

    if (buffer_.size() > 128) {
      ESP_LOGW(TAG, "Buffer overrun (%d bytes) - attempting resync", buffer_.size());
      auto it = std::find(buffer_.begin() + 1, buffer_.end(), 0x55);
      if (it != buffer_.end()) {
        buffer_.erase(buffer_.begin(), it);
      } else {
        buffer_.clear();
      }
      expected_length_ = 0;
      if (++overrun_count >= 5) {
        buffer_.clear();
        overrun_count = 0;
      }
    }
  }
  yield();
}

void MR24HPB1::parse_frame_(std::vector<uint8_t> &bytes) {
  size_t len = bytes.size();
  if (len < 7 || bytes[0] != 0x55) {
    ESP_LOGW(TAG, "Invalid frame: too short or missing header (len=%d)", len);
    return;
  }

  uint8_t fn = bytes[3];
  uint8_t addr1 = bytes[4];
  uint8_t addr2 = bytes[5];
  ESP_LOGD(TAG, "Parsing frame: fn=0x%02X addr1=0x%02X addr2=0x%02X (len=%d)", fn, addr1, addr2, len);

  if (addr1 == 0x03 && addr2 == 0x05 && len >= 10) {
    bool presence = bytes[6] == 0x01;
    bool motion = bytes[7] == 0x01;
    if (presence_binary_sensor_) presence_binary_sensor_->publish_state(presence);
    if (motion_binary_sensor_) motion_binary_sensor_->publish_state(motion);

  } else if (addr1 == 0x03 && addr2 == 0x06 && len >= 11) {
    union { uint8_t b[4]; float f; } val;
    for (int i = 0; i < 4; i++) val.b[i] = bytes[6 + i];
    float pct = val.f;
    const char *cls = "unknown";
    if (pct < 1.0f) cls = "unoccupied";
    else if (pct < 2.0f) cls = "resting";
    else if (pct <= 30.0f) cls = "micro-movement";
    else if (pct <= 60.0f) cls = "walking";
    else cls = "running";
    if (movement_pct_sensor_) movement_pct_sensor_->publish_state(pct);
    if (movement_class_text_sensor_) movement_class_text_sensor_->publish_state(cls);

  } else if (addr1 == 0x01 && addr2 == 0x01 && len >= 19) {
    std::string idstr;
    bool has_printable = false;
    for (size_t i = 6; i < len - 2; i++) {
      char c = static_cast<char>(bytes[i]);
      if (c >= 32 && c <= 126) { idstr += c; has_printable = true; }
      else idstr += '.';
    }
    if (!has_printable) idstr = "unset";
    got_device_id_ = true;
    if (device_id_text_sensor_) device_id_text_sensor_->publish_state(idstr);

  } else if (addr1 == 0x01 && addr2 == 0x02 && len >= 22) {
    std::string ver;
    for (size_t i = 6; i < len - 2; i++)
      if (bytes[i] != 0 && bytes[i] != 0xFF) ver += static_cast<char>(bytes[i]);
    got_software_version_ = true;
    if (software_version_text_sensor_) software_version_text_sensor_->publish_state(ver);

  } else if (addr1 == 0x01 && addr2 == 0x03 && len >= 8) {
    std::string ver;
    for (size_t i = 6; i < len - 2; i++)
      if (bytes[i] != 0 && bytes[i] != 0xFF) ver += static_cast<char>(bytes[i]);
    got_hardware_version_ = true;
    if (hardware_version_text_sensor_) hardware_version_text_sensor_->publish_state(ver);

  } else if (addr1 == 0x04 && addr2 == 0x0C && len >= 8) {
    uint8_t gear = bytes[6];
    if (sensitivity_number_) sensitivity_number_->publish_state(static_cast<float>(gear));

  } else if (addr1 == 0x04 && addr2 == 0x10 && len >= 8) {
    uint8_t mode = bytes[6];
    if (scene_select_) {
      auto opts = scene_select_->traits.get_options();
      if (mode < opts.size()) scene_select_->publish_state(opts[mode]);
    }

  } else {
    ESP_LOGW(TAG, "Unhandled frame fn=0x%02X addr1=0x%02X addr2=0x%02X (len=%d)", fn, addr1, addr2, len);
  }
}

void MR24HPB1::send_command(uint8_t fn, uint8_t addr1, uint8_t addr2) {
  std::vector<uint8_t> empty;
  send_command(fn, addr1, addr2, empty);
}

void MR24HPB1::send_command(uint8_t fn, uint8_t addr1, uint8_t addr2, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> frame;
  frame.push_back(0x55);
  uint16_t length = 7 + data.size();
  frame.push_back(length & 0xFF);
  frame.push_back((length >> 8) & 0xFF);
  frame.push_back(fn);
  frame.push_back(addr1);
  frame.push_back(addr2);
  frame.insert(frame.end(), data.begin(), data.end());
  uint16_t crc = crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);
  ESP_LOGD(TAG, "Sending frame: fn=0x%02X addr1=0x%02X addr2=0x%02X data_len=%d crc=0x%04X",
           fn, addr1, addr2, (int) data.size(), crc);
  this->write_array(frame);
}

void MR24HPB1::write_scene_mode(uint8_t mode) {
  ESP_LOGI(TAG, "Writing scene mode %d", mode);
  this->send_command(0x02, 0x04, 0x10, {mode});
}

void MR24HPB1::write_sensitivity(uint8_t gear) {
  ESP_LOGI(TAG, "Writing sensitivity %d", gear);
  this->send_command(0x02, 0x04, 0x0C, {gear});
}

void MR24HPB1::reboot() {
  ESP_LOGI(TAG, "Rebooting radar");
  this->send_command(0x02, 0x05, 0x04);
}

void MR24HPB1::send_software_version_query() { this->send_command(0x01, 0x01, 0x02); }
void MR24HPB1::send_hardware_version_query() { this->send_command(0x01, 0x01, 0x03); }
void MR24HPB1::send_device_id_query() { this->send_command(0x01, 0x01, 0x01); }
void MR24HPB1::send_scene_query() { this->send_command(0x01, 0x04, 0x10); }
void MR24HPB1::send_threshold_gear_query() { this->send_command(0x01, 0x04, 0x0C); }

uint16_t MR24HPB1::crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void MR24HPB1::dump_config() {
  ESP_LOGCONFIG(TAG, "Seeed MR24HPB1 UART mmWave radar");
}

}  // namespace seeed_mr24hpb1
}  // namespace esphome
```

- [ ] **Step 2: Commit**

```bash
git add components/seeed_mr24hpb1/seeed_mr24hpb1.cpp
git commit -m "Rewrite hub implementation: polling, writes, control publish-back"
```

---

### Task 3: Rewrite the hub Python (`__init__.py`)

**Files:**
- Modify (replace whole file): `components/seeed_mr24hpb1/__init__.py`

- [ ] **Step 1: Replace with the hub schema**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@alitheg"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor", "select", "number", "button"]

CONF_SEEED_MR24HPB1_ID = "seeed_mr24hpb1_id"

seeed_mr24hpb1_ns = cg.esphome_ns.namespace("seeed_mr24hpb1")
MR24HPB1 = seeed_mr24hpb1_ns.class_("MR24HPB1", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(MR24HPB1)})
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
```

- [ ] **Step 2: Commit**

```bash
git add components/seeed_mr24hpb1/__init__.py
git commit -m "Rewrite hub Python schema as PollingComponent"
```

---

### Task 4: binary_sensor platform (presence, motion) + first compile checkpoint

**Files:**
- Create: `components/seeed_mr24hpb1/binary_sensor.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_PRESENCE = "presence"
CONF_MOTION = "motion"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
            device_class="occupancy"
        ),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class="motion"
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(hub.set_presence_binary_sensor(sens))
    if CONF_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
        cg.add(hub.set_motion_binary_sensor(sens))
```

- [ ] **Step 2: Replace `test.yaml` with the v2 hub + binary_sensor only**

```yaml
---
esphome:
  name: test
  platformio_options:
    extra_scripts:
      - post:../../../export_compdb.py

esp32:
  board: esp32-s3-devkitc-1

wifi:
  ssid: wifi_ssid
  password: wifi_password
  fast_connect: true

logger:

external_components:
  - source: components

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
```

- [ ] **Step 3: Validate config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!`

- [ ] **Step 4: Compile**

Run: `PATH="$PWD/.venv-test/bin:$PATH" .venv-test/bin/esphome compile test.yaml`
Expected: `Successfully created ESP32-S3 image.` and exit 0.

- [ ] **Step 5: Commit**

```bash
git add components/seeed_mr24hpb1/binary_sensor.py test.yaml
git commit -m "Add binary_sensor platform (presence, motion)"
```

---

### Task 5: sensor platform (movement %)

**Files:**
- Create: `components/seeed_mr24hpb1/sensor.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_MOVEMENT_PCT = "movement_pct"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_MOVEMENT_PCT): sensor.sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=1,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_MOVEMENT_PCT in config:
        sens = await sensor.new_sensor(config[CONF_MOVEMENT_PCT])
        cg.add(hub.set_movement_pct_sensor(sens))
```

- [ ] **Step 2: Add to `test.yaml`** (append after the `binary_sensor:` block)

```yaml
sensor:
  - platform: seeed_mr24hpb1
    movement_pct:
      name: "Radar Movement Percentage"
```

- [ ] **Step 3: Validate config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!`

- [ ] **Step 4: Commit**

```bash
git add components/seeed_mr24hpb1/sensor.py test.yaml
git commit -m "Add sensor platform (movement percentage)"
```

---

### Task 6: text_sensor platform (movement class, device id, versions)

**Files:**
- Create: `components/seeed_mr24hpb1/text_sensor.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_MOVEMENT_CLASS = "movement_class"
CONF_DEVICE_ID = "device_id"
CONF_SOFTWARE_VERSION = "software_version"
CONF_HARDWARE_VERSION = "hardware_version"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_MOVEMENT_CLASS): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_DEVICE_ID): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SOFTWARE_VERSION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HARDWARE_VERSION): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_MOVEMENT_CLASS in config:
        s = await text_sensor.new_text_sensor(config[CONF_MOVEMENT_CLASS])
        cg.add(hub.set_movement_class_text_sensor(s))
    if CONF_DEVICE_ID in config:
        s = await text_sensor.new_text_sensor(config[CONF_DEVICE_ID])
        cg.add(hub.set_device_id_text_sensor(s))
    if CONF_SOFTWARE_VERSION in config:
        s = await text_sensor.new_text_sensor(config[CONF_SOFTWARE_VERSION])
        cg.add(hub.set_software_version_text_sensor(s))
    if CONF_HARDWARE_VERSION in config:
        s = await text_sensor.new_text_sensor(config[CONF_HARDWARE_VERSION])
        cg.add(hub.set_hardware_version_text_sensor(s))
```

- [ ] **Step 2: Add to `test.yaml`** (append)

```yaml
text_sensor:
  - platform: seeed_mr24hpb1
    movement_class:
      name: "Radar Movement Class"
    device_id:
      name: "Radar Device ID"
      entity_category: "diagnostic"
    software_version:
      name: "Radar Software Version"
      entity_category: "diagnostic"
    hardware_version:
      name: "Radar Hardware Version"
      entity_category: "diagnostic"
```

- [ ] **Step 3: Validate config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!`

- [ ] **Step 4: Commit**

```bash
git add components/seeed_mr24hpb1/text_sensor.py test.yaml
git commit -m "Add text_sensor platform (movement class, device id, versions)"
```

---

### Task 7: select platform (scene mode) — first control

**Files:**
- Create: `components/seeed_mr24hpb1/select.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

The `SCENE_OPTIONS` order is the single source of truth: the option index equals the protocol byte (`0`=Default … `6`=Hotel). The C++ publishes `opts[byte]` and maps a selected option back to its index, so the two never drift.

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

SceneSelect = seeed_mr24hpb1_ns.class_("SceneSelect", select.Select)

CONF_SCENE_MODE = "scene_mode"
SCENE_OPTIONS = [
    "Default",
    "Area Detection",
    "Bathroom",
    "Bedroom",
    "Living Room",
    "Office",
    "Hotel",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_SCENE_MODE): select.select_schema(SceneSelect),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_SCENE_MODE in config:
        sel = await select.new_select(config[CONF_SCENE_MODE], options=SCENE_OPTIONS)
        await cg.register_parented(sel, config[CONF_SEEED_MR24HPB1_ID])
        cg.add(hub.set_scene_select(sel))
```

- [ ] **Step 2: Add to `test.yaml`** (append)

```yaml
select:
  - platform: seeed_mr24hpb1
    scene_mode:
      name: "Radar Scene Mode"
```

- [ ] **Step 3: Validate config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!`

- [ ] **Step 4: Compile (C++ control path now exercised)**

Run: `PATH="$PWD/.venv-test/bin:$PATH" .venv-test/bin/esphome compile test.yaml`
Expected: `Successfully created ESP32-S3 image.` and exit 0.

- [ ] **Step 5: Commit**

```bash
git add components/seeed_mr24hpb1/select.py test.yaml
git commit -m "Add select platform (scene mode)"
```

---

### Task 8: number platform (sensitivity, slider)

**Files:**
- Create: `components/seeed_mr24hpb1/number.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

Default the mode to slider (per spec). If `number.number_schema` already defines `CONF_MODE`, this override replaces its default; verify the enum name `NUMBER_MODES` against the installed ESPHome during the compile step and adjust if needed.

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_MODE

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

SensitivityNumber = seeed_mr24hpb1_ns.class_("SensitivityNumber", number.Number)

CONF_SENSITIVITY = "sensitivity"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_SENSITIVITY): number.number_schema(SensitivityNumber).extend(
            {cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(number.NUMBER_MODES, upper=True)}
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_SENSITIVITY in config:
        num = await number.new_number(
            config[CONF_SENSITIVITY], min_value=1, max_value=3, step=1
        )
        await cg.register_parented(num, config[CONF_SEEED_MR24HPB1_ID])
        cg.add(hub.set_sensitivity_number(num))
```

- [ ] **Step 2: Add to `test.yaml`** (append)

```yaml
number:
  - platform: seeed_mr24hpb1
    sensitivity:
      name: "Radar Sensitivity"
```

- [ ] **Step 3: Validate config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!` (and the dumped `sensitivity` shows `mode: slider`).

- [ ] **Step 4: Commit**

```bash
git add components/seeed_mr24hpb1/number.py test.yaml
git commit -m "Add number platform (sensitivity slider)"
```

---

### Task 9: button platform (reboot) + final compile

**Files:**
- Create: `components/seeed_mr24hpb1/button.py`
- Modify: `test.yaml`

- [ ] **Step 1: Create the platform**

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

RebootButton = seeed_mr24hpb1_ns.class_("RebootButton", button.Button)

CONF_REBOOT = "reboot"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_REBOOT): button.button_schema(RebootButton),
    }
)


async def to_code(config):
    if CONF_REBOOT in config:
        btn = await button.new_button(config[CONF_REBOOT])
        await cg.register_parented(btn, config[CONF_SEEED_MR24HPB1_ID])
```

- [ ] **Step 2: Add to `test.yaml`** (append)

```yaml
button:
  - platform: seeed_mr24hpb1
    reboot:
      name: "Radar Reboot"
```

- [ ] **Step 3: Validate + compile the full v2 config**

Run: `.venv-test/bin/esphome config test.yaml`
Expected: `INFO Configuration is valid!`

Run: `PATH="$PWD/.venv-test/bin:$PATH" .venv-test/bin/esphome compile test.yaml`
Expected: `Successfully created ESP32-S3 image.` and exit 0.

- [ ] **Step 4: Commit**

```bash
git add components/seeed_mr24hpb1/button.py test.yaml
git commit -m "Add button platform (reboot)"
```

---

### Task 10: Documentation + device config + version pin

**Files:**
- Modify: `README.md`
- Modify (outside repo): `~/Downloads/office-presence.yaml`

- [ ] **Step 1: Rewrite the README config example** to the v2 hub + platform shape (mirror `test.yaml`), add a "Migrating from v1" section with a before→after snippet (v1 `template` sensors + `*_sensor` keys → v2 platform blocks), document the three controls (scene `select`, sensitivity `number` slider, reboot `button`), and bump the install pin:

```yaml
external_components:
  - source: github://alitheg/esphome-seeed-mr24hpb1@v2.0.0
```

- [ ] **Step 2: Rewrite `~/Downloads/office-presence.yaml`** to the v2 shape: replace the `template` sensor declarations and `seeed_mr24hpb1: *_sensor:` keys with `binary_sensor`/`sensor`/`text_sensor`/`select`/`number`/`button` platform blocks, add `update_interval: 60s` to the hub, and delete the `interval:` query block (the hub now polls). Point the source at the branch for testing: `github://alitheg/esphome-seeed-mr24hpb1@feat/v2-hub-platform` with `refresh: 0s`.

- [ ] **Step 3: Commit (repo file only)**

```bash
git add README.md
git commit -m "Document v2 config, controls and migration"
```

---

### Task 11: On-device verification (user)

- [ ] Flash `office-presence.yaml` over USB.
- [ ] Confirm presence/motion/movement/versions populate, scene mode and sensitivity read back within ~1 minute of boot.
- [ ] Change scene mode (select) and sensitivity (number) in HA; confirm the radar applies them and the entity reflects the new value.
- [ ] Press the reboot button; confirm the module restarts.

After sign-off: merge `feat/v2-hub-platform` to `main`, tag `v2.0.0`, push (triggers the release workflow), and switch `office-presence.yaml` back to `@v2.0.0`.

---

## Self-review

- **Spec coverage:** hub+platform architecture (Tasks 1-3), all read entities (4-6), scene select / sensitivity number / reboot button (7-9), polling + write + publish-back (Task 2), migration + docs + versioning (Task 10), on-device test (Task 11). All spec sections map to a task.
- **Type consistency:** hub setters (`set_presence_binary_sensor`, `set_movement_pct_sensor`, `set_*_text_sensor`, `set_scene_select`, `set_sensitivity_number`) and write methods (`write_scene_mode`, `write_sensitivity`, `reboot`) are declared in Task 1 and used identically in Tasks 2, 4-9. Control classes `SceneSelect`/`SensitivityNumber`/`RebootButton` declared in Task 1, registered in Tasks 7-9.
- **Known verification points (not placeholders):** `number.NUMBER_MODES` enum name and whether `number_schema` already carries `CONF_MODE` (Task 8) - confirm at compile and adjust if the installed ESPHome differs.
