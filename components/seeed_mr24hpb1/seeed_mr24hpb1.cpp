#include "esphome/core/log.h"
#include "seeed_mr24hpb1.h"

namespace esphome {
namespace seeed_mr24hpb1 {

static const char *const TAG = "seeed_mr24hpb1.sensor";

void MR24HPB1::setup() {
  ESP_LOGI("seeed_mr24hpb1", "Setup started");
  if (this->movement_class_sensor_ != nullptr) {
    ESP_LOGI("seeed_mr24hpb1", "Movement class sensor connected");
  } else {
    ESP_LOGW("seeed_mr24hpb1", "Movement class sensor is NULL");
  }
  buffer_.clear();
}

void MR24HPB1::set_presence_sensor(binary_sensor::BinarySensor *sensor) {
  ESP_LOGI(TAG, "set_presence_sensor called");
  this->presence_sensor_ = sensor;
}

void MR24HPB1::set_motion_sensor(binary_sensor::BinarySensor *sensor) {
  ESP_LOGI(TAG, "set_motion_sensor called");
  this->motion_sensor_ = sensor;
}

void MR24HPB1::set_movement_pct_sensor(sensor::Sensor *sensor) {
  ESP_LOGI(TAG, "set_movement_pct_sensor called");
  this->movement_pct_sensor_ = sensor;
}

void MR24HPB1::set_threshold_gear_sensor(sensor::Sensor *sensor) {
  ESP_LOGI(TAG, "set_threshold_gear_sensor called");
  this->threshold_gear_sensor_ = sensor;
}

void MR24HPB1::set_movement_class_sensor(text_sensor::TextSensor *sensor) {
  ESP_LOGI(TAG, "set_movement_class_sensor called");
  this->movement_class_sensor_ = sensor;
}

void MR24HPB1::set_device_id_sensor(text_sensor::TextSensor *sensor) {
  ESP_LOGI(TAG, "set_device_id_sensor called");
  this->device_id_sensor_ = sensor;
}

void MR24HPB1::set_software_version_sensor(text_sensor::TextSensor *sensor) {
  ESP_LOGI(TAG, "set_software_version_sensor called");
  this->software_version_sensor_ = sensor;
}

void MR24HPB1::set_scene_mode_sensor(text_sensor::TextSensor *sensor) {
  ESP_LOGI(TAG, "set_scene_mode_sensor called");
  this->scene_mode_sensor_ = sensor;
}

void MR24HPB1::loop() {
  static bool logged = false;
  static int overrun_count = 0;

  if (!logged) {
    ESP_LOGI(TAG, "Presence: %s", presence_sensor_ ? "OK" : "MISSING");
    ESP_LOGI(TAG, "Motion: %s", motion_sensor_ ? "OK" : "MISSING");
    ESP_LOGI(TAG, "Movement class: %s", movement_class_sensor_ ? "OK" : "MISSING");
    logged = true;
  }

  while (available()) {
    uint8_t byte;
    read_byte(&byte);

    // Wait for frame start
    if (buffer_.empty() && byte != 0x55) {
      // ESP_LOGV(TAG, "Ignoring byte 0x%02X — not frame start", byte);
      continue;
    }

    buffer_.push_back(byte);

    // Once we have enough to determine length
    if (buffer_.size() == 3) {
      expected_length_ = static_cast<size_t>(buffer_[1]) +
                         (static_cast<size_t>(buffer_[2]) << 8) + 3;  // len includes itself
      if (expected_length_ > 256) {
        ESP_LOGW(TAG, "Payload length too large (%d) — discarding", expected_length_);
        buffer_.clear();
        expected_length_ = 0;
        continue;
      }
      ESP_LOGD(TAG, "Expecting frame of length %d", expected_length_);
    }

    // If we have the expected frame length
    if (expected_length_ > 0 && buffer_.size() >= expected_length_) {
      std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + expected_length_);
      parse_frame_(frame);
      buffer_.erase(buffer_.begin(), buffer_.begin() + expected_length_);
      expected_length_ = 0;
      overrun_count = 0;  // Reset on successful parse
    }

    // Safety: prevent runaway growth & try to resync
    if (buffer_.size() > 128) {
      ESP_LOGW(TAG, "Buffer overrun (%d bytes) — attempting resync", buffer_.size());
      auto it = std::find(buffer_.begin() + 1, buffer_.end(), 0x55);
      if (it != buffer_.end()) {
        ESP_LOGD(TAG, "Found new frame start at offset %d", std::distance(buffer_.begin(), it));
        buffer_.erase(buffer_.begin(), it);
      } else {
        ESP_LOGW(TAG, "No valid start found — clearing buffer");
        buffer_.clear();
      }
      expected_length_ = 0;

      overrun_count++;
      if (overrun_count >= 5) {
        ESP_LOGW(TAG, "Too many overruns — clearing buffer fully");
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

  std::string hex;
  for (auto b : bytes)
    hex += (hex.empty() ? "" : " ") + esphome::format_hex(b);
  ESP_LOGD(TAG, "Raw frame: %s", hex.c_str());

  if (addr1 == 0x03 && addr2 == 0x05 && len >= 10) {
    uint8_t presence = bytes[6];
    uint8_t motion = bytes[7];
    ESP_LOGI(TAG, "Presence: %s, Motion: %s", presence ? "PRESENT" : "ABSENT", motion ? "MOVING" : "STILL");

    if (presence_sensor_) presence_sensor_->publish_state(presence);
    else ESP_LOGW(TAG, "presence_sensor_ not set — cannot publish presence");

    if (motion_sensor_) motion_sensor_->publish_state(motion);
    else ESP_LOGW(TAG, "motion_sensor_ not set — cannot publish motion");

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

    ESP_LOGI(TAG, "Movement: %.1f%% → Class: %s", pct, cls);

    if (movement_pct_sensor_) movement_pct_sensor_->publish_state(pct);
    else ESP_LOGW(TAG, "movement_pct_sensor_ not set — cannot publish movement %%");

    if (movement_class_sensor_) movement_class_sensor_->publish_state(cls);
    else ESP_LOGW(TAG, "movement_class_sensor_ not set — cannot publish movement class");

  } else if (addr1 == 0x01 && addr2 == 0x01 && len >= 19) {
    std::string idstr;
    for (size_t i = 6; i < len - 2; i++) {
      char c = static_cast<char>(bytes[i]);
      idstr += (c >= 32 && c <= 126) ? c : '.';  // dot for non-printable
    }
    ESP_LOGI(TAG, "Device ID: %s", idstr.c_str());

    if (device_id_sensor_) device_id_sensor_->publish_state(idstr);
    else ESP_LOGW(TAG, "device_id_sensor_ not set — cannot publish device ID");

  } else if (addr1 == 0x01 && addr2 == 0x02 && len >= 22) {
    std::string ver;
    for (size_t i = 6; i < len - 2; i++) {
      if (bytes[i] != 0 && bytes[i] != 0xFF)
        ver += static_cast<char>(bytes[i]);
    }
    ESP_LOGI(TAG, "Software Version: %s", ver.c_str());

    if (software_version_sensor_) software_version_sensor_->publish_state(ver);
    else ESP_LOGW(TAG, "software_version_sensor_ not set — cannot publish software version");

  } else if (addr1 == 0x04 && addr2 == 0x0C && len >= 8) {
    uint8_t gear = bytes[6];
    ESP_LOGI(TAG, "Threshold gear: %d", gear);

    if (threshold_gear_sensor_) threshold_gear_sensor_->publish_state(gear);
    else ESP_LOGW(TAG, "threshold_gear_sensor_ not set — cannot publish gear value");

  } else if (addr1 == 0x04 && addr2 == 0x10 && len >= 8) {
    uint8_t mode = bytes[6];
    const char *scene = "Unknown";
    switch (mode) {
      case 0x00: scene = "Default"; break;
      case 0x01: scene = "Bathroom"; break;
      case 0x02: scene = "Bedroom"; break;
      case 0x03: scene = "Living Room"; break;
      case 0x04: scene = "Office"; break;
      case 0x05: scene = "Hotel"; break;
    }
    ESP_LOGI(TAG, "Scene mode: %s (0x%02X)", scene, mode);

    if (scene_mode_sensor_) scene_mode_sensor_->publish_state(scene);
    else ESP_LOGW(TAG, "scene_mode_sensor_ not set — cannot publish scene mode");

  } else {
    ESP_LOGW(TAG, "Unhandled frame fn=0x%02X addr1=0x%02X addr2=0x%02X (len=%d)", fn, addr1, addr2, len);
  }
}

void MR24HPB1::send_command(uint8_t fn, uint8_t addr1, uint8_t addr2) {
  std::vector<uint8_t> empty_data;
  send_command(fn, addr1, addr2, empty_data);
}
void MR24HPB1::send_command(uint8_t fn, uint8_t addr1, uint8_t addr2, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> frame;
  frame.push_back(0x55);  // Start byte
  
  uint16_t length = 7 + data.size();  // includes itself
  frame.push_back(length & 0xFF);     // Length_L
  frame.push_back((length >> 8) & 0xFF); // Length_H
  
  frame.push_back(fn);     // Function code
  frame.push_back(addr1);  // Address 1
  frame.push_back(addr2);  // Address 2
  
  frame.insert(frame.end(), data.begin(), data.end());  // Payload
  
  uint16_t crc = crc16(frame.data(), frame.size());     // CRC over everything so far
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);

  ESP_LOGD(TAG, "Sending frame: fn=0x%02X addr1=0x%02X addr2=0x%02X data_len=%d crc=0x%04X", fn, addr1, addr2, (int)data.size(), crc);
  this->write_array(frame);
}

// Call this from YAML via custom action or lambda
void MR24HPB1::send_software_version_query() {
  ESP_LOGI(TAG, "Sending Software Version Query...");
  this->send_command(0x01, 0x01, 0x02);  // Read function, System Param Inquiry, Software Version
}
void MR24HPB1::send_device_id_query() {
  ESP_LOGI(TAG, "Sending Device ID Query...");
  this->send_command(0x01, 0x01, 0x01);  // Read function, System Param Inquiry, Device ID
}
void MR24HPB1::send_hardware_version_query() {
  ESP_LOGI(TAG, "Sending Hardware Version Query...");
  this->send_command(0x01, 0x01, 0x03);  // Read function, System Param Inquiry, Hardware Version
}
void MR24HPB1::send_scene_query() {
  ESP_LOGI(TAG, "Sending Scene Mode Query...");
  this->send_command(0x01, 0x04, 0x10);  // Read function, System Param Inquiry, Scene Mode
}

void MR24HPB1::send_threshold_gear_query() {
  ESP_LOGI(TAG, "Sending Threshold Gear Query...");
  this->send_command(0x01, 0x04, 0x0C);  // Read function, System Param Inquiry, Threshold Gear
}

// CRC16-IBM (modbus) implementation (polynomial 0x8005, initial 0xFFFF)
uint16_t MR24HPB1::crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }
  return crc;
}


void MR24HPB1::dump_config() {
  ESP_LOGCONFIG(TAG, "Seeed MR24HPB1 UART mmWave radar");
}

}  // namespace seeed_mr24hpb1
}  // namespace esphome