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
      overrun_count_ = 0;
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
      if (++overrun_count_ >= 5) {
        buffer_.clear();
        overrun_count_ = 0;
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
      const auto &opts = scene_select_->traits.get_options();
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
