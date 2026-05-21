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
