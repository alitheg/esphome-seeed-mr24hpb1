#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace seeed_mr24hpb1 {

class MR24HPB1 : public Component, public uart::UARTDevice {
 public:
  explicit MR24HPB1(uart::UARTComponent *parent) : UARTDevice(parent) {}

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void parse_frame_(std::vector<uint8_t> &bytes);
  std::vector<uint8_t> buffer_;

  // Internal sensors (exposed automatically)
  binary_sensor::BinarySensor *presence_sensor_;
  binary_sensor::BinarySensor *motion_sensor_;
  sensor::Sensor *movement_pct_sensor_;
  sensor::Sensor *threshold_gear_sensor_;
  text_sensor::TextSensor *movement_class_sensor_;
  text_sensor::TextSensor *device_id_sensor_;
  text_sensor::TextSensor *software_version_sensor_;
  text_sensor::TextSensor *scene_mode_sensor_;
};

}  // namespace seeed_mr24hpb1
}  // namespace esphome