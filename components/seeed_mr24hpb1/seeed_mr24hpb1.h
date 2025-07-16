#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace seeed_mr24hpb1 {

class MR24HPB1 : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_presence_sensor(binary_sensor::BinarySensor *sensor);
  
  void set_motion_sensor(binary_sensor::BinarySensor *sensor);
  
  void set_movement_pct_sensor(sensor::Sensor *sensor);
  
  void set_threshold_gear_sensor(sensor::Sensor *sensor);
  
  void set_movement_class_sensor(text_sensor::TextSensor *sensor);
  
  void set_device_id_sensor(text_sensor::TextSensor *sensor);
  
  void set_software_version_sensor(text_sensor::TextSensor *sensor);
  
  void set_scene_mode_sensor(text_sensor::TextSensor *sensor);
  
  void send_scene_query();
  void send_threshold_gear_query();
  void send_software_version_query();
  void send_hardware_version_query();
  void send_device_id_query();

 protected:
  void parse_frame_(std::vector<uint8_t> &bytes);
  std::vector<uint8_t> buffer_;

  binary_sensor::BinarySensor *presence_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_sensor_{nullptr};
  sensor::Sensor *movement_pct_sensor_{nullptr};
  sensor::Sensor *threshold_gear_sensor_{nullptr};
  text_sensor::TextSensor *movement_class_sensor_{nullptr};
  text_sensor::TextSensor *device_id_sensor_{nullptr};
  text_sensor::TextSensor *software_version_sensor_{nullptr};
  text_sensor::TextSensor *scene_mode_sensor_{nullptr};
  uint16_t expected_length_{0};
  void send_command(uint8_t fn, uint8_t addr1, uint8_t addr2);
  void send_command(uint8_t fn, uint8_t addr1, uint8_t addr2, const std::vector<uint8_t>& data);
  uint16_t crc16(const uint8_t *data, size_t length);
};

}  // namespace seeed_mr24hpb1
}  // namespace esphome