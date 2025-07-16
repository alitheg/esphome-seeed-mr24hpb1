#include "esphome/core/log.h"
#include "seeed_mr24hpb1.h"

namespace esphome {
  namespace seeed_mr24hpb1 {
      
    static const char *TAG = "seeed_mr24hpb1.sensor";
    
    MR24HPB1::MR24HPB1(uart::UARTComponent *parent) : UARTDevice(parent) {}
    
    void MR24HPB1::setup() {
      ESP_LOGCONFIG(TAG, "Setting up mmWave radar sensors...");
    
      presence_sensor_ = new binary_sensor::BinarySensor();
      presence_sensor_->set_name("Radar Presence");
      register_child(presence_sensor_);
    
      motion_sensor_ = new binary_sensor::BinarySensor();
      motion_sensor_->set_name("Radar Motion");
      register_child(motion_sensor_);
    
      movement_pct_sensor_ = new sensor::Sensor();
      movement_pct_sensor_->set_name("Radar Movement %");
      register_child(movement_pct_sensor_);
    
      threshold_gear_sensor_ = new sensor::Sensor();
      threshold_gear_sensor_->set_name("Radar Threshold Gear");
      register_child(threshold_gear_sensor_);
    
      movement_class_sensor_ = new text_sensor::TextSensor();
      movement_class_sensor_->set_name("Radar Movement Class");
      register_child(movement_class_sensor_);
    
      device_id_sensor_ = new text_sensor::TextSensor();
      device_id_sensor_->set_name("Radar Device ID");
      register_child(device_id_sensor_);
    
      software_version_sensor_ = new text_sensor::TextSensor();
      software_version_sensor_->set_name("Radar Software Version");
      register_child(software_version_sensor_);
    
      scene_mode_sensor_ = new text_sensor::TextSensor();
      scene_mode_sensor_->set_name("Radar Scene Mode");
      register_child(scene_mode_sensor_);
    }
    
    void MR24HPB1::update() {
    
    }
    
    void MR24HPB1::loop() {
      while (available()) {
        uint8_t byte;
        read_byte(&byte);
        buffer_.push_back(byte);
    
        // Minimal frame validation
        if (buffer_.size() >= 7 && buffer_[0] == 0x55) {
          uint8_t fn = buffer_[3];
          uint8_t addr1 = buffer_[4];
          uint8_t addr2 = buffer_[5];
    
          // Process known frame types
          if ((fn == 0x03 || fn == 0x04) && (addr1 == 0x03 || addr1 == 0x01 || addr1 == 0x04)) {
            parse_frame_(buffer_);
            buffer_.clear();
          }
        }
    
        // Prevent runaway buffer
        if (buffer_.size() > 64)
          buffer_.clear();
      }
    }
    
    void MR24HPB1::parse_frame_(std::vector<uint8_t> &bytes) {
      size_t len = bytes.size();
      if (len < 7 || bytes[0] != 0x55) return;
    
      uint8_t fn = bytes[3];
      uint8_t addr1 = bytes[4];
      uint8_t addr2 = bytes[5];
    
      ESP_LOGD(TAG, "Parsing frame: fn=0x%02X addr1=0x%02X addr2=0x%02X len=%d", fn, addr1, addr2, len);
    
      if (addr1 == 0x03 && addr2 == 0x05 && len >= 10) {
        // Environmental status
        uint8_t presence = bytes[6];
        uint8_t motion = bytes[7];
        if (presence_sensor_) presence_sensor_->publish_state(presence);
        if (motion_sensor_) motion_sensor_->publish_state(motion);
        ESP_LOGI(TAG, "Environmental: presence=%s, motion=%s",
                 presence ? "present" : "absent",
                 motion ? "moving" : "still");
      }
      else if (addr1 == 0x03 && addr2 == 0x06 && len >= 11) {
        // Body movement percentage
        union { uint8_t b[4]; float f; } val;
        for (int i = 0; i < 4; i++) val.b[i] = bytes[6 + i];
        float pct = val.f;
        if (movement_pct_sensor_) movement_pct_sensor_->publish_state(pct);
    
        const char *cls = "unknown";
        if (pct < 1.0f) cls = "unoccupied";
        else if (pct < 2.0f) cls = "resting";
        else if (pct <= 30.0f) cls = "micro-movement";
        else if (pct <= 60.0f) cls = "walking";
        else cls = "running";
    
        if (movement_class_sensor_) movement_class_sensor_->publish_state(cls);
        ESP_LOGI(TAG, "Movement %.1f%% class %s", pct, cls);
      }
      else if (addr1 == 0x01 && addr2 == 0x01 && len >= 19) {
        // Device ID
        std::string idstr;
        for (size_t i = 6; i < len - 2; i++) {
          if (bytes[i] != 0 && bytes[i] != 0xFF)
            idstr += static_cast<char>(bytes[i]);
        }
        if (!idstr.empty()) {
          if (device_id_sensor_) device_id_sensor_->publish_state(idstr);
          ESP_LOGI(TAG, "Device ID: %s", idstr.c_str());
        }
      }
      else if (addr1 == 0x01 && addr2 == 0x02 && len >= 22) {
        // Software version
        std::string ver;
        for (size_t i = 6; i < len - 2; i++) {
          if (bytes[i] != 0 && bytes[i] != 0xFF)
            ver += static_cast<char>(bytes[i]);
        }
        if (!ver.empty()) {
          if (software_version_sensor_) software_version_sensor_->publish_state(ver);
          ESP_LOGI(TAG, "Software Version: %s", ver.c_str());
        }
      }
      else if (addr1 == 0x04 && addr2 == 0x0C && len >= 8) {
        // Threshold gear
        uint8_t gear = bytes[6];
        if (threshold_gear_sensor_) threshold_gear_sensor_->publish_state(gear);
        ESP_LOGI(TAG, "Threshold gear: %d", gear);
      }
      else if (addr1 == 0x04 && addr2 == 0x10 && len >= 8) {
        // Scene setup
        uint8_t mode = bytes[6];
        const char *scene = "unknown";
        switch (mode) {
          case 0x00: scene = "Default"; break;
          case 0x01: scene = "Bathroom"; break;
          case 0x02: scene = "Bedroom"; break;
          case 0x03: scene = "Living Room"; break;
          case 0x04: scene = "Office"; break;
          case 0x05: scene = "Hotel"; break;
        }
        if (scene_mode_sensor_) scene_mode_sensor_->publish_state(scene);
        ESP_LOGI(TAG, "Scene: %s", scene);
      }
      else {
        ESP_LOGW(TAG, "Unhandled known frame: fn=0x%02X addr1=0x%02X addr2=0x%02X len=%d", fn, addr1, addr2, len);
      }
    }
    
    void MR24HPB1::dump_config(){
        ESP_LOGCONFIG(TAG, "Empty UART sensor");
    }
  
  }  // namespace seeed_mr24hpb1
}  // namespace esphome