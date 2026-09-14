#pragma once

#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

class Ov3660CameraConfigurator {
 public:
  Ov3660CameraConfigurator();

  void set_pclk_divider(uint8_t divider);
  void setup();
  void loop();

  bool sensor_detected() const;
  bool pclk_divider_applied() const;
  uint8_t requested_pclk_divider() const;
  int original_pclk_ratio() const;
  int applied_pclk_ratio() const;
  int vfifo_ctrl0c() const;

 private:
  bool apply_pclk_divider_();

  uint8_t requested_pclk_divider_;
  bool sensor_detected_;
  bool pclk_divider_applied_;
  uint32_t last_attempt_ms_;
  int original_pclk_ratio_;
  int applied_pclk_ratio_;
  int vfifo_ctrl0c_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
