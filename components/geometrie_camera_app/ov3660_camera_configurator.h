#pragma once

#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

class Ov3660CameraConfigurator {
 public:
  Ov3660CameraConfigurator();

  void setup();
  void loop();

  bool sensor_detected() const;
  bool pclk_test_applied() const;
  int original_clock_pol_control() const;
  int modified_clock_pol_control() const;

 private:
  bool apply_pclk_polarity_test_();

  bool sensor_detected_;
  bool pclk_test_applied_;
  uint32_t last_attempt_ms_;
  int original_clock_pol_control_;
  int modified_clock_pol_control_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
