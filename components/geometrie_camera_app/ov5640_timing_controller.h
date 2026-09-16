#pragma once

#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct Ov5640TimingSnapshot {
  bool sensor_available;
  bool sensor_is_ov5640;
  uint16_t sensor_pid;
  int pclk_divider;
  int vfifo_ctrl0c;
  bool pclk_manual;
};

class Ov5640TimingController {
 public:
  Ov5640TimingController();

  Ov5640TimingSnapshot snapshot() const;
  bool set_pclk_divider(uint8_t divider);

  static uint8_t min_pclk_divider();
  static uint8_t max_pclk_divider();

 private:
  static constexpr uint16_t PCLK_RATIO_REGISTER = 0x3824;
  static constexpr uint16_t VFIFO_CTRL0C_REGISTER = 0x460C;
  static constexpr uint8_t PCLK_RATIO_MASK = 0x1F;
  static constexpr uint8_t PCLK_MANUAL_ENABLE_MASK = 0x02;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
