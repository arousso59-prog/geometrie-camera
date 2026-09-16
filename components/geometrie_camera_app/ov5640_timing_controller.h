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
  int hts;
  int vts;
  bool baseline_available;
  uint16_t baseline_hts;
  uint16_t baseline_vts;
};

class Ov5640TimingController {
 public:
  Ov5640TimingController();

  Ov5640TimingSnapshot snapshot() const;
  bool set_pclk_divider(uint8_t divider);
  bool set_hts(uint16_t hts);
  bool set_vts(uint16_t vts);
  bool restore_total_timing();
  bool baseline_available() const;

  static uint8_t min_pclk_divider();
  static uint8_t max_pclk_divider();
  static uint16_t min_total_timing();
  static uint16_t max_total_timing();

 private:
  bool capture_baseline_if_needed_();
  bool read_total_timing_(uint16_t *hts, uint16_t *vts) const;
  bool write_total_timing_register_(uint16_t high_register, uint16_t value) const;

  bool baseline_available_;
  uint16_t baseline_hts_;
  uint16_t baseline_vts_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
