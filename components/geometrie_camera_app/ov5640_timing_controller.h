#pragma once

#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct Ov5640TimingSnapshot {
  bool sensor_available;
  bool sensor_is_ov5640;
  uint16_t sensor_pid;
  int xclk_mhz;
  int pclk_divider;
  int vfifo_ctrl0c;
  bool pclk_manual;
  int hts;
  int vts;
  int jpeg_mode;
  int href_blanking;
  bool baseline_available;
  uint8_t baseline_xclk_mhz;
  uint16_t baseline_hts;
  uint16_t baseline_vts;
  uint8_t baseline_jpeg_mode;
  uint8_t baseline_href_blanking;
};

class Ov5640TimingController {
 public:
  Ov5640TimingController();

  Ov5640TimingSnapshot snapshot() const;
  bool set_xclk_mhz(uint8_t mhz);
  bool set_pclk_divider(uint8_t divider);
  bool set_hts(uint16_t hts);
  bool set_vts(uint16_t vts);
  bool set_jpeg_mode(uint8_t mode);
  bool set_href_blanking(uint8_t blanking);
  bool restore_baseline();
  bool baseline_available() const;

  static uint8_t min_xclk_mhz();
  static uint8_t max_xclk_mhz();
  static uint8_t min_pclk_divider();
  static uint8_t max_pclk_divider();
  static uint16_t min_total_timing();
  static uint16_t max_total_timing();
  static uint8_t min_href_blanking();
  static uint8_t max_href_blanking();

 private:
  bool capture_baseline_if_needed_();
  bool read_total_timing_(uint16_t *hts, uint16_t *vts) const;
  bool read_jpeg_output_timing_(uint8_t *jpeg_mode, uint8_t *href_blanking) const;
  bool write_total_timing_register_(uint16_t high_register, uint16_t value) const;
  bool write_byte_register_(uint16_t reg, uint8_t value) const;

  bool baseline_available_;
  uint8_t baseline_xclk_mhz_;
  uint16_t baseline_hts_;
  uint16_t baseline_vts_;
  uint8_t baseline_jpeg_mode_;
  uint8_t baseline_href_blanking_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
