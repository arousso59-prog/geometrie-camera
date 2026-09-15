#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "esphome/components/camera/camera.h"

namespace esphome {
namespace esp32_camera {
class ESP32Camera;
}
namespace geometrie_camera_app {

class GrayscaleDiagnostic : public camera::CameraListener {
 public:
  GrayscaleDiagnostic();
  ~GrayscaleDiagnostic();

  void set_camera(esp32_camera::ESP32Camera *camera);
  bool request_capture();

  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  bool ready() const;
  bool capture_pending() const;
  uint32_t capture_count() const;
  uint32_t last_capture_ms() const;
  uint16_t width() const;
  uint16_t height() const;
  const uint8_t *bmp_data() const;
  size_t bmp_size() const;

  uint32_t request_started_ms() const;
  uint32_t frame_received_ms() const;
  uint32_t acquisition_ms() const;
  uint32_t diagnostic_processing_ms() const;
  uint32_t total_cycle_ms() const;

  uint8_t raw_min() const;
  uint8_t raw_max() const;
  float raw_mean() const;
  uint32_t raw_zero_count() const;
  uint32_t raw_full_count() const;
  size_t raw_pixel_count() const;

 private:
  bool build_bmp_(const uint8_t *grayscale, size_t grayscale_size, uint16_t width, uint16_t height);
  void calculate_statistics_(const uint8_t *grayscale, size_t pixel_count);
  bool ensure_buffer_(size_t required_size);
  void clear_buffer_();

  esp32_camera::ESP32Camera *camera_;
  uint8_t *bmp_buffer_;
  size_t bmp_size_;
  size_t bmp_capacity_;
  uint16_t width_;
  uint16_t height_;
  uint32_t capture_count_;
  uint32_t last_capture_ms_;
  bool capture_pending_;
  bool ready_;

  uint32_t request_started_ms_;
  uint32_t frame_received_ms_;
  uint32_t acquisition_ms_;
  uint32_t diagnostic_processing_ms_;
  uint32_t total_cycle_ms_;

  uint8_t raw_min_;
  uint8_t raw_max_;
  float raw_mean_;
  uint32_t raw_zero_count_;
  uint32_t raw_full_count_;
  size_t raw_pixel_count_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
