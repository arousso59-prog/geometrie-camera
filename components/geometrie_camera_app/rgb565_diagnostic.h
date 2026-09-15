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

class Rgb565Diagnostic : public camera::CameraListener {
 public:
  Rgb565Diagnostic();
  ~Rgb565Diagnostic();

  void set_camera(esp32_camera::ESP32Camera *camera);
  bool request_capture();

  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  bool ready() const;
  bool capture_pending() const;
  uint32_t capture_count() const;
  uint32_t last_capture_ms() const;
  uint16_t width() const;
  uint16_t height() const;
  size_t source_size() const;
  const uint8_t *bmp_data() const;
  size_t bmp_size() const;

 private:
  void clear_buffer_();

  esp32_camera::ESP32Camera *camera_;
  uint8_t *bmp_buffer_;
  size_t bmp_size_;
  size_t source_size_;
  uint16_t width_;
  uint16_t height_;
  uint32_t capture_count_;
  uint32_t last_capture_ms_;
  bool capture_pending_;
  bool ready_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
