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

class JpegDiagnostic : public camera::CameraListener {
 public:
  JpegDiagnostic();
  ~JpegDiagnostic();

  void set_camera(esp32_camera::ESP32Camera *camera);
  bool request_capture();
  void loop();
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  bool ready() const;
  bool capture_pending() const;
  uint32_t capture_count() const;
  uint32_t discarded_frame_count() const;
  uint32_t last_capture_ms() const;
  uint16_t width() const;
  uint16_t height() const;
  const uint8_t *jpeg_data() const;
  size_t jpeg_size() const;
  bool has_soi() const;
  bool has_eoi() const;

  uint32_t request_started_ms() const;
  uint32_t stale_frame_received_ms() const;
  uint32_t fresh_request_started_ms() const;
  uint32_t frame_received_ms() const;
  uint32_t acquisition_ms() const;
  uint32_t copy_ms() const;
  uint32_t total_cycle_ms() const;

 private:
  bool ensure_buffer_(size_t required_size);
  void clear_buffer_();

  esp32_camera::ESP32Camera *camera_;
  uint8_t *jpeg_buffer_;
  size_t jpeg_size_;
  size_t jpeg_capacity_;
  uint16_t width_;
  uint16_t height_;
  bool has_soi_;
  bool has_eoi_;
  uint32_t capture_count_;
  uint32_t discarded_frame_count_;
  uint32_t last_capture_ms_;
  bool capture_pending_;
  bool discard_next_frame_;
  bool fresh_request_pending_;
  bool ready_;
  uint32_t request_started_ms_;
  uint32_t stale_frame_received_ms_;
  uint32_t fresh_request_started_ms_;
  uint32_t frame_received_ms_;
  uint32_t acquisition_ms_;
  uint32_t copy_ms_;
  uint32_t total_cycle_ms_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
