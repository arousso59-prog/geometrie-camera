#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct CameraFrameInfo {
  bool valid{false};
  uint16_t width{0};
  uint16_t height{0};
  size_t size_bytes{0};
  uint32_t timestamp_ms{0};
};

class CameraManager {
 public:
  void setup();
  void loop();

  bool ready() const;
  const CameraFrameInfo &last_frame_info() const;

  // Sera relie au composant esp32_camera apres validation du pinout.
  bool request_capture();

 private:
  bool ready_{false};
  CameraFrameInfo last_frame_{};
};

}  // namespace geometrie_camera_app
}  // namespace esphome
