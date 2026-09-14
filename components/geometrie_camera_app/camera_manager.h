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
  bool physical_camera_ready() const;
  bool placeholder_mode() const;
  uint32_t capture_count() const;
  const CameraFrameInfo &last_frame_info() const;

  // En V0 cette methode produit une capture bouchon valide.
  // Plus tard son implementation sera remplacee par l'acquisition OV3660,
  // sans modifier l'API HTTP ni le logiciel PC.
  bool request_capture();

 private:
  bool ready_{false};
  bool physical_camera_ready_{false};
  bool placeholder_mode_{true};
  uint32_t capture_count_{0};
  CameraFrameInfo last_frame_{};
};

}  // namespace geometrie_camera_app
}  // namespace esphome
