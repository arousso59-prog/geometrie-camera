#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController {
 public:
  CameraResolutionController();

  bool sync_from_sensor();
  bool apply(const std::string &resolution);
  bool is_supported(const std::string &resolution) const;

  const std::string &active_resolution() const;
  uint16_t active_width() const;
  uint16_t active_height() const;

  static const char *allowed_resolutions_text();

 private:
  std::string active_resolution_;
  uint16_t active_width_;
  uint16_t active_height_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
