#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController {
 public:
  CameraResolutionController();

  bool sync_from_sensor();
  bool refresh_sensor_identity();
  bool apply(const std::string &resolution);
  bool is_supported(const std::string &resolution) const;

  const std::string &active_resolution() const;
  uint16_t active_width() const;
  uint16_t active_height() const;

  uint16_t sensor_pid() const;
  const std::string &sensor_name() const;
  const std::string &sensor_max_resolution() const;

  static const char *allowed_resolutions_text();

 private:
  void update_sensor_identity_();

  std::string active_resolution_;
  uint16_t active_width_;
  uint16_t active_height_;

  uint16_t sensor_pid_;
  std::string sensor_name_;
  std::string sensor_max_resolution_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
