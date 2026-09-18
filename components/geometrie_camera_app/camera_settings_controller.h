#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

struct CameraSettingsSnapshot {
  bool available;
  std::string pixel_format;
  bool monochrome;
  int brightness;
  int contrast;
  bool exposure_ctrl;
  int ae_level;
  int aec_value;
  bool gain_ctrl;
  int agc_gain;
};

class CameraSettingsController {
 public:
  CameraSettingsController();

  CameraSettingsSnapshot read() const;
  bool read_live_exposure_gain(int &exposure, int &gain,
                               std::string &error) const;

  bool set_monochrome(bool enabled, std::string &error) const;
  bool set_brightness(int value, std::string &error) const;
  bool set_contrast(int value, std::string &error) const;
  bool set_exposure_ctrl(bool enabled, std::string &error) const;
  bool set_ae_level(int value, std::string &error) const;
  bool set_aec_value(int value, std::string &error) const;
  bool set_gain_ctrl(bool enabled, std::string &error) const;
  bool set_agc_gain(int value, std::string &error) const;

 private:
  bool validate_range_(const char *name, int value, int minimum, int maximum, std::string &error) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
