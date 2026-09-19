#pragma once

#include <cstdint>

#include "esphome/core/preferences.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometryCalibrationStorage {
 public:
  GeometryCalibrationStorage();

  void setup();
  bool load(CameraCalibration &calibration);
  bool save(const CameraCalibration &calibration,
            float stddev_fx_px, float stddev_fy_px);
  bool clear();

  bool available() const;
  bool stored_valid() const;
  float stored_stddev_fx_px() const;
  float stored_stddev_fy_px() const;

 private:
  struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t target_model;
    CameraCalibration calibration;
    float stddev_fx_px;
    float stddev_fy_px;
  };

  static constexpr uint32_t MAGIC = 0x47335231UL;  // "G3R1"
  static constexpr uint16_t VERSION = 1;
  static constexpr uint16_t TARGET_MODEL_R1 = 1;
  static constexpr uint32_t PREFERENCE_KEY = 0x47523101UL;

  ESPPreferenceObject preference_;
  bool available_;
  bool stored_valid_;
  float stored_stddev_fx_px_;
  float stored_stddev_fy_px_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
