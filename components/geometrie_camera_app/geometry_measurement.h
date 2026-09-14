#pragma once

#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct CameraCalibration {
  float fx_px{1000.0f};
  float fy_px{1000.0f};
  float cx_px{1024.0f};
  float cy_px{768.0f};
};

struct TargetObservation {
  bool valid{false};
  float center_x_px{0.0f};
  float center_y_px{0.0f};
  float width_px{0.0f};
  float height_px{0.0f};
  float rotation_deg{0.0f};
  float quality{0.0f};
};

struct GeometryMeasurement {
  bool valid{false};
  float yaw_deg{0.0f};
  float pitch_deg{0.0f};
  float roll_deg{0.0f};
  float quality{0.0f};
  uint32_t timestamp_ms{0};
};

class GeometryMeasurementEngine {
 public:
  void set_calibration(const CameraCalibration &calibration);
  const CameraCalibration &calibration() const;

  GeometryMeasurement compute(const TargetObservation &observation, uint32_t timestamp_ms) const;

 private:
  CameraCalibration calibration_{};
};

}  // namespace geometrie_camera_app
}  // namespace esphome
