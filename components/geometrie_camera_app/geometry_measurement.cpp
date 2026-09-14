#include "geometry_measurement.h"

#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr float RAD_TO_DEG_F = 57.29577951308232f;
}

void GeometryMeasurementEngine::set_calibration(const CameraCalibration &calibration) {
  this->calibration_ = calibration;
}

const CameraCalibration &GeometryMeasurementEngine::calibration() const {
  return this->calibration_;
}

GeometryMeasurement GeometryMeasurementEngine::compute(const TargetObservation &observation,
                                                       uint32_t timestamp_ms) const {
  GeometryMeasurement result;
  result.timestamp_ms = timestamp_ms;
  result.quality = observation.quality;

  if (!observation.valid || this->calibration_.fx_px <= 0.0f || this->calibration_.fy_px <= 0.0f) {
    return result;
  }

  const float dx = observation.center_x_px - this->calibration_.cx_px;
  const float dy = observation.center_y_px - this->calibration_.cy_px;

  result.yaw_deg = std::atan2(dx, this->calibration_.fx_px) * RAD_TO_DEG_F;
  result.pitch_deg = std::atan2(dy, this->calibration_.fy_px) * RAD_TO_DEG_F;
  result.roll_deg = observation.rotation_deg;
  result.valid = true;

  return result;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
