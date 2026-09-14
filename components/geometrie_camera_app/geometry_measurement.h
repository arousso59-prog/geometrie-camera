#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometryMeasurementEngine {
 public:
  GeometryMeasurementEngine();

  void set_calibration(const CameraCalibration &calibration);
  const CameraCalibration &calibration() const;
  GeometryMeasurement compute(const TargetObservation &observation, uint32_t timestamp_ms) const;

 private:
  CameraCalibration calibration_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
