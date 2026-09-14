#pragma once

#include <cstdint>

#include "geometry_measurement.h"
#include "target_detector.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class MeasurementManager {
 public:
  MeasurementManager();

  void setup();
  void reset();
  bool process(const GrayFrameView &frame, uint32_t timestamp_ms);

  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();

 private:
  TargetDetector target_detector_;
  GeometryMeasurementEngine measurement_engine_;
  GeometryMeasurement last_measurement_;
  uint32_t valid_measurement_count_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
