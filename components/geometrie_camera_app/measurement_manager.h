#pragma once

#include <cstdint>

#include "geometry_measurement.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class MeasurementManager {
 public:
  MeasurementManager();

  void setup();
  void reset();
  bool process(const TargetObservation &observation,
               uint16_t frame_width, uint16_t frame_height,
               uint32_t timestamp_ms);

  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;
  GeometryMeasurementEngine &measurement_engine();
  const GeometryMeasurementEngine &measurement_engine() const;

 private:
  GeometryMeasurementEngine measurement_engine_;
  GeometryMeasurement last_measurement_;
  uint32_t valid_measurement_count_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
