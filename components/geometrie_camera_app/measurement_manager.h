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
               uint32_t timestamp_ms,
               bool stabilize = false);
  void reset_stabilization();

  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;
  const GeometryMeasurement &raw_measurement() const;
  bool last_measurement_stabilized() const;
  uint8_t stabilization_sample_count() const;
  uint8_t stabilization_window_size() const;
  float distance_stddev_mm() const;
  float distance_span_mm() const;
  GeometryMeasurementEngine &measurement_engine();
  const GeometryMeasurementEngine &measurement_engine() const;

 private:
  static constexpr uint8_t STABILIZATION_WINDOW = 5;

  void append_stabilization_sample_(const GeometryMeasurement &measurement);
  void compute_stabilized_measurement_();

  GeometryMeasurementEngine measurement_engine_;
  GeometryMeasurement raw_measurement_;
  GeometryMeasurement last_measurement_;
  GeometryMeasurement stabilization_samples_[STABILIZATION_WINDOW];
  uint8_t stabilization_count_;
  uint8_t stabilization_next_index_;
  bool last_measurement_stabilized_;
  float distance_stddev_mm_;
  float distance_span_mm_;
  uint32_t valid_measurement_count_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
