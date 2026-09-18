#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometryMeasurementEngine {
 public:
  GeometryMeasurementEngine();

  // Cible R1 fixe : cadre metrologique noir 240 x 90 mm.
  // Les anciennes API taille-cible sont conservees en lecture/compatibilite,
  // mais la geometrie n'est plus configurable.
  bool set_target_size_mm(float target_size_mm);
  float target_size_mm() const;
  float target_width_mm() const;
  float target_height_mm() const;

  void set_calibration(const CameraCalibration &calibration);
  void clear_calibration();
  bool has_calibration() const;
  const CameraCalibration &calibration() const;
  CameraCalibration effective_calibration(uint16_t frame_width, uint16_t frame_height) const;

  bool calibrate_from_known_distance(const TargetObservation &observation,
                                     uint16_t frame_width, uint16_t frame_height,
                                     float known_distance_mm);
  bool derive_calibration_from_known_distance(const TargetObservation &observation,
                                              uint16_t frame_width, uint16_t frame_height,
                                              float known_distance_mm,
                                              CameraCalibration &result) const;

  bool set_distortion_coefficients(float k1, float k2, float p1, float p2, float k3);
  void clear_distortion();

  GeometryMeasurement compute(const TargetObservation &observation,
                              uint16_t frame_width, uint16_t frame_height,
                              uint32_t timestamp_ms) const;

 private:
  CameraCalibration calibration_;
  float target_width_mm_;
  float target_height_mm_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
