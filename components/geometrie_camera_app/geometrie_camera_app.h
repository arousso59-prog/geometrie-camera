#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "camera_manager.h"
#include "geometry_measurement.h"
#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometrieCameraApp : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  std::string status_text() const;
  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;

  CameraManager &camera_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();

 private:
  CameraManager camera_manager_{};
  TargetDetector target_detector_{};
  GeometryMeasurementEngine measurement_engine_{};

  GeometryMeasurement last_measurement_{};
  uint32_t valid_measurement_count_{0};
};

}  // namespace geometrie_camera_app
}  // namespace esphome
