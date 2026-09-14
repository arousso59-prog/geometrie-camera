#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "camera_api.h"
#include "camera_manager.h"
#include "grayscale_diagnostic.h"
#include "grayscale_diagnostic_api.h"
#include "measurement_manager.h"
#include "placeholder_image_provider.h"
#include "types.h"

namespace esphome {
namespace esp32_camera {
class ESP32Camera;
}
namespace geometrie_camera_app {

class GeometrieCameraApp : public Component {
 public:
  GeometrieCameraApp();

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_camera(esp32_camera::ESP32Camera *camera);

  std::string status_text() const;
  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;

  CameraManager &camera_manager();
  MeasurementManager &measurement_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();
  GrayscaleDiagnostic &grayscale_diagnostic();

 private:
  void register_api_if_possible_();

  PlaceholderImageProvider placeholder_image_provider_;
  CameraManager camera_manager_;
  MeasurementManager measurement_manager_;
  GrayscaleDiagnostic grayscale_diagnostic_;
  CameraApiHandler api_handler_;
  GrayscaleDiagnosticApiHandler diagnostic_api_handler_;
  bool api_registered_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
