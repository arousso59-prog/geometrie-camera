#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "camera_api.h"
#include "camera_manager.h"
#include "camera_resolution_controller.h"
#include "grayscale_diagnostic.h"
#include "grayscale_diagnostic_api.h"
#include "measurement_manager.h"
#include "ov3660_camera_configurator.h"
#include "placeholder_image_provider.h"
#include "rgb565_diagnostic.h"
#include "rgb565_diagnostic_api.h"
#include "target_search_diagnostic.h"
#include "target_search_diagnostic_api.h"
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
  void set_ov3660_pclk_divider(uint8_t divider);

  std::string status_text() const;
  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;

  CameraManager &camera_manager();
  MeasurementManager &measurement_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();
  CameraResolutionController &resolution_controller();
  GrayscaleDiagnostic &grayscale_diagnostic();
  Rgb565Diagnostic &rgb565_diagnostic();
  TargetSearchDiagnostic &target_search_diagnostic();
  Ov3660CameraConfigurator &camera_configurator();

 private:
  void register_api_if_possible_();

  PlaceholderImageProvider placeholder_image_provider_;
  CameraManager camera_manager_;
  MeasurementManager measurement_manager_;
  CameraResolutionController resolution_controller_;
  GrayscaleDiagnostic grayscale_diagnostic_;
  Rgb565Diagnostic rgb565_diagnostic_;
  TargetSearchDiagnostic target_search_diagnostic_;
  Ov3660CameraConfigurator camera_configurator_;
  CameraApiHandler api_handler_;
  GrayscaleDiagnosticApiHandler diagnostic_api_handler_;
  Rgb565DiagnosticApiHandler rgb565_diagnostic_api_handler_;
  TargetSearchDiagnosticApiHandler target_search_diagnostic_api_handler_;
  bool api_registered_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
