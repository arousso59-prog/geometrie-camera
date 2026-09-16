#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "api_wsdl.h"
#include "camera_resolution_controller.h"
#include "camera_settings_api.h"
#include "camera_settings_controller.h"
#include "jpeg_diagnostic.h"
#include "jpeg_diagnostic_api.h"
#include "jpeg_filtered_diagnostic.h"
#include "jpeg_filtered_diagnostic_api.h"
#include "measurement_manager.h"
#include "runtime_diagnostics.h"
#include "runtime_diagnostics_api.h"
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

  MeasurementManager &measurement_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();
  CameraResolutionController &resolution_controller();
  CameraSettingsController &settings_controller();
  JpegDiagnostic &jpeg_diagnostic();
  JpegFilteredDiagnostic &jpeg_filtered_diagnostic();

 private:
  void register_api_if_possible_();

  MeasurementManager measurement_manager_;
  CameraResolutionController resolution_controller_;
  CameraSettingsController settings_controller_;
  JpegDiagnostic jpeg_diagnostic_;
  JpegFilteredDiagnostic jpeg_filtered_diagnostic_;
  RuntimeDiagnostics runtime_diagnostics_;
  ApiWsdlHandler api_wsdl_handler_;
  CameraSettingsApiHandler settings_api_handler_;
  JpegDiagnosticApiHandler jpeg_diagnostic_api_handler_;
  JpegFilteredDiagnosticApiHandler jpeg_filtered_diagnostic_api_handler_;
  RuntimeDiagnosticsApiHandler runtime_diagnostics_api_handler_;
  bool api_registered_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
