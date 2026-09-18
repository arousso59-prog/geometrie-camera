#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "api_wsdl.h"
#include "camera_resolution_controller.h"
#include "camera_settings_controller.h"
#include "camera_viewport_controller.h"
#include "continuous_measurement_api.h"
#include "continuous_measurement_controller.h"
#include "full_calibration_api.h"
#include "full_calibration_controller.h"
#include "jpeg_diagnostic.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_api.h"
#include "measurement_manager.h"
#include "runtime_diagnostics.h"
#include "runtime_diagnostics_api.h"
#include "target_detection_api.h"
#include "target_detection_preview.h"
#include "target_detection_service.h"
#include "target_detector.h"
#include "target_tracking_api.h"
#include "target_tracking_controller.h"
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

  bool start_continuous_measurement();
  void stop_continuous_measurement();
  bool set_continuous_interval_ms(uint32_t interval_ms);
  bool continuous_running() const;
  const char *continuous_state_text() const;
  uint32_t continuous_interval_ms() const;
  uint32_t continuous_cycle_count() const;
  uint32_t continuous_last_cycle_ms() const;
  bool continuous_target_found() const;
  bool continuous_measurement_valid() const;
  std::string active_resolution_text() const;

  MeasurementManager &measurement_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();
  CameraResolutionController &resolution_controller();
  CameraSettingsController &settings_controller();
  CameraViewportController &viewport_controller();
  TargetTrackingController &tracking_controller();
  JpegDiagnostic &jpeg_diagnostic();
  JpegFilteredDiagnostic &jpeg_filtered_diagnostic();
  TargetDetectionService &target_detection_service();
  ContinuousMeasurementController &continuous_measurement_controller();
  FullCalibrationController &full_calibration_controller();

 private:
  void register_api_if_possible_();

  MeasurementManager measurement_manager_;
  TargetDetector target_detector_;
  CameraResolutionController resolution_controller_;
  CameraSettingsController settings_controller_;
  CameraViewportController viewport_controller_;
  TargetTrackingController tracking_controller_;
  JpegDiagnostic jpeg_diagnostic_;
  JpegFilteredDiagnostic jpeg_filtered_diagnostic_;
  TargetDetectionService target_detection_service_;
  TargetDetectionPreview target_detection_preview_;
  ContinuousMeasurementController continuous_measurement_controller_;
  FullCalibrationController full_calibration_controller_;
  RuntimeDiagnostics runtime_diagnostics_;
  ApiWsdlHandler api_wsdl_handler_;
  TargetTrackingApiHandler tracking_api_handler_;
  TargetDetectionApiHandler target_detection_api_handler_;
  MeasurementApiHandler measurement_api_handler_;
  ContinuousMeasurementApiHandler continuous_measurement_api_handler_;
  FullCalibrationApiHandler full_calibration_api_handler_;
  RuntimeDiagnosticsApiHandler runtime_diagnostics_api_handler_;
  bool api_registered_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
