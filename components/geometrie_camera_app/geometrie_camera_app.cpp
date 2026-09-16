#include "geometrie_camera_app.h"

#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

static const char *const TAG = "geometrie_camera_app";

GeometrieCameraApp::GeometrieCameraApp()
    : measurement_manager_(),
      target_detector_(),
      resolution_controller_(),
      settings_controller_(),
      jpeg_diagnostic_(),
      image_sharpness_evaluator_(&this->jpeg_diagnostic_),
      jpeg_filtered_diagnostic_(&this->jpeg_diagnostic_),
      target_detection_service_(&this->jpeg_filtered_diagnostic_, &this->target_detector_),
      target_detection_preview_(),
      continuous_measurement_controller_(&this->jpeg_diagnostic_, &this->image_sharpness_evaluator_,
                                         &this->jpeg_filtered_diagnostic_, &this->target_detection_service_,
                                         &this->measurement_manager_),
      runtime_diagnostics_(),
      api_wsdl_handler_(&this->resolution_controller_),
      settings_api_handler_(&this->settings_controller_, &this->resolution_controller_),
      jpeg_diagnostic_api_handler_(&this->jpeg_diagnostic_, &this->resolution_controller_),
      jpeg_filtered_diagnostic_api_handler_(&this->jpeg_filtered_diagnostic_),
      target_detection_api_handler_(&this->target_detection_service_, &this->jpeg_filtered_diagnostic_,
                                    &this->target_detection_preview_),
      measurement_api_handler_(&this->measurement_manager_, &this->target_detection_service_,
                               &this->jpeg_filtered_diagnostic_),
      continuous_measurement_api_handler_(&this->continuous_measurement_controller_),
      runtime_diagnostics_api_handler_(&this->runtime_diagnostics_, &this->jpeg_diagnostic_),
      api_registered_(false) {}

void GeometrieCameraApp::setup() {
  ESP_LOGI(TAG, "Initialisation application geometrie camera");
  this->measurement_manager_.setup();
  this->resolution_controller_.sync_from_sensor();
  this->register_api_if_possible_();
}

void GeometrieCameraApp::loop() {
  this->runtime_diagnostics_.record_loop();
  this->jpeg_diagnostic_.loop();
  this->continuous_measurement_controller_.loop();

  if (!this->api_registered_) {
    this->register_api_if_possible_();
  }
}

void GeometrieCameraApp::dump_config() {
  ESP_LOGCONFIG(TAG, "Geometrie Camera App:");
  ESP_LOGCONFIG(TAG, "  API HTTP: %s", this->api_registered_ ? "REGISTERED" : "WAITING");
  ESP_LOGCONFIG(TAG, "  API WSDL: /api/wsdl");
  ESP_LOGCONFIG(TAG, "  Runtime diagnostics API: /api/runtime/status");
  ESP_LOGCONFIG(TAG, "  Camera settings API: /api/camera/settings*");
  ESP_LOGCONFIG(TAG, "  JPEG capture/filter API: /diagnostic-jpeg/*");
  ESP_LOGCONFIG(TAG, "  Target detection API: /target/detect, /target/status, /target/preview.bmp");
  ESP_LOGCONFIG(TAG, "  Measurement API: /measurement/*");
  ESP_LOGCONFIG(TAG, "  Continuous API: /continuous/start, /continuous/stop, /continuous/status");
  ESP_LOGCONFIG(TAG, "  Target physical size: %.2f mm", this->measurement_manager_.measurement_engine().target_size_mm());
  ESP_LOGCONFIG(TAG, "  Geometry calibration: %s",
                this->measurement_manager_.measurement_engine().has_calibration() ? "VALID" : "REQUIRED");
  ESP_LOGCONFIG(TAG, "  Resolution active: %s", this->resolution_controller_.active_resolution().c_str());
  ESP_LOGCONFIG(TAG, "  Continuous interval: %u ms",
                static_cast<unsigned>(this->continuous_measurement_controller_.interval_ms()));
  ESP_LOGCONFIG(TAG, "  Sharpness guard: JPEG 1/8, maximum 2 recaptures");
  ESP_LOGCONFIG(TAG, "  JPEG ready: %s", this->jpeg_diagnostic_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  JPEG filtered ready: %s", this->jpeg_filtered_diagnostic_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Target detection ready: %s", this->target_detection_service_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Valid measurements: %u",
                static_cast<unsigned>(this->measurement_manager_.valid_measurement_count()));
}

void GeometrieCameraApp::set_camera(esp32_camera::ESP32Camera *camera) {
  this->jpeg_diagnostic_.set_camera(camera);
}

std::string GeometrieCameraApp::status_text() const {
  if (!this->api_registered_) {
    return "API camera en attente du serveur web";
  }

  if (this->continuous_measurement_controller_.running()) {
    if (this->continuous_measurement_controller_.cycle_count() == 0) {
      return std::string("Mesure continue - ") + this->continuous_measurement_controller_.state_text();
    }
    if (this->continuous_measurement_controller_.last_cycle_measurement_valid()) {
      return "Mesure continue - cible trouvee - mesure valide";
    }
    if (this->continuous_measurement_controller_.last_cycle_target_found()) {
      return "Mesure continue - cible trouvee - mesure invalide";
    }
    return "Mesure continue - cible non trouvee";
  }

  if (this->continuous_measurement_controller_.state() == ContinuousMeasurementState::ERROR) {
    return std::string("Mesure continue arretee - ") + this->continuous_measurement_controller_.last_error();
  }

  if (this->jpeg_diagnostic_.capture_pending()) {
    return "Capture JPEG en cours";
  }
  if (!this->jpeg_diagnostic_.ready()) {
    return "Camera JPEG prete - aucune capture valide";
  }
  if (!this->target_detection_service_.ready()) {
    return "Image JPEG valide - cible a detecter";
  }
  if (!this->target_detection_service_.target_found()) {
    return "Detection terminee - cible non trouvee";
  }
  if (!this->measurement_manager_.measurement_engine().has_calibration()) {
    return "Cible detectee - calibration distance requise";
  }
  if (!this->measurement_manager_.last_measurement().valid) {
    return "Cible detectee - mesure a lancer";
  }
  return "Camera prete - mesure valide";
}

uint32_t GeometrieCameraApp::valid_measurement_count() const {
  return this->measurement_manager_.valid_measurement_count();
}

const GeometryMeasurement &GeometrieCameraApp::last_measurement() const {
  return this->measurement_manager_.last_measurement();
}

bool GeometrieCameraApp::start_continuous_measurement() {
  return this->continuous_measurement_controller_.start(this->continuous_measurement_controller_.interval_ms());
}

void GeometrieCameraApp::stop_continuous_measurement() {
  this->continuous_measurement_controller_.stop();
}

bool GeometrieCameraApp::set_continuous_interval_ms(uint32_t interval_ms) {
  return this->continuous_measurement_controller_.set_interval_ms(interval_ms);
}

bool GeometrieCameraApp::continuous_running() const { return this->continuous_measurement_controller_.running(); }
const char *GeometrieCameraApp::continuous_state_text() const { return this->continuous_measurement_controller_.state_text(); }
uint32_t GeometrieCameraApp::continuous_interval_ms() const { return this->continuous_measurement_controller_.interval_ms(); }
uint32_t GeometrieCameraApp::continuous_cycle_count() const { return this->continuous_measurement_controller_.cycle_count(); }
uint32_t GeometrieCameraApp::continuous_last_cycle_ms() const { return this->continuous_measurement_controller_.last_cycle_ms(); }
bool GeometrieCameraApp::continuous_target_found() const { return this->continuous_measurement_controller_.last_cycle_target_found(); }
bool GeometrieCameraApp::continuous_measurement_valid() const { return this->continuous_measurement_controller_.last_cycle_measurement_valid(); }
std::string GeometrieCameraApp::active_resolution_text() const { return this->resolution_controller_.active_resolution(); }

MeasurementManager &GeometrieCameraApp::measurement_manager() { return this->measurement_manager_; }
TargetDetector &GeometrieCameraApp::target_detector() { return this->target_detector_; }
GeometryMeasurementEngine &GeometrieCameraApp::measurement_engine() { return this->measurement_manager_.measurement_engine(); }
CameraResolutionController &GeometrieCameraApp::resolution_controller() { return this->resolution_controller_; }
CameraSettingsController &GeometrieCameraApp::settings_controller() { return this->settings_controller_; }
JpegDiagnostic &GeometrieCameraApp::jpeg_diagnostic() { return this->jpeg_diagnostic_; }
JpegFilteredDiagnostic &GeometrieCameraApp::jpeg_filtered_diagnostic() { return this->jpeg_filtered_diagnostic_; }
TargetDetectionService &GeometrieCameraApp::target_detection_service() { return this->target_detection_service_; }
ContinuousMeasurementController &GeometrieCameraApp::continuous_measurement_controller() { return this->continuous_measurement_controller_; }

void GeometrieCameraApp::register_api_if_possible_() {
  if (this->api_registered_ || web_server_base::global_web_server_base == nullptr) {
    return;
  }

  web_server_base::global_web_server_base->add_handler(&this->api_wsdl_handler_);
  web_server_base::global_web_server_base->add_handler(&this->settings_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->jpeg_diagnostic_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->jpeg_filtered_diagnostic_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->target_detection_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->measurement_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->continuous_measurement_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->runtime_diagnostics_api_handler_);
  this->api_registered_ = true;

  ESP_LOGI(TAG,
           "API HTTP camera enregistree: /api/wsdl, /api/runtime/status, /api/camera/settings*, "
           "/diagnostic-jpeg/*, /target/*, /measurement/* et /continuous/*");
}

}  // namespace geometrie_camera_app
}  // namespace esphome
