#include "geometrie_camera_app.h"

#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

static const char *const TAG = "geometrie_camera_app";

GeometrieCameraApp::GeometrieCameraApp()
    : placeholder_image_provider_(),
      camera_manager_(&this->placeholder_image_provider_),
      measurement_manager_(),
      grayscale_diagnostic_(),
      api_handler_(&this->camera_manager_, &this->measurement_manager_),
      diagnostic_api_handler_(&this->grayscale_diagnostic_),
      api_registered_(false) {}

void GeometrieCameraApp::setup() {
  ESP_LOGI(TAG, "Initialisation application geometrie camera");

  this->measurement_manager_.setup();
  this->camera_manager_.setup();
  this->register_api_if_possible_();
}

void GeometrieCameraApp::loop() {
  this->camera_manager_.loop();

  if (!this->api_registered_) {
    this->register_api_if_possible_();
  }
}

void GeometrieCameraApp::dump_config() {
  ESP_LOGCONFIG(TAG, "Geometrie Camera App:");
  ESP_LOGCONFIG(TAG, "  API HTTP: %s", this->api_registered_ ? "REGISTERED" : "WAITING");
  ESP_LOGCONFIG(TAG, "  Camera service ready: %s", this->camera_manager_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Physical camera ready: %s", this->camera_manager_.physical_camera_ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Placeholder mode: %s", this->camera_manager_.placeholder_mode() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Grayscale diagnostic ready: %s", this->grayscale_diagnostic_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Valid measurements: %u",
                static_cast<unsigned>(this->measurement_manager_.valid_measurement_count()));
}

void GeometrieCameraApp::set_camera(esp32_camera::ESP32Camera *camera) {
  this->grayscale_diagnostic_.set_camera(camera);
}

std::string GeometrieCameraApp::status_text() const {
  if (!this->api_registered_) {
    return "V0 - API camera en attente du serveur web";
  }

  if (this->camera_manager_.placeholder_mode()) {
    return "V0 - API bouchon active - diagnostic camera brute disponible";
  }

  if (!this->measurement_manager_.last_measurement().valid) {
    return "Camera prete - aucune mesure valide";
  }

  return "Camera prete - mesure valide";
}

uint32_t GeometrieCameraApp::valid_measurement_count() const {
  return this->measurement_manager_.valid_measurement_count();
}

const GeometryMeasurement &GeometrieCameraApp::last_measurement() const {
  return this->measurement_manager_.last_measurement();
}

CameraManager &GeometrieCameraApp::camera_manager() {
  return this->camera_manager_;
}

MeasurementManager &GeometrieCameraApp::measurement_manager() {
  return this->measurement_manager_;
}

TargetDetector &GeometrieCameraApp::target_detector() {
  return this->measurement_manager_.target_detector();
}

GeometryMeasurementEngine &GeometrieCameraApp::measurement_engine() {
  return this->measurement_manager_.measurement_engine();
}

GrayscaleDiagnostic &GeometrieCameraApp::grayscale_diagnostic() {
  return this->grayscale_diagnostic_;
}

void GeometrieCameraApp::register_api_if_possible_() {
  if (this->api_registered_ || web_server_base::global_web_server_base == nullptr) {
    return;
  }

  web_server_base::global_web_server_base->add_handler(&this->api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->diagnostic_api_handler_);
  this->api_registered_ = true;

  ESP_LOGI(TAG, "API HTTP camera enregistree: /api/* et /diagnostic/*");
}

}  // namespace geometrie_camera_app
}  // namespace esphome
