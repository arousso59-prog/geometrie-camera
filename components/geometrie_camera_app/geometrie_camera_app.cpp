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
      rgb565_diagnostic_(),
      target_search_diagnostic_(&this->measurement_manager_.target_detector(), &this->grayscale_diagnostic_),
      camera_configurator_(),
      api_handler_(&this->camera_manager_, &this->measurement_manager_),
      diagnostic_api_handler_(&this->grayscale_diagnostic_),
      rgb565_diagnostic_api_handler_(&this->rgb565_diagnostic_),
      target_search_diagnostic_api_handler_(&this->target_search_diagnostic_, &this->grayscale_diagnostic_),
      api_registered_(false) {}

void GeometrieCameraApp::setup() {
  ESP_LOGI(TAG, "Initialisation application geometrie camera");

  this->measurement_manager_.setup();
  this->camera_manager_.setup();
  this->camera_configurator_.setup();
  this->register_api_if_possible_();
}

void GeometrieCameraApp::loop() {
  this->camera_manager_.loop();
  this->camera_configurator_.loop();

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
  ESP_LOGCONFIG(TAG, "  RGB565 diagnostic ready: %s", this->rgb565_diagnostic_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Target search diagnostic ready: %s", this->target_search_diagnostic_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  OV3660 detected: %s", this->camera_configurator_.sensor_detected() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  PCLK divider requested: %u",
                static_cast<unsigned>(this->camera_configurator_.requested_pclk_divider()));
  ESP_LOGCONFIG(TAG, "  PCLK divider applied: %s",
                this->camera_configurator_.pclk_divider_applied() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Valid measurements: %u",
                static_cast<unsigned>(this->measurement_manager_.valid_measurement_count()));
}

void GeometrieCameraApp::set_camera(esp32_camera::ESP32Camera *camera) {
  this->grayscale_diagnostic_.set_camera(camera);
  this->rgb565_diagnostic_.set_camera(camera);
  this->target_search_diagnostic_.set_camera(camera);
}

void GeometrieCameraApp::set_ov3660_pclk_divider(uint8_t divider) {
  this->camera_configurator_.set_pclk_divider(divider);
}

std::string GeometrieCameraApp::status_text() const {
  if (!this->api_registered_) {
    return "V0 - API camera en attente du serveur web";
  }

  if (this->camera_manager_.placeholder_mode()) {
    return "V0 - API bouchon active - diagnostic camera disponible";
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

CameraManager &GeometrieCameraApp::camera_manager() { return this->camera_manager_; }
MeasurementManager &GeometrieCameraApp::measurement_manager() { return this->measurement_manager_; }
TargetDetector &GeometrieCameraApp::target_detector() { return this->measurement_manager_.target_detector(); }
GeometryMeasurementEngine &GeometrieCameraApp::measurement_engine() { return this->measurement_manager_.measurement_engine(); }
GrayscaleDiagnostic &GeometrieCameraApp::grayscale_diagnostic() { return this->grayscale_diagnostic_; }
Rgb565Diagnostic &GeometrieCameraApp::rgb565_diagnostic() { return this->rgb565_diagnostic_; }
TargetSearchDiagnostic &GeometrieCameraApp::target_search_diagnostic() { return this->target_search_diagnostic_; }
Ov3660CameraConfigurator &GeometrieCameraApp::camera_configurator() { return this->camera_configurator_; }

void GeometrieCameraApp::register_api_if_possible_() {
  if (this->api_registered_ || web_server_base::global_web_server_base == nullptr) {
    return;
  }

  web_server_base::global_web_server_base->add_handler(&this->api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->diagnostic_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->rgb565_diagnostic_api_handler_);
  web_server_base::global_web_server_base->add_handler(&this->target_search_diagnostic_api_handler_);
  this->api_registered_ = true;

  ESP_LOGI(TAG, "API HTTP camera enregistree: /api/*, /diagnostic/*, /diagnostic-rgb565/* et /target/*");
}

}  // namespace geometrie_camera_app
}  // namespace esphome
