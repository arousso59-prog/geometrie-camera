#include "geometrie_camera_app.h"

#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

static const char *const TAG = "geometrie_camera_app";

GeometrieCameraApp::GeometrieCameraApp() : api_handler_(this) {}

void GeometrieCameraApp::setup() {
  ESP_LOGI(TAG, "Initialisation application geometrie camera");
  this->camera_manager_.setup();
  this->register_api_if_possible_();
}

void GeometrieCameraApp::loop() {
  this->camera_manager_.loop();

  // Rend l'enregistrement robuste meme si l'ordre de setup ESPHome change.
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
  ESP_LOGCONFIG(TAG, "  Valid measurements: %u", static_cast<unsigned>(this->valid_measurement_count_));
}

std::string GeometrieCameraApp::status_text() const {
  if (!this->api_registered_) {
    return "V0 - API camera en attente du serveur web";
  }

  if (this->camera_manager_.placeholder_mode()) {
    return "V0 - API bouchon active - image noir/blanc";
  }

  if (!this->last_measurement_.valid) {
    return "Camera prete - aucune mesure valide";
  }

  return "Camera prete - mesure valide";
}

uint32_t GeometrieCameraApp::valid_measurement_count() const {
  return this->valid_measurement_count_;
}

const GeometryMeasurement &GeometrieCameraApp::last_measurement() const {
  return this->last_measurement_;
}

CameraManager &GeometrieCameraApp::camera_manager() {
  return this->camera_manager_;
}

TargetDetector &GeometrieCameraApp::target_detector() {
  return this->target_detector_;
}

GeometryMeasurementEngine &GeometrieCameraApp::measurement_engine() {
  return this->measurement_engine_;
}

void GeometrieCameraApp::register_api_if_possible_() {
  if (this->api_registered_ || web_server_base::global_web_server_base == nullptr) {
    return;
  }

  web_server_base::global_web_server_base->add_handler(&this->api_handler_);
  this->api_registered_ = true;

  ESP_LOGI(TAG, "API HTTP camera enregistree: /api/status /api/capture /api/measure /image.jpg");
}

}  // namespace geometrie_camera_app
}  // namespace esphome
