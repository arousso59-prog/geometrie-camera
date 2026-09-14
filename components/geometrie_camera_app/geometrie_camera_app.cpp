#include "geometrie_camera_app.h"

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

static const char *const TAG = "geometrie_camera_app";

void GeometrieCameraApp::setup() {
  ESP_LOGI(TAG, "Initialisation application geometrie camera");
  this->camera_manager_.setup();
}

void GeometrieCameraApp::loop() {
  this->camera_manager_.loop();
}

void GeometrieCameraApp::dump_config() {
  ESP_LOGCONFIG(TAG, "Geometrie Camera App:");
  ESP_LOGCONFIG(TAG, "  Camera ready: %s", this->camera_manager_.ready() ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Valid measurements: %u", static_cast<unsigned>(this->valid_measurement_count_));
}

std::string GeometrieCameraApp::status_text() const {
  if (!this->camera_manager_.ready()) {
    return "V0 - camera en attente de validation du pinout";
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

}  // namespace geometrie_camera_app
}  // namespace esphome
