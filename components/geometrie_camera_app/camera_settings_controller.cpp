#include "camera_settings_controller.h"

#include <cstdio>

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "camera_settings";

sensor_t *get_sensor(std::string &error) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    error = "camera_sensor_unavailable";
  }
  return sensor;
}

const char *pixel_format_to_text(pixformat_t format) {
  switch (format) {
    case PIXFORMAT_JPEG:
      return "jpeg";
    case PIXFORMAT_GRAYSCALE:
      return "grayscale";
    case PIXFORMAT_RGB565:
      return "rgb565";
    case PIXFORMAT_YUV422:
      return "yuv422";
    case PIXFORMAT_RGB888:
      return "rgb888";
    case PIXFORMAT_RAW:
      return "raw";
    default:
      return "unknown";
  }
}
}  // namespace

CameraSettingsController::CameraSettingsController() {}

CameraSettingsSnapshot CameraSettingsController::read() const {
  CameraSettingsSnapshot snapshot{};
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    snapshot.available = false;
    return snapshot;
  }

  snapshot.available = true;
  snapshot.pixel_format = pixel_format_to_text(sensor->pixformat);
  snapshot.brightness = sensor->status.brightness;
  snapshot.contrast = sensor->status.contrast;
  snapshot.exposure_ctrl = sensor->status.aec != 0;
  snapshot.ae_level = sensor->status.ae_level;
  snapshot.aec_value = sensor->status.aec_value;
  snapshot.gain_ctrl = sensor->status.agc != 0;
  snapshot.agc_gain = sensor->status.agc_gain;
  return snapshot;
}

bool CameraSettingsController::validate_range_(const char *name, int value, int minimum, int maximum,
                                               std::string &error) const {
  if (value >= minimum && value <= maximum) {
    return true;
  }

  char buffer[96];
  std::snprintf(buffer, sizeof(buffer), "%s_out_of_range_%d_%d", name, minimum, maximum);
  error = buffer;
  return false;
}

bool CameraSettingsController::set_brightness(int value, std::string &error) const {
  if (!this->validate_range_("brightness", value, -2, 2, error)) return false;
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_brightness == nullptr) return false;
  if (sensor->set_brightness(sensor, value) != 0) {
    error = "brightness_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "Brightness applique: %d", value);
  return true;
}

bool CameraSettingsController::set_contrast(int value, std::string &error) const {
  if (!this->validate_range_("contrast", value, -2, 2, error)) return false;
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_contrast == nullptr) return false;
  if (sensor->set_contrast(sensor, value) != 0) {
    error = "contrast_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "Contraste applique: %d", value);
  return true;
}

bool CameraSettingsController::set_exposure_ctrl(bool enabled, std::string &error) const {
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_exposure_ctrl == nullptr) return false;
  if (sensor->set_exposure_ctrl(sensor, enabled ? 1 : 0) != 0) {
    error = "exposure_ctrl_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "Auto exposition: %s", enabled ? "ON" : "OFF");
  return true;
}

bool CameraSettingsController::set_ae_level(int value, std::string &error) const {
  if (!this->validate_range_("ae_level", value, -2, 2, error)) return false;
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_ae_level == nullptr) return false;
  if (sensor->set_ae_level(sensor, value) != 0) {
    error = "ae_level_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "AE level applique: %d", value);
  return true;
}

bool CameraSettingsController::set_aec_value(int value, std::string &error) const {
  if (!this->validate_range_("aec_value", value, 0, 1200, error)) return false;
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_aec_value == nullptr) return false;
  if (sensor->set_aec_value(sensor, value) != 0) {
    error = "aec_value_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "AEC value appliquee: %d", value);
  return true;
}

bool CameraSettingsController::set_gain_ctrl(bool enabled, std::string &error) const {
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_gain_ctrl == nullptr) return false;
  if (sensor->set_gain_ctrl(sensor, enabled ? 1 : 0) != 0) {
    error = "gain_ctrl_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "Auto gain: %s", enabled ? "ON" : "OFF");
  return true;
}

bool CameraSettingsController::set_agc_gain(int value, std::string &error) const {
  if (!this->validate_range_("agc_gain", value, 0, 30, error)) return false;
  sensor_t *sensor = get_sensor(error);
  if (sensor == nullptr || sensor->set_agc_gain == nullptr) return false;
  if (sensor->set_agc_gain(sensor, value) != 0) {
    error = "agc_gain_apply_failed";
    return false;
  }
  ESP_LOGI(TAG, "AGC gain applique: %d", value);
  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
