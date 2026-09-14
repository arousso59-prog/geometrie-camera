#include "ov3660_camera_configurator.h"

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "ov3660_config";
constexpr int CLOCK_POL_CONTROL_REGISTER = 0x4740;
constexpr int PCLK_POLARITY_MASK = 0x20;
constexpr uint32_t RETRY_INTERVAL_MS = 500;
}

Ov3660CameraConfigurator::Ov3660CameraConfigurator()
    : sensor_detected_(false),
      pclk_test_applied_(false),
      last_attempt_ms_(0),
      original_clock_pol_control_(-1),
      modified_clock_pol_control_(-1) {}

void Ov3660CameraConfigurator::setup() {
  this->sensor_detected_ = false;
  this->pclk_test_applied_ = false;
  this->last_attempt_ms_ = 0;
  this->original_clock_pol_control_ = -1;
  this->modified_clock_pol_control_ = -1;
}

void Ov3660CameraConfigurator::loop() {
  if (this->pclk_test_applied_) {
    return;
  }

  const uint32_t now = millis();
  if (now - this->last_attempt_ms_ < RETRY_INTERVAL_MS) {
    return;
  }

  this->last_attempt_ms_ = now;
  this->apply_pclk_polarity_test_();
}

bool Ov3660CameraConfigurator::sensor_detected() const {
  return this->sensor_detected_;
}

bool Ov3660CameraConfigurator::pclk_test_applied() const {
  return this->pclk_test_applied_;
}

int Ov3660CameraConfigurator::original_clock_pol_control() const {
  return this->original_clock_pol_control_;
}

int Ov3660CameraConfigurator::modified_clock_pol_control() const {
  return this->modified_clock_pol_control_;
}

bool Ov3660CameraConfigurator::apply_pclk_polarity_test_() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return false;
  }

  if (sensor->id.PID != OV3660_PID) {
    ESP_LOGW(TAG, "Capteur detecte mais PID inattendu: 0x%04X", sensor->id.PID);
    return false;
  }

  this->sensor_detected_ = true;

  if (sensor->get_reg == nullptr || sensor->set_reg == nullptr) {
    ESP_LOGE(TAG, "Acces registres OV3660 indisponible");
    return false;
  }

  const int current_value = sensor->get_reg(sensor, CLOCK_POL_CONTROL_REGISTER, 0xFF);
  if (current_value < 0) {
    ESP_LOGE(TAG, "Lecture registre 0x4740 impossible: %d", current_value);
    return false;
  }

  const int requested_pclk_bit = (current_value ^ PCLK_POLARITY_MASK) & PCLK_POLARITY_MASK;
  const int result = sensor->set_reg(sensor, CLOCK_POL_CONTROL_REGISTER, PCLK_POLARITY_MASK, requested_pclk_bit);
  if (result != 0) {
    ESP_LOGE(TAG, "Echec inversion polarite PCLK: %d", result);
    return false;
  }

  const int readback_value = sensor->get_reg(sensor, CLOCK_POL_CONTROL_REGISTER, 0xFF);
  if (readback_value < 0) {
    ESP_LOGE(TAG, "Lecture de controle registre 0x4740 impossible: %d", readback_value);
    return false;
  }

  this->original_clock_pol_control_ = current_value;
  this->modified_clock_pol_control_ = readback_value;
  this->pclk_test_applied_ = true;

  ESP_LOGW(TAG,
           "TEST PCLK applique: CLOCK_POL_CONTROL 0x4740: 0x%02X -> 0x%02X (bit5 inverse)",
           current_value & 0xFF, readback_value & 0xFF);

  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
