#include "ov3660_camera_configurator.h"

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "ov3660_config";
constexpr int PCLK_RATIO_REGISTER = 0x3824;
constexpr int VFIFO_CTRL0C_REGISTER = 0x460C;
constexpr int PCLK_RATIO_MASK = 0x1F;
constexpr int PCLK_MANUAL_ENABLE_MASK = 0x02;
constexpr uint32_t RETRY_INTERVAL_MS = 500;
}

Ov3660CameraConfigurator::Ov3660CameraConfigurator()
    : requested_pclk_divider_(0),
      sensor_detected_(false),
      pclk_divider_applied_(false),
      last_attempt_ms_(0),
      original_pclk_ratio_(-1),
      applied_pclk_ratio_(-1),
      vfifo_ctrl0c_(-1) {}

void Ov3660CameraConfigurator::set_pclk_divider(uint8_t divider) {
  this->requested_pclk_divider_ = divider & PCLK_RATIO_MASK;
}

void Ov3660CameraConfigurator::setup() {
  this->sensor_detected_ = false;
  this->pclk_divider_applied_ = false;
  this->last_attempt_ms_ = 0;
  this->original_pclk_ratio_ = -1;
  this->applied_pclk_ratio_ = -1;
  this->vfifo_ctrl0c_ = -1;
}

void Ov3660CameraConfigurator::loop() {
  if (this->requested_pclk_divider_ == 0 || this->pclk_divider_applied_) {
    return;
  }

  const uint32_t now = millis();
  if (now - this->last_attempt_ms_ < RETRY_INTERVAL_MS) {
    return;
  }

  this->last_attempt_ms_ = now;
  this->apply_pclk_divider_();
}

bool Ov3660CameraConfigurator::sensor_detected() const {
  return this->sensor_detected_;
}

bool Ov3660CameraConfigurator::pclk_divider_applied() const {
  return this->pclk_divider_applied_;
}

uint8_t Ov3660CameraConfigurator::requested_pclk_divider() const {
  return this->requested_pclk_divider_;
}

int Ov3660CameraConfigurator::original_pclk_ratio() const {
  return this->original_pclk_ratio_;
}

int Ov3660CameraConfigurator::applied_pclk_ratio() const {
  return this->applied_pclk_ratio_;
}

int Ov3660CameraConfigurator::vfifo_ctrl0c() const {
  return this->vfifo_ctrl0c_;
}

bool Ov3660CameraConfigurator::apply_pclk_divider_() {
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

  const int current_ratio = sensor->get_reg(sensor, PCLK_RATIO_REGISTER, 0xFF);
  if (current_ratio < 0) {
    ESP_LOGE(TAG, "Lecture PCLK_RATIO 0x3824 impossible: %d", current_ratio);
    return false;
  }

  const int vfifo_ctrl = sensor->get_reg(sensor, VFIFO_CTRL0C_REGISTER, 0xFF);
  if (vfifo_ctrl < 0) {
    ESP_LOGE(TAG, "Lecture VFIFO_CTRL0C 0x460C impossible: %d", vfifo_ctrl);
    return false;
  }

  this->original_pclk_ratio_ = current_ratio & PCLK_RATIO_MASK;
  this->vfifo_ctrl0c_ = vfifo_ctrl & 0xFF;

  if ((vfifo_ctrl & PCLK_MANUAL_ENABLE_MASK) == 0) {
    ESP_LOGE(TAG, "PCLK manuel non actif dans VFIFO_CTRL0C: 0x%02X", vfifo_ctrl & 0xFF);
    return false;
  }

  const int result = sensor->set_reg(sensor, PCLK_RATIO_REGISTER, PCLK_RATIO_MASK,
                                     this->requested_pclk_divider_ & PCLK_RATIO_MASK);
  if (result != 0) {
    ESP_LOGE(TAG, "Echec reglage PCLK_RATIO: %d", result);
    return false;
  }

  const int readback = sensor->get_reg(sensor, PCLK_RATIO_REGISTER, 0xFF);
  if (readback < 0) {
    ESP_LOGE(TAG, "Lecture de controle PCLK_RATIO impossible: %d", readback);
    return false;
  }

  this->applied_pclk_ratio_ = readback & PCLK_RATIO_MASK;
  this->pclk_divider_applied_ = this->applied_pclk_ratio_ == this->requested_pclk_divider_;

  if (!this->pclk_divider_applied_) {
    ESP_LOGE(TAG, "PCLK_RATIO inattendu apres ecriture: demande=%u lecture=%d",
             static_cast<unsigned>(this->requested_pclk_divider_), this->applied_pclk_ratio_);
    return false;
  }

  ESP_LOGW(TAG,
           "TEST PCLK JPEG applique: PCLK_RATIO 0x3824: %d -> %d, VFIFO_CTRL0C=0x%02X. "
           "Avec la PLL JPEG standard a 20 MHz XCLK, divider 20 vise environ 5 MHz PCLK.",
           this->original_pclk_ratio_, this->applied_pclk_ratio_, this->vfifo_ctrl0c_ & 0xFF);

  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
