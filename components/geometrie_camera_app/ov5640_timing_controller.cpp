#include "ov5640_timing_controller.h"

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "ov5640_timing";
}

Ov5640TimingController::Ov5640TimingController() {}

Ov5640TimingSnapshot Ov5640TimingController::snapshot() const {
  Ov5640TimingSnapshot result{};
  result.sensor_available = false;
  result.sensor_is_ov5640 = false;
  result.sensor_pid = 0;
  result.pclk_divider = -1;
  result.vfifo_ctrl0c = -1;
  result.pclk_manual = false;

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return result;
  }

  result.sensor_available = true;
  result.sensor_pid = sensor->id.PID;
  result.sensor_is_ov5640 = sensor->id.PID == OV5640_PID;

  if (!result.sensor_is_ov5640 || sensor->get_reg == nullptr) {
    return result;
  }

  const int pclk_ratio = sensor->get_reg(sensor, PCLK_RATIO_REGISTER, 0xFF);
  const int vfifo_ctrl = sensor->get_reg(sensor, VFIFO_CTRL0C_REGISTER, 0xFF);

  if (pclk_ratio >= 0) {
    result.pclk_divider = pclk_ratio & PCLK_RATIO_MASK;
  }
  if (vfifo_ctrl >= 0) {
    result.vfifo_ctrl0c = vfifo_ctrl & 0xFF;
    result.pclk_manual = (vfifo_ctrl & PCLK_MANUAL_ENABLE_MASK) != 0;
  }

  return result;
}

bool Ov5640TimingController::set_pclk_divider(uint8_t divider) {
  if (divider < min_pclk_divider() || divider > max_pclk_divider()) {
    ESP_LOGW(TAG, "Diviseur PCLK invalide: %u", static_cast<unsigned>(divider));
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    ESP_LOGE(TAG, "Capteur camera indisponible");
    return false;
  }

  if (sensor->id.PID != OV5640_PID) {
    ESP_LOGE(TAG, "Capteur non OV5640: PID=0x%04X", sensor->id.PID);
    return false;
  }

  if (sensor->get_reg == nullptr || sensor->set_reg == nullptr) {
    ESP_LOGE(TAG, "Acces registres OV5640 indisponible");
    return false;
  }

  const int vfifo_ctrl = sensor->get_reg(sensor, VFIFO_CTRL0C_REGISTER, 0xFF);
  if (vfifo_ctrl < 0) {
    ESP_LOGE(TAG, "Lecture VFIFO_CTRL0C impossible: %d", vfifo_ctrl);
    return false;
  }

  if ((vfifo_ctrl & PCLK_MANUAL_ENABLE_MASK) == 0) {
    ESP_LOGE(TAG, "PCLK manuel non actif: VFIFO_CTRL0C=0x%02X", vfifo_ctrl & 0xFF);
    return false;
  }

  const int before = sensor->get_reg(sensor, PCLK_RATIO_REGISTER, 0xFF);
  if (before < 0) {
    ESP_LOGE(TAG, "Lecture PCLK_RATIO impossible: %d", before);
    return false;
  }

  const int write_result = sensor->set_reg(sensor, PCLK_RATIO_REGISTER, PCLK_RATIO_MASK,
                                           divider & PCLK_RATIO_MASK);
  if (write_result != 0) {
    ESP_LOGE(TAG, "Ecriture PCLK_RATIO impossible: %d", write_result);
    return false;
  }

  const int after = sensor->get_reg(sensor, PCLK_RATIO_REGISTER, 0xFF);
  if (after < 0) {
    ESP_LOGE(TAG, "Lecture de controle PCLK_RATIO impossible: %d", after);
    return false;
  }

  const uint8_t applied = static_cast<uint8_t>(after) & PCLK_RATIO_MASK;
  if (applied != divider) {
    ESP_LOGE(TAG, "Readback PCLK inattendu: demande=%u lecture=%u",
             static_cast<unsigned>(divider), static_cast<unsigned>(applied));
    return false;
  }

  ESP_LOGW(TAG, "OV5640 PCLK divider: %u -> %u (VFIFO_CTRL0C=0x%02X)",
           static_cast<unsigned>(before & PCLK_RATIO_MASK), static_cast<unsigned>(applied),
           vfifo_ctrl & 0xFF);
  return true;
}

uint8_t Ov5640TimingController::min_pclk_divider() { return 1; }
uint8_t Ov5640TimingController::max_pclk_divider() { return PCLK_RATIO_MASK; }

}  // namespace geometrie_camera_app
}  // namespace esphome
