#include "ov5640_timing_controller.h"

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "ov5640_timing";
constexpr uint16_t PCLK_RATIO_REGISTER = 0x3824;
constexpr uint16_t VFIFO_CTRL0C_REGISTER = 0x460C;
constexpr uint16_t HTS_HIGH_REGISTER = 0x380C;
constexpr uint16_t VTS_HIGH_REGISTER = 0x380E;
constexpr uint16_t JPEG_MODE_REGISTER = 0x4713;
constexpr uint16_t DVP_HREF_CTRL_REGISTER = 0x471F;
constexpr uint8_t PCLK_RATIO_MASK = 0x1F;
constexpr uint8_t PCLK_MANUAL_ENABLE_MASK = 0x02;
constexpr uint8_t MIN_XCLK_MHZ = 5;
constexpr uint8_t MAX_XCLK_MHZ = 8;
constexpr uint8_t MIN_PCLK_DIVIDER = 1;
constexpr uint8_t MAX_PCLK_DIVIDER = PCLK_RATIO_MASK;
constexpr uint16_t MIN_TOTAL_TIMING = 1;
constexpr uint16_t MAX_TOTAL_TIMING = 0xFFFF;
constexpr uint8_t MIN_HREF_BLANKING = 0;
constexpr uint8_t MAX_HREF_BLANKING = 0xFF;
constexpr int XCLK_TIMER_ARGUMENT = 0;
constexpr int HZ_PER_MHZ = 1000000;
}

Ov5640TimingController::Ov5640TimingController()
    : baseline_available_(false),
      baseline_xclk_mhz_(0),
      baseline_hts_(0),
      baseline_vts_(0),
      baseline_jpeg_mode_(0),
      baseline_href_blanking_(0) {}

Ov5640TimingSnapshot Ov5640TimingController::snapshot() const {
  Ov5640TimingSnapshot result{};
  result.sensor_available = false;
  result.sensor_is_ov5640 = false;
  result.sensor_pid = 0;
  result.xclk_mhz = -1;
  result.pclk_divider = -1;
  result.vfifo_ctrl0c = -1;
  result.pclk_manual = false;
  result.hts = -1;
  result.vts = -1;
  result.jpeg_mode = -1;
  result.href_blanking = -1;
  result.baseline_available = this->baseline_available_;
  result.baseline_xclk_mhz = this->baseline_xclk_mhz_;
  result.baseline_hts = this->baseline_hts_;
  result.baseline_vts = this->baseline_vts_;
  result.baseline_jpeg_mode = this->baseline_jpeg_mode_;
  result.baseline_href_blanking = this->baseline_href_blanking_;

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return result;
  }

  result.sensor_available = true;
  result.sensor_pid = sensor->id.PID;
  result.sensor_is_ov5640 = sensor->id.PID == OV5640_PID;
  if (sensor->xclk_freq_hz > 0) {
    result.xclk_mhz = sensor->xclk_freq_hz / HZ_PER_MHZ;
  }

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

  uint16_t hts = 0;
  uint16_t vts = 0;
  if (this->read_total_timing_(&hts, &vts)) {
    result.hts = hts;
    result.vts = vts;
  }

  uint8_t jpeg_mode = 0;
  uint8_t href_blanking = 0;
  if (this->read_jpeg_output_timing_(&jpeg_mode, &href_blanking)) {
    result.jpeg_mode = jpeg_mode;
    result.href_blanking = href_blanking;
  }

  return result;
}

bool Ov5640TimingController::set_xclk_mhz(uint8_t mhz) {
  if (mhz < min_xclk_mhz() || mhz > max_xclk_mhz()) {
    ESP_LOGW(TAG, "XCLK de test invalide: %u MHz", static_cast<unsigned>(mhz));
    return false;
  }

  if (!this->capture_baseline_if_needed_()) {
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
  if (sensor->set_xclk == nullptr) {
    ESP_LOGE(TAG, "Callback set_xclk OV5640 indisponible");
    return false;
  }

  const int before_hz = sensor->xclk_freq_hz;
  const int result = sensor->set_xclk(sensor, XCLK_TIMER_ARGUMENT, mhz);
  if (result != 0) {
    ESP_LOGE(TAG, "Changement XCLK vers %u MHz impossible: %d", static_cast<unsigned>(mhz), result);
    if (before_hz > 0 && before_hz % HZ_PER_MHZ == 0) {
      sensor->set_xclk(sensor, XCLK_TIMER_ARGUMENT, before_hz / HZ_PER_MHZ);
    }
    return false;
  }

  const int expected_hz = static_cast<int>(mhz) * HZ_PER_MHZ;
  if (sensor->xclk_freq_hz != expected_hz) {
    ESP_LOGE(TAG, "Readback XCLK inattendu: demande=%d Hz lecture=%d Hz", expected_hz,
             sensor->xclk_freq_hz);
    if (before_hz > 0 && before_hz % HZ_PER_MHZ == 0) {
      sensor->set_xclk(sensor, XCLK_TIMER_ARGUMENT, before_hz / HZ_PER_MHZ);
    }
    return false;
  }

  ESP_LOGW(TAG, "OV5640 XCLK runtime: %d MHz -> %u MHz", before_hz / HZ_PER_MHZ,
           static_cast<unsigned>(mhz));
  return true;
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

bool Ov5640TimingController::set_hts(uint16_t hts) {
  if (hts < min_total_timing()) {
    return false;
  }

  if (!this->capture_baseline_if_needed_()) {
    return false;
  }

  uint16_t before_hts = 0;
  uint16_t before_vts = 0;
  if (!this->read_total_timing_(&before_hts, &before_vts)) {
    return false;
  }

  if (!this->write_total_timing_register_(HTS_HIGH_REGISTER, hts)) {
    return false;
  }

  uint16_t after_hts = 0;
  uint16_t after_vts = 0;
  if (!this->read_total_timing_(&after_hts, &after_vts) || after_hts != hts) {
    ESP_LOGE(TAG, "Readback HTS inattendu: demande=0x%04X lecture=0x%04X", hts, after_hts);
    this->write_total_timing_register_(HTS_HIGH_REGISTER, before_hts);
    return false;
  }

  ESP_LOGW(TAG, "OV5640 HTS: 0x%04X -> 0x%04X", before_hts, after_hts);
  return true;
}

bool Ov5640TimingController::set_vts(uint16_t vts) {
  if (vts < min_total_timing()) {
    return false;
  }

  if (!this->capture_baseline_if_needed_()) {
    return false;
  }

  uint16_t before_hts = 0;
  uint16_t before_vts = 0;
  if (!this->read_total_timing_(&before_hts, &before_vts)) {
    return false;
  }

  if (!this->write_total_timing_register_(VTS_HIGH_REGISTER, vts)) {
    return false;
  }

  uint16_t after_hts = 0;
  uint16_t after_vts = 0;
  if (!this->read_total_timing_(&after_hts, &after_vts) || after_vts != vts) {
    ESP_LOGE(TAG, "Readback VTS inattendu: demande=0x%04X lecture=0x%04X", vts, after_vts);
    this->write_total_timing_register_(VTS_HIGH_REGISTER, before_vts);
    return false;
  }

  ESP_LOGW(TAG, "OV5640 VTS: 0x%04X -> 0x%04X", before_vts, after_vts);
  return true;
}

bool Ov5640TimingController::set_jpeg_mode(uint8_t mode) {
  if (mode != 2 && mode != 3) {
    ESP_LOGW(TAG, "Mode JPEG OV5640 non autorise pour ce test: %u", static_cast<unsigned>(mode));
    return false;
  }

  if (!this->capture_baseline_if_needed_()) {
    return false;
  }

  uint8_t before_mode = 0;
  uint8_t before_blanking = 0;
  if (!this->read_jpeg_output_timing_(&before_mode, &before_blanking)) {
    return false;
  }

  if (!this->write_byte_register_(JPEG_MODE_REGISTER, mode)) {
    return false;
  }

  uint8_t after_mode = 0;
  uint8_t after_blanking = 0;
  if (!this->read_jpeg_output_timing_(&after_mode, &after_blanking) || after_mode != mode) {
    ESP_LOGE(TAG, "Readback JPEG mode inattendu: demande=0x%02X lecture=0x%02X", mode, after_mode);
    this->write_byte_register_(JPEG_MODE_REGISTER, before_mode);
    return false;
  }

  ESP_LOGW(TAG, "OV5640 JPEG mode 0x4713: 0x%02X -> 0x%02X", before_mode, after_mode);
  return true;
}

bool Ov5640TimingController::set_href_blanking(uint8_t blanking) {
  if (!this->capture_baseline_if_needed_()) {
    return false;
  }

  uint8_t before_mode = 0;
  uint8_t before_blanking = 0;
  if (!this->read_jpeg_output_timing_(&before_mode, &before_blanking)) {
    return false;
  }

  if (!this->write_byte_register_(DVP_HREF_CTRL_REGISTER, blanking)) {
    return false;
  }

  uint8_t after_mode = 0;
  uint8_t after_blanking = 0;
  if (!this->read_jpeg_output_timing_(&after_mode, &after_blanking) || after_blanking != blanking) {
    ESP_LOGE(TAG, "Readback HREF blanking inattendu: demande=0x%02X lecture=0x%02X",
             blanking, after_blanking);
    this->write_byte_register_(DVP_HREF_CTRL_REGISTER, before_blanking);
    return false;
  }

  ESP_LOGW(TAG, "OV5640 HREF blanking 0x471F: 0x%02X -> 0x%02X", before_blanking, after_blanking);
  return true;
}

bool Ov5640TimingController::restore_baseline() {
  if (!this->baseline_available_) {
    ESP_LOGW(TAG, "Aucune reference de timing OV5640 a restaurer");
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->set_xclk == nullptr) {
    ESP_LOGE(TAG, "Capteur OV5640/XCLK indisponible pour restauration");
    return false;
  }

  uint16_t current_hts = 0;
  uint16_t current_vts = 0;
  uint8_t current_jpeg_mode = 0;
  uint8_t current_href_blanking = 0;
  const int current_xclk_mhz = sensor->xclk_freq_hz > 0 ? sensor->xclk_freq_hz / HZ_PER_MHZ : -1;
  if (!this->read_total_timing_(&current_hts, &current_vts) ||
      !this->read_jpeg_output_timing_(&current_jpeg_mode, &current_href_blanking)) {
    return false;
  }

  const bool writes_ok =
      this->write_total_timing_register_(HTS_HIGH_REGISTER, this->baseline_hts_) &&
      this->write_total_timing_register_(VTS_HIGH_REGISTER, this->baseline_vts_) &&
      this->write_byte_register_(JPEG_MODE_REGISTER, this->baseline_jpeg_mode_) &&
      this->write_byte_register_(DVP_HREF_CTRL_REGISTER, this->baseline_href_blanking_) &&
      sensor->set_xclk(sensor, XCLK_TIMER_ARGUMENT, this->baseline_xclk_mhz_) == 0;
  if (!writes_ok) {
    return false;
  }

  uint16_t restored_hts = 0;
  uint16_t restored_vts = 0;
  uint8_t restored_jpeg_mode = 0;
  uint8_t restored_href_blanking = 0;
  if (!this->read_total_timing_(&restored_hts, &restored_vts) ||
      !this->read_jpeg_output_timing_(&restored_jpeg_mode, &restored_href_blanking) ||
      restored_hts != this->baseline_hts_ || restored_vts != this->baseline_vts_ ||
      restored_jpeg_mode != this->baseline_jpeg_mode_ ||
      restored_href_blanking != this->baseline_href_blanking_ ||
      sensor->xclk_freq_hz != static_cast<int>(this->baseline_xclk_mhz_) * HZ_PER_MHZ) {
    ESP_LOGE(TAG, "Restauration de la reference OV5640 non confirmee");
    return false;
  }

  ESP_LOGW(TAG,
           "OV5640 reference restauree: XCLK %d->%u MHz HTS 0x%04X->0x%04X VTS 0x%04X->0x%04X JPEG 0x%02X->0x%02X HREF 0x%02X->0x%02X",
           current_xclk_mhz, static_cast<unsigned>(this->baseline_xclk_mhz_),
           current_hts, restored_hts, current_vts, restored_vts,
           current_jpeg_mode, restored_jpeg_mode, current_href_blanking, restored_href_blanking);

  this->baseline_available_ = false;
  this->baseline_xclk_mhz_ = 0;
  this->baseline_hts_ = 0;
  this->baseline_vts_ = 0;
  this->baseline_jpeg_mode_ = 0;
  this->baseline_href_blanking_ = 0;
  return true;
}

bool Ov5640TimingController::baseline_available() const { return this->baseline_available_; }

bool Ov5640TimingController::capture_baseline_if_needed_() {
  if (this->baseline_available_) {
    return true;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->xclk_freq_hz <= 0) {
    ESP_LOGE(TAG, "Impossible de memoriser XCLK de reference OV5640");
    return false;
  }

  const int xclk_mhz = sensor->xclk_freq_hz / HZ_PER_MHZ;
  if (xclk_mhz <= 0 || xclk_mhz > 255) {
    ESP_LOGE(TAG, "XCLK de reference invalide: %d Hz", sensor->xclk_freq_hz);
    return false;
  }

  uint16_t hts = 0;
  uint16_t vts = 0;
  uint8_t jpeg_mode = 0;
  uint8_t href_blanking = 0;
  if (!this->read_total_timing_(&hts, &vts) ||
      !this->read_jpeg_output_timing_(&jpeg_mode, &href_blanking)) {
    return false;
  }

  this->baseline_xclk_mhz_ = static_cast<uint8_t>(xclk_mhz);
  this->baseline_hts_ = hts;
  this->baseline_vts_ = vts;
  this->baseline_jpeg_mode_ = jpeg_mode;
  this->baseline_href_blanking_ = href_blanking;
  this->baseline_available_ = true;
  ESP_LOGI(TAG,
           "Reference OV5640 memorisee: XCLK=%u MHz HTS=0x%04X VTS=0x%04X JPEG=0x%02X HREF=0x%02X",
           static_cast<unsigned>(this->baseline_xclk_mhz_), hts, vts, jpeg_mode, href_blanking);
  return true;
}

bool Ov5640TimingController::read_total_timing_(uint16_t *hts, uint16_t *vts) const {
  if (hts == nullptr || vts == nullptr) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->get_reg == nullptr) {
    return false;
  }

  const int hts_high = sensor->get_reg(sensor, HTS_HIGH_REGISTER, 0xFF);
  const int hts_low = sensor->get_reg(sensor, HTS_HIGH_REGISTER + 1, 0xFF);
  const int vts_high = sensor->get_reg(sensor, VTS_HIGH_REGISTER, 0xFF);
  const int vts_low = sensor->get_reg(sensor, VTS_HIGH_REGISTER + 1, 0xFF);
  if (hts_high < 0 || hts_low < 0 || vts_high < 0 || vts_low < 0) {
    ESP_LOGE(TAG, "Lecture HTS/VTS impossible");
    return false;
  }

  *hts = static_cast<uint16_t>(((hts_high & 0xFF) << 8) | (hts_low & 0xFF));
  *vts = static_cast<uint16_t>(((vts_high & 0xFF) << 8) | (vts_low & 0xFF));
  return true;
}

bool Ov5640TimingController::read_jpeg_output_timing_(uint8_t *jpeg_mode,
                                                      uint8_t *href_blanking) const {
  if (jpeg_mode == nullptr || href_blanking == nullptr) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->get_reg == nullptr) {
    return false;
  }

  const int mode = sensor->get_reg(sensor, JPEG_MODE_REGISTER, 0xFF);
  const int blanking = sensor->get_reg(sensor, DVP_HREF_CTRL_REGISTER, 0xFF);
  if (mode < 0 || blanking < 0) {
    ESP_LOGE(TAG, "Lecture JPEG mode/HREF blanking impossible");
    return false;
  }

  *jpeg_mode = static_cast<uint8_t>(mode & 0xFF);
  *href_blanking = static_cast<uint8_t>(blanking & 0xFF);
  return true;
}

bool Ov5640TimingController::write_total_timing_register_(uint16_t high_register,
                                                          uint16_t value) const {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->set_reg == nullptr) {
    return false;
  }

  const int high_result = sensor->set_reg(sensor, high_register, 0xFF, (value >> 8) & 0xFF);
  const int low_result = sensor->set_reg(sensor, high_register + 1, 0xFF, value & 0xFF);
  if (high_result != 0 || low_result != 0) {
    ESP_LOGE(TAG, "Ecriture registre timing 0x%04X impossible: high=%d low=%d",
             high_register, high_result, low_result);
    return false;
  }
  return true;
}

bool Ov5640TimingController::write_byte_register_(uint16_t reg, uint8_t value) const {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->set_reg == nullptr) {
    return false;
  }

  const int result = sensor->set_reg(sensor, reg, 0xFF, value);
  if (result != 0) {
    ESP_LOGE(TAG, "Ecriture registre OV5640 0x%04X impossible: %d", reg, result);
    return false;
  }
  return true;
}

uint8_t Ov5640TimingController::min_xclk_mhz() { return MIN_XCLK_MHZ; }
uint8_t Ov5640TimingController::max_xclk_mhz() { return MAX_XCLK_MHZ; }
uint8_t Ov5640TimingController::min_pclk_divider() { return MIN_PCLK_DIVIDER; }
uint8_t Ov5640TimingController::max_pclk_divider() { return MAX_PCLK_DIVIDER; }
uint16_t Ov5640TimingController::min_total_timing() { return MIN_TOTAL_TIMING; }
uint16_t Ov5640TimingController::max_total_timing() { return MAX_TOTAL_TIMING; }
uint8_t Ov5640TimingController::min_href_blanking() { return MIN_HREF_BLANKING; }
uint8_t Ov5640TimingController::max_href_blanking() { return MAX_HREF_BLANKING; }

}  // namespace geometrie_camera_app
}  // namespace esphome
