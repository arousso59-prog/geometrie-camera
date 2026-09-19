#include "geometry_calibration_storage.h"

#include <cmath>

#include "esphome/core/log.h"
#include "target_board_model.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "geometry_cal_storage";

bool calibration_is_valid(const CameraCalibration &c) {
  return std::isfinite(c.fx_px) && std::isfinite(c.fy_px) &&
         std::isfinite(c.cx_px) && std::isfinite(c.cy_px) &&
         c.fx_px > 100.0f && c.fy_px > 100.0f &&
         c.reference_width_px == 2560 &&
         c.reference_height_px == 1920;
}
}  // namespace

GeometryCalibrationStorage::GeometryCalibrationStorage()
    : preference_(),
      available_(false),
      stored_valid_(false),
      stored_stddev_fx_px_(0.0f),
      stored_stddev_fy_px_(0.0f) {}

void GeometryCalibrationStorage::setup() {
  if (global_preferences == nullptr) {
    ESP_LOGW(TAG, "Preferences ESPHome indisponibles");
    return;
  }

  this->preference_ =
      global_preferences->make_preference<Record>(PREFERENCE_KEY, true);
  this->available_ = true;
}

bool GeometryCalibrationStorage::load(CameraCalibration &calibration) {
  this->stored_valid_ = false;
  if (!this->available_) return false;

  Record record{};
  if (!this->preference_.load(&record)) {
    ESP_LOGI(TAG, "Aucune calibration geometrique R1 persistante");
    return false;
  }

  if (record.magic != MAGIC ||
      record.version != VERSION ||
      record.target_model != TARGET_MODEL_R1 ||
      !calibration_is_valid(record.calibration)) {
    ESP_LOGW(TAG, "Calibration geometrique persistante invalide ou obsolete");
    return false;
  }

  calibration = record.calibration;
  this->stored_stddev_fx_px_ = record.stddev_fx_px;
  this->stored_stddev_fy_px_ = record.stddev_fy_px;
  this->stored_valid_ = true;

  ESP_LOGI(TAG,
           "Calibration R1 chargee depuis NVS: fx=%.3f fy=%.3f sigma=(%.3f,%.3f)",
           calibration.fx_px, calibration.fy_px,
           this->stored_stddev_fx_px_, this->stored_stddev_fy_px_);
  return true;
}

bool GeometryCalibrationStorage::save(
    const CameraCalibration &calibration,
    float stddev_fx_px, float stddev_fy_px) {
  if (!this->available_ || !calibration_is_valid(calibration)) return false;

  Record record{};
  record.magic = MAGIC;
  record.version = VERSION;
  record.target_model = TARGET_MODEL_R1;
  record.calibration = calibration;
  record.stddev_fx_px = stddev_fx_px;
  record.stddev_fy_px = stddev_fy_px;

  if (!this->preference_.save(&record)) {
    ESP_LOGE(TAG, "Echec sauvegarde calibration R1 dans NVS");
    return false;
  }
  if (global_preferences != nullptr) {
    global_preferences->sync();
  }

  this->stored_valid_ = true;
  this->stored_stddev_fx_px_ = stddev_fx_px;
  this->stored_stddev_fy_px_ = stddev_fy_px;
  ESP_LOGI(TAG, "Calibration geometrique R1 sauvegardee en NVS");
  return true;
}

bool GeometryCalibrationStorage::clear() {
  if (!this->available_) return false;
  Record empty{};
  const bool ok = this->preference_.save(&empty);
  if (ok && global_preferences != nullptr) global_preferences->sync();
  if (ok) {
    this->stored_valid_ = false;
    this->stored_stddev_fx_px_ = 0.0f;
    this->stored_stddev_fy_px_ = 0.0f;
  }
  return ok;
}

bool GeometryCalibrationStorage::available() const { return this->available_; }
bool GeometryCalibrationStorage::stored_valid() const { return this->stored_valid_; }
float GeometryCalibrationStorage::stored_stddev_fx_px() const {
  return this->stored_stddev_fx_px_;
}
float GeometryCalibrationStorage::stored_stddev_fy_px() const {
  return this->stored_stddev_fy_px_;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
