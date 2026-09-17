#include "full_calibration_controller.h"

#include <algorithm>
#include <cmath>

#include "camera_resolution_controller.h"
#include "continuous_measurement_controller.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "geometry_measurement.h"
#include "jpeg_diagnostic.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_manager.h"
#include "target_detection_service.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "full_calibration";
constexpr uint16_t CALIBRATION_WIDTH = 2560;
constexpr uint16_t CALIBRATION_HEIGHT = 1920;
constexpr uint32_t CAPTURE_TIMEOUT_MS = 15000;
}

FullCalibrationController::FullCalibrationController(
    CameraResolutionController *resolution_controller,
    JpegDiagnostic *jpeg_source,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager,
    ContinuousMeasurementController *continuous_controller)
    : resolution_controller_(resolution_controller),
      jpeg_source_(jpeg_source),
      filtered_source_(filtered_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      continuous_controller_(continuous_controller),
      state_(FullCalibrationState::IDLE),
      last_error_(),
      known_distance_mm_(0.0f),
      target_size_mm_(0.0f),
      requested_samples_(DEFAULT_SAMPLE_COUNT),
      valid_samples_(0),
      attempts_(0),
      max_attempts_(DEFAULT_SAMPLE_COUNT * 2),
      capture_count_before_request_(0),
      capture_started_ms_(0),
      samples_{},
      result_calibration_(),
      mean_fx_px_(0.0f),
      mean_fy_px_(0.0f),
      stddev_fx_px_(0.0f),
      stddev_fy_px_(0.0f),
      previous_target_size_mm_(0.0f),
      previous_calibration_(),
      previous_config_saved_(false) {}

bool FullCalibrationController::start(float known_distance_mm,
                                      float target_size_mm,
                                      uint8_t sample_count,
                                      bool force) {
  if (this->running()) {
    this->last_error_ = "calibration_already_running";
    return false;
  }
  if (this->resolution_controller_ == nullptr ||
      this->jpeg_source_ == nullptr ||
      this->filtered_source_ == nullptr ||
      this->detection_service_ == nullptr ||
      this->measurement_manager_ == nullptr) {
    this->last_error_ = "calibration_dependencies_unavailable";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (!std::isfinite(known_distance_mm) || known_distance_mm < 50.0f ||
      known_distance_mm > 20000.0f) {
    this->last_error_ = "distance_mm_out_of_range";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (!std::isfinite(target_size_mm) || target_size_mm < 1.0f ||
      target_size_mm > 1000.0f) {
    this->last_error_ = "target_size_mm_out_of_range";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (sample_count < 3 || sample_count > MAX_SAMPLE_COUNT) {
    this->last_error_ = "sample_count_out_of_range";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  if (engine.has_calibration() && !force) {
    this->last_error_ = "calibration_locked";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (this->continuous_controller_ != nullptr && this->continuous_controller_->running()) {
    this->continuous_controller_->stop();
  }

  this->previous_target_size_mm_ = engine.target_size_mm();
  this->previous_calibration_ = engine.calibration();
  this->previous_config_saved_ = true;

  this->reset_run_();
  this->known_distance_mm_ = known_distance_mm;
  this->target_size_mm_ = target_size_mm;
  this->requested_samples_ = sample_count;
  this->max_attempts_ = std::min<uint8_t>(
      static_cast<uint8_t>(MAX_SAMPLE_COUNT * 2),
      static_cast<uint8_t>(std::max<int>(sample_count + 3, sample_count * 2)));

  if (!engine.set_target_size_mm(target_size_mm)) {
    this->fail_("target_size_apply_failed");
    return false;
  }

  ESP_LOGI(TAG,
           "Calibration full demandee: distance=%.2f mm cible=%.2f mm echantillons=%u",
           known_distance_mm, target_size_mm,
           static_cast<unsigned>(sample_count));

  // Si une acquisition etait encore en vol au moment ou le continu a ete
  // arrete, la laisser se terminer proprement avant de changer de framesize.
  if (this->jpeg_source_->capture_pending()) {
    this->capture_started_ms_ = millis();
    this->state_ = FullCalibrationState::WAIT_IDLE;
    return true;
  }

  if (!this->begin_full_resolution_()) {
    this->fail_("full_resolution_start_failed");
    return false;
  }
  return true;
}

void FullCalibrationController::loop() {
  switch (this->state_) {
    case FullCalibrationState::WAIT_IDLE: {
      const uint32_t now = millis();
      if (this->jpeg_source_->capture_pending()) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("camera_idle_timeout");
        }
        return;
      }

      if (!this->begin_full_resolution_()) {
        this->fail_("full_resolution_start_failed");
      }
      return;
    }

    case FullCalibrationState::WAIT_CAPTURE: {
      const uint32_t now = millis();
      if (this->jpeg_source_->capture_pending()) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("capture_timeout");
        }
        return;
      }

      if (!this->jpeg_source_->ready() ||
          this->jpeg_source_->capture_count() <= this->capture_count_before_request_) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("fresh_capture_not_received");
        }
        return;
      }

      if (this->jpeg_source_->width() != CALIBRATION_WIDTH ||
          this->jpeg_source_->height() != CALIBRATION_HEIGHT) {
        this->fail_("unexpected_full_resolution");
        return;
      }

      this->state_ = FullCalibrationState::FILTER;
      return;
    }

    case FullCalibrationState::FILTER:
      // Camera nominalement N/B : aucune correction couleur pendant la
      // calibration 5 MP, seulement le decodage JPEG -> niveaux de gris.
      if (!this->filtered_source_->process(false)) {
        this->fail_("full_resolution_filter_failed");
        return;
      }
      this->state_ = FullCalibrationState::DETECT;
      return;

    case FullCalibrationState::DETECT: {
      this->attempts_++;

      if (!this->detection_service_->detect()) {
        this->fail_("full_resolution_detection_failed");
        return;
      }

      if (this->detection_service_->target_found()) {
        CameraCalibration sample;
        if (this->derive_current_sample_(sample)) {
          this->samples_[this->valid_samples_] = sample;
          this->valid_samples_++;
          ESP_LOGI(TAG,
                   "Calibration full: echantillon %u/%u fx=%.3f fy=%.3f qualite=%.3f",
                   static_cast<unsigned>(this->valid_samples_),
                   static_cast<unsigned>(this->requested_samples_),
                   sample.fx_px, sample.fy_px,
                   this->detection_service_->last_observation().quality);
        } else {
          ESP_LOGW(TAG, "Calibration full: echantillon detecte mais non exploitable");
        }
      } else {
        ESP_LOGW(TAG,
                 "Calibration full: cible non validee tentative %u/%u",
                 static_cast<unsigned>(this->attempts_),
                 static_cast<unsigned>(this->max_attempts_));
      }

      if (this->valid_samples_ >= this->requested_samples_) {
        this->finish_success_();
        return;
      }

      if (this->attempts_ >= this->max_attempts_) {
        this->fail_("not_enough_valid_samples");
        return;
      }

      if (!this->request_next_capture_()) {
        this->fail_("capture_request_failed");
      }
      return;
    }

    case FullCalibrationState::IDLE:
    case FullCalibrationState::COMPLETE:
    case FullCalibrationState::ERROR:
    default:
      return;
  }
}

void FullCalibrationController::cancel() {
  if (!this->running()) {
    return;
  }
  this->restore_previous_measurement_config_();
  this->restore_nominal_camera_();
  this->last_error_ = "cancelled";
  this->state_ = FullCalibrationState::IDLE;
}

bool FullCalibrationController::running() const {
  return this->state_ == FullCalibrationState::WAIT_IDLE ||
         this->state_ == FullCalibrationState::WAIT_CAPTURE ||
         this->state_ == FullCalibrationState::FILTER ||
         this->state_ == FullCalibrationState::DETECT;
}

FullCalibrationState FullCalibrationController::state() const { return this->state_; }

const char *FullCalibrationController::state_text() const {
  switch (this->state_) {
    case FullCalibrationState::IDLE: return "idle";
    case FullCalibrationState::WAIT_IDLE: return "wait_idle";
    case FullCalibrationState::WAIT_CAPTURE: return "wait_capture";
    case FullCalibrationState::FILTER: return "filter";
    case FullCalibrationState::DETECT: return "detect";
    case FullCalibrationState::COMPLETE: return "complete";
    case FullCalibrationState::ERROR: return "error";
    default: return "unknown";
  }
}

const std::string &FullCalibrationController::last_error() const { return this->last_error_; }
uint8_t FullCalibrationController::requested_samples() const { return this->requested_samples_; }
uint8_t FullCalibrationController::valid_samples() const { return this->valid_samples_; }
uint8_t FullCalibrationController::attempts() const { return this->attempts_; }
uint8_t FullCalibrationController::max_attempts() const { return this->max_attempts_; }
float FullCalibrationController::known_distance_mm() const { return this->known_distance_mm_; }
float FullCalibrationController::target_size_mm() const { return this->target_size_mm_; }
float FullCalibrationController::mean_fx_px() const { return this->mean_fx_px_; }
float FullCalibrationController::mean_fy_px() const { return this->mean_fy_px_; }
float FullCalibrationController::stddev_fx_px() const { return this->stddev_fx_px_; }
float FullCalibrationController::stddev_fy_px() const { return this->stddev_fy_px_; }
const CameraCalibration &FullCalibrationController::result_calibration() const {
  return this->result_calibration_;
}

bool FullCalibrationController::begin_full_resolution_() {
  // Liberer les buffers de traitement 800x600 avant l'allocation 5 MP pour
  // maximiser la PSRAM disponible pendant cette operation exceptionnelle.
  this->filtered_source_->release_buffers();
  this->jpeg_source_->release_buffer();

  if (!this->resolution_controller_->apply("2560x1920")) {
    return false;
  }

  this->detection_service_->reset_tracking();
  ESP_LOGI(TAG, "Calibration full: resolution 2560x1920 active");

  return this->request_next_capture_();
}

bool FullCalibrationController::request_next_capture_() {
  if (this->jpeg_source_->capture_pending()) {
    return false;
  }

  this->capture_count_before_request_ = this->jpeg_source_->capture_count();
  this->capture_started_ms_ = millis();
  if (!this->jpeg_source_->request_capture()) {
    return false;
  }

  this->state_ = FullCalibrationState::WAIT_CAPTURE;
  return true;
}

bool FullCalibrationController::derive_current_sample_(CameraCalibration &sample) {
  if (this->filtered_source_->width() != CALIBRATION_WIDTH ||
      this->filtered_source_->height() != CALIBRATION_HEIGHT) {
    return false;
  }

  return this->measurement_manager_->measurement_engine().derive_calibration_from_known_distance(
      this->detection_service_->last_observation(),
      CALIBRATION_WIDTH,
      CALIBRATION_HEIGHT,
      this->known_distance_mm_,
      sample);
}

void FullCalibrationController::finish_success_() {
  if (this->valid_samples_ == 0) {
    this->fail_("no_valid_samples");
    return;
  }

  double sum_fx = 0.0;
  double sum_fy = 0.0;
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    sum_fx += this->samples_[i].fx_px;
    sum_fy += this->samples_[i].fy_px;
  }

  this->mean_fx_px_ = static_cast<float>(sum_fx / this->valid_samples_);
  this->mean_fy_px_ = static_cast<float>(sum_fy / this->valid_samples_);

  double variance_fx = 0.0;
  double variance_fy = 0.0;
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    const double dx = this->samples_[i].fx_px - this->mean_fx_px_;
    const double dy = this->samples_[i].fy_px - this->mean_fy_px_;
    variance_fx += dx * dx;
    variance_fy += dy * dy;
  }
  variance_fx /= this->valid_samples_;
  variance_fy /= this->valid_samples_;
  this->stddev_fx_px_ = static_cast<float>(std::sqrt(variance_fx));
  this->stddev_fy_px_ = static_cast<float>(std::sqrt(variance_fy));

  this->result_calibration_ = this->samples_[0];
  this->result_calibration_.fx_px = this->mean_fx_px_;
  this->result_calibration_.fy_px = this->mean_fy_px_;
  this->result_calibration_.cx_px = (CALIBRATION_WIDTH - 1.0f) * 0.5f;
  this->result_calibration_.cy_px = (CALIBRATION_HEIGHT - 1.0f) * 0.5f;
  this->result_calibration_.reference_width_px = CALIBRATION_WIDTH;
  this->result_calibration_.reference_height_px = CALIBRATION_HEIGHT;

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  engine.set_calibration(this->result_calibration_);
  this->measurement_manager_->reset();

  this->previous_config_saved_ = false;
  this->restore_nominal_camera_();
  this->last_error_.clear();
  this->state_ = FullCalibrationState::COMPLETE;

  ESP_LOGI(TAG,
           "Calibration full terminee: n=%u fx=%.3f +/- %.3f fy=%.3f +/- %.3f",
           static_cast<unsigned>(this->valid_samples_),
           this->mean_fx_px_, this->stddev_fx_px_,
           this->mean_fy_px_, this->stddev_fy_px_);
}

void FullCalibrationController::fail_(const char *error) {
  this->last_error_ = error != nullptr ? error : "calibration_failed";
  ESP_LOGE(TAG, "Calibration full en echec: %s", this->last_error_.c_str());
  this->restore_previous_measurement_config_();
  this->restore_nominal_camera_();
  this->state_ = FullCalibrationState::ERROR;
}

void FullCalibrationController::restore_nominal_camera_() {
  if (this->filtered_source_ != nullptr) {
    this->filtered_source_->release_buffers();
  }
  if (this->jpeg_source_ != nullptr) {
    this->jpeg_source_->release_buffer();
  }
  if (this->resolution_controller_ != nullptr &&
      !this->resolution_controller_->apply("800x600")) {
    ESP_LOGE(TAG, "Impossible de restaurer la resolution nominale 800x600");
  }
  if (this->detection_service_ != nullptr) {
    this->detection_service_->reset_tracking();
  }
}

void FullCalibrationController::restore_previous_measurement_config_() {
  if (!this->previous_config_saved_ || this->measurement_manager_ == nullptr) {
    return;
  }

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  engine.set_target_size_mm(this->previous_target_size_mm_);
  engine.set_calibration(this->previous_calibration_);
  this->measurement_manager_->reset();
  this->previous_config_saved_ = false;
}

void FullCalibrationController::reset_run_() {
  this->last_error_.clear();
  this->valid_samples_ = 0;
  this->attempts_ = 0;
  this->capture_count_before_request_ = 0;
  this->capture_started_ms_ = 0;
  this->result_calibration_ = CameraCalibration();
  this->mean_fx_px_ = 0.0f;
  this->mean_fy_px_ = 0.0f;
  this->stddev_fx_px_ = 0.0f;
  this->stddev_fy_px_ = 0.0f;
  for (auto &sample : this->samples_) {
    sample = CameraCalibration();
  }
  this->state_ = FullCalibrationState::IDLE;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
