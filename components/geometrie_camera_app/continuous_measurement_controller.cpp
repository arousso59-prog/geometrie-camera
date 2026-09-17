#include "continuous_measurement_controller.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "image_sharpness_evaluator.h"
#include "jpeg_diagnostic.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_manager.h"
#include "target_detection_service.h"
#include "target_tracking_controller.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "continuous_measurement";
constexpr uint32_t MIN_INTERVAL_MS = 200;
constexpr uint32_t MAX_INTERVAL_MS = 10000;
constexpr uint32_t CAPTURE_TIMEOUT_MS = 10000;
constexpr uint8_t MAX_BLUR_RETRIES = 2;
constexpr uint32_t SHARPNESS_MIN_PERCENT_OF_REFERENCE = 45;
constexpr uint16_t SHARPNESS_ROI_MIN_SIZE_PX = 48;
constexpr float SHARPNESS_ROI_TARGET_SCALE = 2.5f;
constexpr uint32_t SHARPNESS_REFERENCE_MAX_STEP_PERCENT = 15;
constexpr uint32_t SHARPNESS_REFERENCE_FILTER_DENOMINATOR = 8;
}

ContinuousMeasurementController::ContinuousMeasurementController(
    JpegDiagnostic *jpeg_source,
    ImageSharpnessEvaluator *sharpness_evaluator,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager,
    TargetTrackingController *tracking_controller)
    : jpeg_source_(jpeg_source),
      sharpness_evaluator_(sharpness_evaluator),
      filtered_source_(filtered_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      tracking_controller_(tracking_controller),
      running_(false),
      interval_ms_(1000),
      sharpness_enabled_(false),
      artifact_correction_enabled_(false),
      state_(ContinuousMeasurementState::STOPPED),
      cycle_count_(0),
      target_found_count_(0),
      valid_measurement_count_(0),
      cycle_started_ms_(0),
      last_cycle_completed_ms_(0),
      last_cycle_ms_(0),
      capture_count_before_request_(0),
      last_cycle_target_found_(false),
      last_cycle_measurement_valid_(false),
      current_cycle_viewport_mode_("search"),
      current_cycle_viewport_x_(0),
      current_cycle_viewport_y_(0),
      current_cycle_viewport_width_(2560),
      current_cycle_viewport_height_(1920),
      last_cycle_viewport_mode_("search"),
      last_cycle_viewport_x_(0),
      last_cycle_viewport_y_(0),
      last_cycle_viewport_width_(2560),
      last_cycle_viewport_height_(1920),
      last_error_(),
      current_capture_ms_(0),
      current_sharpness_ms_(0),
      current_filter_ms_(0),
      current_filter_decode_ms_(0),
      current_filter_correction_ms_(0),
      current_detect_ms_(0),
      current_compute_ms_(0),
      last_capture_ms_(0),
      last_sharpness_ms_(0),
      last_filter_ms_(0),
      last_filter_decode_ms_(0),
      last_filter_correction_ms_(0),
      last_detect_ms_(0),
      last_compute_ms_(0),
      last_sharpness_score_x100_(0),
      sharpness_reference_score_x100_(0),
      last_sharpness_ok_(false),
      last_capture_retry_count_(0),
      blur_retry_count_(0),
      sharpness_roi_valid_(false),
      sharpness_roi_x_(0),
      sharpness_roi_y_(0),
      sharpness_roi_width_(0),
      sharpness_roi_height_(0) {}

bool ContinuousMeasurementController::start(uint32_t interval_ms) {
  if (!this->set_interval_ms(interval_ms)) {
    this->running_ = false;
    this->state_ = ContinuousMeasurementState::ERROR;
    this->last_error_ = "interval_ms_out_of_range";
    return false;
  }

  if (this->jpeg_source_ == nullptr || this->sharpness_evaluator_ == nullptr ||
      this->filtered_source_ == nullptr || this->detection_service_ == nullptr ||
      this->measurement_manager_ == nullptr) {
    this->running_ = false;
    this->state_ = ContinuousMeasurementState::ERROR;
    this->last_error_ = "continuous_measurement_unavailable";
    return false;
  }

  if (!this->measurement_manager_->measurement_engine().has_calibration()) {
    this->running_ = false;
    this->state_ = ContinuousMeasurementState::ERROR;
    this->last_error_ = "calibration_required";
    return false;
  }

  if (this->tracking_controller_ != nullptr && !this->tracking_controller_->start()) {
    this->running_ = false;
    this->state_ = ContinuousMeasurementState::ERROR;
    this->last_error_ = this->tracking_controller_->last_error().empty()
                            ? "tracking_start_failed"
                            : this->tracking_controller_->last_error();
    return false;
  }

  this->running_ = true;
  this->state_ = ContinuousMeasurementState::REQUEST_CAPTURE;
  this->cycle_count_ = 0;
  this->target_found_count_ = 0;
  this->valid_measurement_count_ = 0;
  this->cycle_started_ms_ = 0;
  this->last_cycle_completed_ms_ = 0;
  this->last_cycle_ms_ = 0;
  this->capture_count_before_request_ = 0;
  this->last_cycle_target_found_ = false;
  this->last_cycle_measurement_valid_ = false;
  this->current_cycle_viewport_mode_ = "search";
  this->current_cycle_viewport_x_ = 0;
  this->current_cycle_viewport_y_ = 0;
  this->current_cycle_viewport_width_ = 2560;
  this->current_cycle_viewport_height_ = 1920;
  this->last_cycle_viewport_mode_ = "search";
  this->last_cycle_viewport_x_ = 0;
  this->last_cycle_viewport_y_ = 0;
  this->last_cycle_viewport_width_ = 2560;
  this->last_cycle_viewport_height_ = 1920;

  this->current_capture_ms_ = 0;
  this->current_sharpness_ms_ = 0;
  this->current_filter_ms_ = 0;
  this->current_filter_decode_ms_ = 0;
  this->current_filter_correction_ms_ = 0;
  this->current_detect_ms_ = 0;
  this->current_compute_ms_ = 0;
  this->last_capture_ms_ = 0;
  this->last_sharpness_ms_ = 0;
  this->last_filter_ms_ = 0;
  this->last_filter_decode_ms_ = 0;
  this->last_filter_correction_ms_ = 0;
  this->last_detect_ms_ = 0;
  this->last_compute_ms_ = 0;

  this->last_sharpness_score_x100_ = 0;
  this->sharpness_reference_score_x100_ = 0;
  this->last_sharpness_ok_ = !this->sharpness_enabled_;
  this->last_capture_retry_count_ = 0;
  this->blur_retry_count_ = 0;
  this->sharpness_roi_valid_ = false;
  this->sharpness_roi_x_ = 0;
  this->sharpness_roi_y_ = 0;
  this->sharpness_roi_width_ = 0;
  this->sharpness_roi_height_ = 0;
  this->last_error_.clear();

  if (this->tracking_controller_ != nullptr && this->tracking_controller_->enabled()) {
    this->reset_local_tracking_after_viewport_change_();
  } else if (this->sharpness_enabled_ && this->detection_service_->ready() &&
             this->detection_service_->target_found()) {
    this->update_sharpness_roi_from_target_();
  }

  ESP_LOGI(TAG,
           "Mesure continue demarree, intervalle=%u ms, tracking=%s, nettete=%s, correction_artifacts=%s",
           static_cast<unsigned>(this->interval_ms_),
           this->tracking_controller_ != nullptr && this->tracking_controller_->enabled() ? "AUTO" : "OFF",
           this->sharpness_enabled_ ? "ON" : "OFF",
           this->artifact_correction_enabled_ ? "ON" : "OFF");
  return true;
}

void ContinuousMeasurementController::stop() {
  if (this->running_) {
    ESP_LOGI(TAG, "Mesure continue arretee apres %u cycles", static_cast<unsigned>(this->cycle_count_));
  }
  this->running_ = false;
  this->state_ = ContinuousMeasurementState::STOPPED;
  if (this->tracking_controller_ != nullptr) {
    this->tracking_controller_->stop();
    this->reset_local_tracking_after_viewport_change_();
  }
}

void ContinuousMeasurementController::loop() {
  if (!this->running_) return;

  if (this->measurement_manager_ == nullptr ||
      !this->measurement_manager_->measurement_engine().has_calibration()) {
    this->stop_with_error_("calibration_lost");
    return;
  }

  const uint32_t now = millis();

  switch (this->state_) {
    case ContinuousMeasurementState::REQUEST_CAPTURE:
      this->begin_cycle_();
      if (!this->request_capture_()) {
        this->fail_cycle_("capture_request_failed");
        return;
      }
      this->state_ = ContinuousMeasurementState::WAIT_CAPTURE;
      return;

    case ContinuousMeasurementState::WAIT_CAPTURE:
      if (!this->jpeg_source_->capture_pending() &&
          this->jpeg_source_->capture_count() > this->capture_count_before_request_ &&
          this->jpeg_source_->ready()) {
        this->current_capture_ms_ += this->jpeg_source_->total_cycle_ms();
        this->state_ = ContinuousMeasurementState::SHARPNESS;
        return;
      }
      if (this->jpeg_source_->request_started_ms() != 0 &&
          now - this->jpeg_source_->request_started_ms() > CAPTURE_TIMEOUT_MS) {
        this->fail_cycle_("capture_timeout");
      }
      return;

    case ContinuousMeasurementState::SHARPNESS: {
      if (!this->sharpness_enabled_) {
        this->last_sharpness_ok_ = true;
        this->state_ = ContinuousMeasurementState::FILTER;
        return;
      }

      if (!this->sharpness_roi_valid_) {
        this->last_sharpness_ok_ = true;
        this->state_ = ContinuousMeasurementState::FILTER;
        return;
      }

      if (!this->sharpness_evaluator_->evaluate_region(
              this->sharpness_roi_x_, this->sharpness_roi_y_,
              this->sharpness_roi_width_, this->sharpness_roi_height_)) {
        this->last_sharpness_ok_ = true;
        ESP_LOGW(TAG, "Controle nettete ROI indisponible; pipeline poursuivi");
        this->state_ = ContinuousMeasurementState::FILTER;
        return;
      }

      this->current_sharpness_ms_ += this->sharpness_evaluator_->evaluation_ms();
      this->last_sharpness_score_x100_ = this->sharpness_evaluator_->score_x100();
      const bool too_blurry = this->sharpness_is_too_low_(this->last_sharpness_score_x100_);

      if (too_blurry && this->last_capture_retry_count_ < MAX_BLUR_RETRIES) {
        this->last_sharpness_ok_ = false;
        this->last_capture_retry_count_++;
        this->blur_retry_count_++;
        ESP_LOGI(TAG, "Image floue dans ROI cible: score_edges_x100=%u reference=%u, recapture %u/%u",
                 static_cast<unsigned>(this->last_sharpness_score_x100_),
                 static_cast<unsigned>(this->sharpness_reference_score_x100_),
                 static_cast<unsigned>(this->last_capture_retry_count_),
                 static_cast<unsigned>(MAX_BLUR_RETRIES));
        if (!this->request_capture_()) {
          this->fail_cycle_("blur_recapture_failed");
          return;
        }
        this->state_ = ContinuousMeasurementState::WAIT_CAPTURE;
        return;
      }

      this->last_sharpness_ok_ = !too_blurry;
      if (too_blurry) {
        ESP_LOGW(TAG, "ROI cible encore floue apres %u recaptures; pipeline poursuivi",
                 static_cast<unsigned>(this->last_capture_retry_count_));
      }
      this->state_ = ContinuousMeasurementState::FILTER;
      return;
    }

    case ContinuousMeasurementState::FILTER:
      if (!this->filtered_source_->process(this->artifact_correction_enabled_)) {
        this->fail_cycle_("filter_failed");
        return;
      }
      this->current_filter_ms_ = this->filtered_source_->total_ms();
      this->current_filter_decode_ms_ = this->filtered_source_->decode_ms();
      this->current_filter_correction_ms_ = this->filtered_source_->correction_ms();
      this->state_ = ContinuousMeasurementState::DETECT;
      return;

    case ContinuousMeasurementState::DETECT: {
      if (!this->detection_service_->detect()) {
        this->fail_cycle_("detection_failed");
        return;
      }
      this->current_detect_ms_ = this->detection_service_->detection_ms();
      if (!this->detection_service_->target_found()) {
        if (this->tracking_controller_ != nullptr && this->tracking_controller_->enabled()) {
          const TrackingUpdateResult tracking_result =
              this->tracking_controller_->update_after_detection(
                  false, this->detection_service_->last_observation());
          if (tracking_result == TrackingUpdateResult::ERROR) {
            this->fail_cycle_("tracking_update_failed");
            return;
          }
          if (tracking_result == TrackingUpdateResult::VIEWPORT_CHANGED) {
            this->reset_local_tracking_after_viewport_change_();
          }
        }
        this->finish_cycle_(false, false);
        return;
      }

      if (this->sharpness_enabled_ && this->last_sharpness_score_x100_ > 0 && this->last_sharpness_ok_) {
        this->update_sharpness_reference_(this->last_sharpness_score_x100_);
      }
      if (this->sharpness_enabled_) {
        this->update_sharpness_roi_from_target_();
      }
      this->state_ = ContinuousMeasurementState::COMPUTE;
      return;
    }

    case ContinuousMeasurementState::COMPUTE: {
      const uint32_t compute_started_ms = millis();
      TargetObservation measurement_observation = this->detection_service_->last_observation();
      uint16_t measurement_width = this->filtered_source_->width();
      uint16_t measurement_height = this->filtered_source_->height();

      if (this->tracking_controller_ != nullptr && this->tracking_controller_->enabled()) {
        measurement_observation = this->tracking_controller_->to_reference(measurement_observation);
        measurement_width = this->tracking_controller_->reference_width();
        measurement_height = this->tracking_controller_->reference_height();
      }

      const bool measured = this->measurement_manager_->process(
          measurement_observation, measurement_width, measurement_height, millis());
      this->current_compute_ms_ = millis() - compute_started_ms;
      if (!measured) {
        this->fail_cycle_("measurement_failed");
        return;
      }

      if (this->tracking_controller_ != nullptr && this->tracking_controller_->enabled()) {
        const TrackingUpdateResult tracking_result = this->tracking_controller_->update_after_detection(
            true, this->detection_service_->last_observation());
        if (tracking_result == TrackingUpdateResult::ERROR) {
          this->fail_cycle_("tracking_update_failed");
          return;
        }
        if (tracking_result == TrackingUpdateResult::VIEWPORT_CHANGED) {
          this->reset_local_tracking_after_viewport_change_();
        }
      }

      this->finish_cycle_(true, true);
      return;
    }

    case ContinuousMeasurementState::WAIT_INTERVAL:
      if (now - this->cycle_started_ms_ >= this->interval_ms_) {
        this->state_ = ContinuousMeasurementState::REQUEST_CAPTURE;
      }
      return;

    case ContinuousMeasurementState::STOPPED:
    case ContinuousMeasurementState::ERROR:
    default:
      return;
  }
}

bool ContinuousMeasurementController::set_interval_ms(uint32_t interval_ms) {
  if (interval_ms < MIN_INTERVAL_MS || interval_ms > MAX_INTERVAL_MS) return false;
  this->interval_ms_ = interval_ms;
  return true;
}

bool ContinuousMeasurementController::set_pipeline_options(bool sharpness_enabled,
                                                           bool artifact_correction_enabled) {
  if (this->running_) {
    this->last_error_ = "continuous_active";
    return false;
  }
  this->sharpness_enabled_ = sharpness_enabled;
  this->artifact_correction_enabled_ = artifact_correction_enabled;
  this->last_error_.clear();
  return true;
}

bool ContinuousMeasurementController::running() const { return this->running_; }
uint32_t ContinuousMeasurementController::interval_ms() const { return this->interval_ms_; }
bool ContinuousMeasurementController::sharpness_enabled() const { return this->sharpness_enabled_; }
bool ContinuousMeasurementController::artifact_correction_enabled() const {
  return this->artifact_correction_enabled_;
}
ContinuousMeasurementState ContinuousMeasurementController::state() const { return this->state_; }

const char *ContinuousMeasurementController::state_text() const {
  switch (this->state_) {
    case ContinuousMeasurementState::STOPPED: return "stopped";
    case ContinuousMeasurementState::REQUEST_CAPTURE: return "request_capture";
    case ContinuousMeasurementState::WAIT_CAPTURE: return "wait_capture";
    case ContinuousMeasurementState::SHARPNESS: return "sharpness";
    case ContinuousMeasurementState::FILTER: return "filter";
    case ContinuousMeasurementState::DETECT: return "detect";
    case ContinuousMeasurementState::COMPUTE: return "compute";
    case ContinuousMeasurementState::WAIT_INTERVAL: return "wait_interval";
    case ContinuousMeasurementState::ERROR: return "error";
    default: return "unknown";
  }
}

uint32_t ContinuousMeasurementController::cycle_count() const { return this->cycle_count_; }
uint32_t ContinuousMeasurementController::target_found_count() const { return this->target_found_count_; }
uint32_t ContinuousMeasurementController::valid_measurement_count() const { return this->valid_measurement_count_; }
uint32_t ContinuousMeasurementController::last_cycle_ms() const { return this->last_cycle_ms_; }
bool ContinuousMeasurementController::last_cycle_target_found() const { return this->last_cycle_target_found_; }
bool ContinuousMeasurementController::last_cycle_measurement_valid() const { return this->last_cycle_measurement_valid_; }
const std::string &ContinuousMeasurementController::last_cycle_viewport_mode() const {
  return this->last_cycle_viewport_mode_;
}
uint16_t ContinuousMeasurementController::last_cycle_viewport_x() const {
  return this->last_cycle_viewport_x_;
}
uint16_t ContinuousMeasurementController::last_cycle_viewport_y() const {
  return this->last_cycle_viewport_y_;
}
uint16_t ContinuousMeasurementController::last_cycle_viewport_width() const {
  return this->last_cycle_viewport_width_;
}
uint16_t ContinuousMeasurementController::last_cycle_viewport_height() const {
  return this->last_cycle_viewport_height_;
}
const std::string &ContinuousMeasurementController::last_error() const { return this->last_error_; }
uint32_t ContinuousMeasurementController::last_capture_ms() const { return this->last_capture_ms_; }
uint32_t ContinuousMeasurementController::last_sharpness_ms() const { return this->last_sharpness_ms_; }
uint32_t ContinuousMeasurementController::last_filter_ms() const { return this->last_filter_ms_; }
uint32_t ContinuousMeasurementController::last_filter_decode_ms() const { return this->last_filter_decode_ms_; }
uint32_t ContinuousMeasurementController::last_filter_correction_ms() const {
  return this->last_filter_correction_ms_;
}
uint32_t ContinuousMeasurementController::last_detect_ms() const { return this->last_detect_ms_; }
uint32_t ContinuousMeasurementController::last_compute_ms() const { return this->last_compute_ms_; }
uint32_t ContinuousMeasurementController::last_sharpness_score_x100() const { return this->last_sharpness_score_x100_; }
uint32_t ContinuousMeasurementController::sharpness_reference_score_x100() const { return this->sharpness_reference_score_x100_; }
bool ContinuousMeasurementController::last_sharpness_ok() const { return this->last_sharpness_ok_; }
uint8_t ContinuousMeasurementController::last_capture_retry_count() const { return this->last_capture_retry_count_; }
uint32_t ContinuousMeasurementController::blur_retry_count() const { return this->blur_retry_count_; }
bool ContinuousMeasurementController::sharpness_roi_valid() const { return this->sharpness_roi_valid_; }
uint16_t ContinuousMeasurementController::sharpness_roi_x() const { return this->sharpness_roi_x_; }
uint16_t ContinuousMeasurementController::sharpness_roi_y() const { return this->sharpness_roi_y_; }
uint16_t ContinuousMeasurementController::sharpness_roi_width() const { return this->sharpness_roi_width_; }
uint16_t ContinuousMeasurementController::sharpness_roi_height() const { return this->sharpness_roi_height_; }

void ContinuousMeasurementController::begin_cycle_() {
  this->cycle_started_ms_ = millis();

  // Memoriser le viewport qui va reellement produire cette image. Le tracking
  // peut changer de viewport APRES la detection du cycle ; sans ce snapshot,
  // l'API associait l'image precedente au mode suivant, ce qui rendait le
  // diagnostic du zoom trompeur.
  if (this->tracking_controller_ != nullptr && this->tracking_controller_->enabled() &&
      this->tracking_controller_->viewport_controller() != nullptr) {
    const auto &viewport = this->tracking_controller_->viewport_controller()->snapshot();
    this->current_cycle_viewport_mode_ = CameraViewportController::mode_text(viewport.mode);
    this->current_cycle_viewport_x_ = viewport.window_x;
    this->current_cycle_viewport_y_ = viewport.window_y;
    this->current_cycle_viewport_width_ = viewport.window_width;
    this->current_cycle_viewport_height_ = viewport.window_height;
  } else {
    this->current_cycle_viewport_mode_ = "search";
    this->current_cycle_viewport_x_ = 0;
    this->current_cycle_viewport_y_ = 0;
    this->current_cycle_viewport_width_ = 2560;
    this->current_cycle_viewport_height_ = 1920;
  }

  this->current_capture_ms_ = 0;
  this->current_sharpness_ms_ = 0;
  this->current_filter_ms_ = 0;
  this->current_filter_decode_ms_ = 0;
  this->current_filter_correction_ms_ = 0;
  this->current_detect_ms_ = 0;
  this->current_compute_ms_ = 0;
  this->last_sharpness_score_x100_ = 0;
  this->last_sharpness_ok_ = !this->sharpness_enabled_;
  this->last_capture_retry_count_ = 0;
}

void ContinuousMeasurementController::publish_cycle_timing_(uint32_t cycle_ms) {
  this->last_capture_ms_ = this->current_capture_ms_;
  this->last_sharpness_ms_ = this->current_sharpness_ms_;
  this->last_filter_ms_ = this->current_filter_ms_;
  this->last_filter_decode_ms_ = this->current_filter_decode_ms_;
  this->last_filter_correction_ms_ = this->current_filter_correction_ms_;
  this->last_detect_ms_ = this->current_detect_ms_;
  this->last_compute_ms_ = this->current_compute_ms_;
  this->last_cycle_ms_ = cycle_ms;
}

void ContinuousMeasurementController::finish_cycle_(bool target_found, bool measurement_valid) {
  const uint32_t now = millis();
  this->publish_cycle_timing_(now - this->cycle_started_ms_);
  this->cycle_count_++;
  if (target_found) this->target_found_count_++;
  if (measurement_valid) this->valid_measurement_count_++;
  this->last_cycle_target_found_ = target_found;
  this->last_cycle_measurement_valid_ = measurement_valid;
  this->last_cycle_viewport_mode_ = this->current_cycle_viewport_mode_;
  this->last_cycle_viewport_x_ = this->current_cycle_viewport_x_;
  this->last_cycle_viewport_y_ = this->current_cycle_viewport_y_;
  this->last_cycle_viewport_width_ = this->current_cycle_viewport_width_;
  this->last_cycle_viewport_height_ = this->current_cycle_viewport_height_;
  this->last_cycle_completed_ms_ = now;
  this->last_error_.clear();
  this->state_ = ContinuousMeasurementState::WAIT_INTERVAL;
}

void ContinuousMeasurementController::fail_cycle_(const char *error) {
  const uint32_t now = millis();
  this->publish_cycle_timing_(now - this->cycle_started_ms_);
  this->cycle_count_++;
  this->last_cycle_target_found_ = false;
  this->last_cycle_measurement_valid_ = false;
  this->last_cycle_viewport_mode_ = this->current_cycle_viewport_mode_;
  this->last_cycle_viewport_x_ = this->current_cycle_viewport_x_;
  this->last_cycle_viewport_y_ = this->current_cycle_viewport_y_;
  this->last_cycle_viewport_width_ = this->current_cycle_viewport_width_;
  this->last_cycle_viewport_height_ = this->current_cycle_viewport_height_;
  this->last_cycle_completed_ms_ = now;
  this->last_error_ = error != nullptr ? error : "cycle_failed";
  this->state_ = ContinuousMeasurementState::WAIT_INTERVAL;
  ESP_LOGW(TAG, "Cycle continu %u en echec: %s", static_cast<unsigned>(this->cycle_count_),
           this->last_error_.c_str());
}

void ContinuousMeasurementController::stop_with_error_(const char *error) {
  this->running_ = false;
  this->state_ = ContinuousMeasurementState::ERROR;
  this->last_error_ = error != nullptr ? error : "continuous_measurement_error";
  ESP_LOGE(TAG, "Mesure continue stoppee: %s", this->last_error_.c_str());
}

bool ContinuousMeasurementController::request_capture_() {
  this->capture_count_before_request_ = this->jpeg_source_->capture_count();
  return this->jpeg_source_->request_capture();
}

bool ContinuousMeasurementController::sharpness_is_too_low_(uint32_t score) const {
  if (this->sharpness_reference_score_x100_ == 0) return false;
  if (score == 0) return true;

  const uint64_t score_percent = static_cast<uint64_t>(score) * 100U;
  const uint64_t minimum = static_cast<uint64_t>(this->sharpness_reference_score_x100_) *
                           SHARPNESS_MIN_PERCENT_OF_REFERENCE;
  return score_percent < minimum;
}

void ContinuousMeasurementController::update_sharpness_reference_(uint32_t score) {
  if (score == 0) return;
  if (this->sharpness_reference_score_x100_ == 0) {
    this->sharpness_reference_score_x100_ = score;
    return;
  }

  const uint32_t previous = this->sharpness_reference_score_x100_;
  const uint32_t lower = static_cast<uint32_t>(
      (static_cast<uint64_t>(previous) * (100U - SHARPNESS_REFERENCE_MAX_STEP_PERCENT)) / 100U);
  const uint32_t upper = static_cast<uint32_t>(
      (static_cast<uint64_t>(previous) * (100U + SHARPNESS_REFERENCE_MAX_STEP_PERCENT)) / 100U);
  const uint32_t bounded_score = std::max(lower, std::min(upper, score));

  this->sharpness_reference_score_x100_ = static_cast<uint32_t>(
      ((SHARPNESS_REFERENCE_FILTER_DENOMINATOR - 1ULL) * previous + bounded_score) /
      SHARPNESS_REFERENCE_FILTER_DENOMINATOR);

  ESP_LOGD(TAG, "Reference nettete stabilisee: mesure=%u bornee=%u reference=%u",
           static_cast<unsigned>(score), static_cast<unsigned>(bounded_score),
           static_cast<unsigned>(this->sharpness_reference_score_x100_));
}

void ContinuousMeasurementController::update_sharpness_roi_from_target_() {
  if (!this->sharpness_enabled_ || this->detection_service_ == nullptr || this->filtered_source_ == nullptr ||
      !this->detection_service_->ready() || !this->detection_service_->target_found()) {
    return;
  }

  const uint16_t frame_width = this->filtered_source_->width();
  const uint16_t frame_height = this->filtered_source_->height();
  if (frame_width == 0 || frame_height == 0) return;

  const auto &target = this->detection_service_->last_observation();
  const float target_size = std::max(target.width_px, target.height_px);
  if (!target.valid || target_size <= 0.0f) return;

  uint32_t desired_size = static_cast<uint32_t>(std::ceil(target_size * SHARPNESS_ROI_TARGET_SCALE));
  desired_size = std::max<uint32_t>(desired_size, SHARPNESS_ROI_MIN_SIZE_PX);

  const uint16_t roi_width = static_cast<uint16_t>(std::min<uint32_t>(desired_size, frame_width));
  const uint16_t roi_height = static_cast<uint16_t>(std::min<uint32_t>(desired_size, frame_height));

  int32_t roi_x = static_cast<int32_t>(std::lround(target.center_x_px)) - static_cast<int32_t>(roi_width / 2U);
  int32_t roi_y = static_cast<int32_t>(std::lround(target.center_y_px)) - static_cast<int32_t>(roi_height / 2U);
  roi_x = std::max<int32_t>(0, std::min<int32_t>(roi_x, static_cast<int32_t>(frame_width - roi_width)));
  roi_y = std::max<int32_t>(0, std::min<int32_t>(roi_y, static_cast<int32_t>(frame_height - roi_height)));

  this->sharpness_roi_x_ = static_cast<uint16_t>(roi_x);
  this->sharpness_roi_y_ = static_cast<uint16_t>(roi_y);
  this->sharpness_roi_width_ = roi_width;
  this->sharpness_roi_height_ = roi_height;
  this->sharpness_roi_valid_ = true;

  ESP_LOGD(TAG, "ROI nettete cible: x=%u y=%u w=%u h=%u",
           static_cast<unsigned>(this->sharpness_roi_x_),
           static_cast<unsigned>(this->sharpness_roi_y_),
           static_cast<unsigned>(this->sharpness_roi_width_),
           static_cast<unsigned>(this->sharpness_roi_height_));
}

void ContinuousMeasurementController::reset_local_tracking_after_viewport_change_() {
  this->sharpness_roi_valid_ = false;
  this->sharpness_roi_x_ = 0;
  this->sharpness_roi_y_ = 0;
  this->sharpness_roi_width_ = 0;
  this->sharpness_roi_height_ = 0;
  this->sharpness_reference_score_x100_ = 0;
  if (this->detection_service_ != nullptr) {
    this->detection_service_->reset_tracking();
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
