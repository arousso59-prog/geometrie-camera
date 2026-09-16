#include "continuous_measurement_controller.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "image_sharpness_evaluator.h"
#include "jpeg_diagnostic.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_manager.h"
#include "target_detection_service.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "continuous_measurement";
constexpr uint32_t MIN_INTERVAL_MS = 200;
constexpr uint32_t MAX_INTERVAL_MS = 10000;
constexpr uint32_t CAPTURE_TIMEOUT_MS = 10000;
constexpr uint8_t MAX_BLUR_RETRIES = 2;
constexpr uint32_t SHARPNESS_MIN_PERCENT_OF_REFERENCE = 60;
}

ContinuousMeasurementController::ContinuousMeasurementController(
    JpegDiagnostic *jpeg_source,
    ImageSharpnessEvaluator *sharpness_evaluator,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager)
    : jpeg_source_(jpeg_source),
      sharpness_evaluator_(sharpness_evaluator),
      filtered_source_(filtered_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      running_(false),
      interval_ms_(1000),
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
      last_error_(),
      last_capture_ms_(0),
      last_sharpness_ms_(0),
      last_filter_ms_(0),
      last_detect_ms_(0),
      last_compute_ms_(0),
      last_sharpness_score_x100_(0),
      sharpness_reference_score_x100_(0),
      last_sharpness_ok_(false),
      last_capture_retry_count_(0),
      blur_retry_count_(0) {}

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
  this->last_capture_ms_ = 0;
  this->last_sharpness_ms_ = 0;
  this->last_filter_ms_ = 0;
  this->last_detect_ms_ = 0;
  this->last_compute_ms_ = 0;
  this->last_sharpness_score_x100_ = 0;
  this->sharpness_reference_score_x100_ = 0;
  this->last_sharpness_ok_ = false;
  this->last_capture_retry_count_ = 0;
  this->blur_retry_count_ = 0;
  this->last_error_.clear();
  ESP_LOGI(TAG, "Mesure continue demarree, intervalle=%u ms", static_cast<unsigned>(this->interval_ms_));
  return true;
}

void ContinuousMeasurementController::stop() {
  if (this->running_) {
    ESP_LOGI(TAG, "Mesure continue arretee apres %u cycles", static_cast<unsigned>(this->cycle_count_));
  }
  this->running_ = false;
  this->state_ = ContinuousMeasurementState::STOPPED;
}

void ContinuousMeasurementController::loop() {
  if (!this->running_) {
    return;
  }

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
        this->last_capture_ms_ += this->jpeg_source_->total_cycle_ms();
        this->state_ = ContinuousMeasurementState::SHARPNESS;
        return;
      }
      if (this->jpeg_source_->request_started_ms() != 0 &&
          now - this->jpeg_source_->request_started_ms() > CAPTURE_TIMEOUT_MS) {
        this->fail_cycle_("capture_timeout");
      }
      return;

    case ContinuousMeasurementState::SHARPNESS: {
      if (!this->sharpness_evaluator_->evaluate()) {
        this->fail_cycle_("sharpness_evaluation_failed");
        return;
      }

      this->last_sharpness_ms_ += this->sharpness_evaluator_->evaluation_ms();
      this->last_sharpness_score_x100_ = this->sharpness_evaluator_->score_x100();
      const bool too_blurry = this->sharpness_is_too_low_(this->last_sharpness_score_x100_);

      if (too_blurry && this->last_capture_retry_count_ < MAX_BLUR_RETRIES) {
        this->last_sharpness_ok_ = false;
        this->last_capture_retry_count_++;
        this->blur_retry_count_++;
        ESP_LOGI(TAG, "Image floue rejetee: score_x100=%u reference=%u, recapture %u/%u",
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
      if (!too_blurry) {
        this->update_sharpness_reference_(this->last_sharpness_score_x100_);
      } else {
        ESP_LOGW(TAG, "Image encore floue apres %u recaptures; pipeline poursuivi pour ne pas bloquer",
                 static_cast<unsigned>(this->last_capture_retry_count_));
      }
      this->state_ = ContinuousMeasurementState::FILTER;
      return;
    }

    case ContinuousMeasurementState::FILTER:
      if (!this->filtered_source_->process()) {
        this->fail_cycle_("filter_failed");
        return;
      }
      this->last_filter_ms_ = this->filtered_source_->total_ms();
      this->state_ = ContinuousMeasurementState::DETECT;
      return;

    case ContinuousMeasurementState::DETECT:
      if (!this->detection_service_->detect()) {
        this->fail_cycle_("detection_failed");
        return;
      }
      this->last_detect_ms_ = this->detection_service_->detection_ms();
      if (!this->detection_service_->target_found()) {
        this->finish_cycle_(false, false);
        return;
      }
      this->state_ = ContinuousMeasurementState::COMPUTE;
      return;

    case ContinuousMeasurementState::COMPUTE: {
      const uint32_t compute_started_ms = millis();
      const bool measured = this->measurement_manager_->process(
          this->detection_service_->last_observation(), this->filtered_source_->width(),
          this->filtered_source_->height(), millis());
      this->last_compute_ms_ = millis() - compute_started_ms;
      if (!measured) {
        this->fail_cycle_("measurement_failed");
        return;
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
  if (interval_ms < MIN_INTERVAL_MS || interval_ms > MAX_INTERVAL_MS) {
    return false;
  }
  this->interval_ms_ = interval_ms;
  return true;
}

bool ContinuousMeasurementController::running() const { return this->running_; }
uint32_t ContinuousMeasurementController::interval_ms() const { return this->interval_ms_; }
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
const std::string &ContinuousMeasurementController::last_error() const { return this->last_error_; }
uint32_t ContinuousMeasurementController::last_capture_ms() const { return this->last_capture_ms_; }
uint32_t ContinuousMeasurementController::last_sharpness_ms() const { return this->last_sharpness_ms_; }
uint32_t ContinuousMeasurementController::last_filter_ms() const { return this->last_filter_ms_; }
uint32_t ContinuousMeasurementController::last_detect_ms() const { return this->last_detect_ms_; }
uint32_t ContinuousMeasurementController::last_compute_ms() const { return this->last_compute_ms_; }
uint32_t ContinuousMeasurementController::last_sharpness_score_x100() const { return this->last_sharpness_score_x100_; }
uint32_t ContinuousMeasurementController::sharpness_reference_score_x100() const { return this->sharpness_reference_score_x100_; }
bool ContinuousMeasurementController::last_sharpness_ok() const { return this->last_sharpness_ok_; }
uint8_t ContinuousMeasurementController::last_capture_retry_count() const { return this->last_capture_retry_count_; }
uint32_t ContinuousMeasurementController::blur_retry_count() const { return this->blur_retry_count_; }

void ContinuousMeasurementController::begin_cycle_() {
  this->cycle_started_ms_ = millis();
  this->last_capture_ms_ = 0;
  this->last_sharpness_ms_ = 0;
  this->last_filter_ms_ = 0;
  this->last_detect_ms_ = 0;
  this->last_compute_ms_ = 0;
  this->last_sharpness_score_x100_ = 0;
  this->last_sharpness_ok_ = false;
  this->last_capture_retry_count_ = 0;
}

void ContinuousMeasurementController::finish_cycle_(bool target_found, bool measurement_valid) {
  const uint32_t now = millis();
  this->cycle_count_++;
  if (target_found) this->target_found_count_++;
  if (measurement_valid) this->valid_measurement_count_++;
  this->last_cycle_target_found_ = target_found;
  this->last_cycle_measurement_valid_ = measurement_valid;
  this->last_cycle_completed_ms_ = now;
  this->last_cycle_ms_ = now - this->cycle_started_ms_;
  this->last_error_.clear();
  this->state_ = ContinuousMeasurementState::WAIT_INTERVAL;
}

void ContinuousMeasurementController::fail_cycle_(const char *error) {
  const uint32_t now = millis();
  this->cycle_count_++;
  this->last_cycle_target_found_ = false;
  this->last_cycle_measurement_valid_ = false;
  this->last_cycle_completed_ms_ = now;
  this->last_cycle_ms_ = now - this->cycle_started_ms_;
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
  if (score == 0) {
    return true;
  }
  if (this->sharpness_reference_score_x100_ == 0) {
    return false;
  }

  const uint64_t score_percent = static_cast<uint64_t>(score) * 100U;
  const uint64_t minimum = static_cast<uint64_t>(this->sharpness_reference_score_x100_) *
                           SHARPNESS_MIN_PERCENT_OF_REFERENCE;
  return score_percent < minimum;
}

void ContinuousMeasurementController::update_sharpness_reference_(uint32_t score) {
  if (score == 0) {
    return;
  }
  if (this->sharpness_reference_score_x100_ == 0) {
    this->sharpness_reference_score_x100_ = score;
    return;
  }

  if (score > this->sharpness_reference_score_x100_) {
    this->sharpness_reference_score_x100_ = static_cast<uint32_t>(
        (3ULL * this->sharpness_reference_score_x100_ + score) / 4ULL);
  } else {
    this->sharpness_reference_score_x100_ = static_cast<uint32_t>(
        (15ULL * this->sharpness_reference_score_x100_ + score) / 16ULL);
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
