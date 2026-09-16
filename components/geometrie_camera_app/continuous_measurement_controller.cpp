#include "continuous_measurement_controller.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
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
}

ContinuousMeasurementController::ContinuousMeasurementController(
    JpegDiagnostic *jpeg_source,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager)
    : jpeg_source_(jpeg_source),
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
      last_error_() {}

bool ContinuousMeasurementController::start(uint32_t interval_ms) {
  if (!this->set_interval_ms(interval_ms)) {
    this->last_error_ = "interval_ms_out_of_range";
    return false;
  }

  if (this->jpeg_source_ == nullptr || this->filtered_source_ == nullptr ||
      this->detection_service_ == nullptr || this->measurement_manager_ == nullptr) {
    this->last_error_ = "continuous_measurement_unavailable";
    return false;
  }

  if (!this->measurement_manager_->measurement_engine().has_calibration()) {
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
      this->capture_count_before_request_ = this->jpeg_source_->capture_count();
      if (!this->jpeg_source_->request_capture()) {
        this->fail_cycle_("capture_request_failed");
        return;
      }
      this->state_ = ContinuousMeasurementState::WAIT_CAPTURE;
      return;

    case ContinuousMeasurementState::WAIT_CAPTURE:
      if (!this->jpeg_source_->capture_pending() &&
          this->jpeg_source_->capture_count() > this->capture_count_before_request_ &&
          this->jpeg_source_->ready()) {
        this->state_ = ContinuousMeasurementState::FILTER;
        return;
      }
      if (now - this->cycle_started_ms_ > CAPTURE_TIMEOUT_MS) {
        this->fail_cycle_("capture_timeout");
      }
      return;

    case ContinuousMeasurementState::FILTER:
      if (!this->filtered_source_->process()) {
        this->fail_cycle_("filter_failed");
        return;
      }
      this->state_ = ContinuousMeasurementState::DETECT;
      return;

    case ContinuousMeasurementState::DETECT:
      if (!this->detection_service_->detect()) {
        this->fail_cycle_("detection_failed");
        return;
      }
      if (!this->detection_service_->target_found()) {
        this->finish_cycle_(false, false);
        return;
      }
      this->state_ = ContinuousMeasurementState::COMPUTE;
      return;

    case ContinuousMeasurementState::COMPUTE: {
      const bool measured = this->measurement_manager_->process(
          this->detection_service_->last_observation(), this->filtered_source_->width(),
          this->filtered_source_->height(), millis());
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
    case ContinuousMeasurementState::STOPPED:
      return "stopped";
    case ContinuousMeasurementState::REQUEST_CAPTURE:
      return "request_capture";
    case ContinuousMeasurementState::WAIT_CAPTURE:
      return "wait_capture";
    case ContinuousMeasurementState::FILTER:
      return "filter";
    case ContinuousMeasurementState::DETECT:
      return "detect";
    case ContinuousMeasurementState::COMPUTE:
      return "compute";
    case ContinuousMeasurementState::WAIT_INTERVAL:
      return "wait_interval";
    case ContinuousMeasurementState::ERROR:
      return "error";
    default:
      return "unknown";
  }
}

uint32_t ContinuousMeasurementController::cycle_count() const { return this->cycle_count_; }
uint32_t ContinuousMeasurementController::target_found_count() const { return this->target_found_count_; }
uint32_t ContinuousMeasurementController::valid_measurement_count() const { return this->valid_measurement_count_; }
uint32_t ContinuousMeasurementController::last_cycle_ms() const { return this->last_cycle_ms_; }
bool ContinuousMeasurementController::last_cycle_target_found() const { return this->last_cycle_target_found_; }
bool ContinuousMeasurementController::last_cycle_measurement_valid() const { return this->last_cycle_measurement_valid_; }
const std::string &ContinuousMeasurementController::last_error() const { return this->last_error_; }

void ContinuousMeasurementController::begin_cycle_() {
  this->cycle_started_ms_ = millis();
}

void ContinuousMeasurementController::finish_cycle_(bool target_found, bool measurement_valid) {
  const uint32_t now = millis();
  this->cycle_count_++;
  if (target_found) {
    this->target_found_count_++;
  }
  if (measurement_valid) {
    this->valid_measurement_count_++;
  }
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

}  // namespace geometrie_camera_app
}  // namespace esphome
