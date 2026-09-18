#include "continuous_measurement_controller.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
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
// Au-dela de cet intervalle, on privilegie une image strictement posterieure
// a la demande pour eviter d'utiliser une frame restee trop longtemps en
// attente. Le mode normal de geometrie (1000 ms) profite du pipeline.
constexpr uint32_t MAX_PIPELINED_INTERVAL_MS = 1500;
}

ContinuousMeasurementController::ContinuousMeasurementController(
    JpegDiagnostic *jpeg_source,
    JpegFilteredDiagnostic *decoded_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager,
    TargetTrackingController *tracking_controller)
    : jpeg_source_(jpeg_source),
      decoded_source_(decoded_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      tracking_controller_(tracking_controller),
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
      current_decode_ms_(0),
      current_detect_ms_(0),
      current_compute_ms_(0),
      last_capture_ms_(0),
      last_decode_ms_(0),
      last_detect_ms_(0),
      last_compute_ms_(0),
      force_fresh_capture_(true),
      current_capture_pipelined_(false),
      last_capture_pipelined_(false) {}

bool ContinuousMeasurementController::start(uint32_t interval_ms) {
  if (!this->set_interval_ms(interval_ms)) {
    this->running_ = false;
    this->state_ = ContinuousMeasurementState::ERROR;
    this->last_error_ = "interval_ms_out_of_range";
    return false;
  }

  if (this->jpeg_source_ == nullptr || this->decoded_source_ == nullptr ||
      this->detection_service_ == nullptr || this->measurement_manager_ == nullptr ||
      this->tracking_controller_ == nullptr) {
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

  if (!this->tracking_controller_->start()) {
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
  this->current_decode_ms_ = 0;
  this->current_detect_ms_ = 0;
  this->current_compute_ms_ = 0;
  this->last_capture_ms_ = 0;
  this->last_decode_ms_ = 0;
  this->last_detect_ms_ = 0;
  this->last_compute_ms_ = 0;
  this->force_fresh_capture_ = true;
  this->current_capture_pipelined_ = false;
  this->last_capture_pipelined_ = false;
  this->last_error_.clear();

  this->reset_local_tracking_after_viewport_change_();

  ESP_LOGI(TAG, "Mesure continue demarree, intervalle=%u ms, tracking haute precision permanent",
           static_cast<unsigned>(this->interval_ms_));
  return true;
}

void ContinuousMeasurementController::stop() {
  const bool was_running = this->running_;
  this->running_ = false;
  this->state_ = ContinuousMeasurementState::STOPPED;

  if (was_running && this->jpeg_source_ != nullptr) {
    this->jpeg_source_->cancel_capture();
  }
  if (this->measurement_manager_ != nullptr) {
    this->measurement_manager_->reset_stabilization();
  }
  if (this->tracking_controller_ != nullptr) {
    this->tracking_controller_->stop();
    this->reset_local_tracking_after_viewport_change_();
  }

  if (was_running) {
    ESP_LOGI(TAG, "Mesure continue arretee apres %u cycles",
             static_cast<unsigned>(this->cycle_count_));
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
        this->state_ = ContinuousMeasurementState::DECODE;
        return;
      }
      if (this->jpeg_source_->request_started_ms() != 0 &&
          now - this->jpeg_source_->request_started_ms() > CAPTURE_TIMEOUT_MS) {
        this->fail_cycle_("capture_timeout");
      }
      return;

    case ContinuousMeasurementState::DECODE:
      if (!this->decoded_source_->process()) {
        if (this->running_) this->fail_cycle_("decode_failed");
        return;
      }
      if (!this->running_) return;
      this->current_decode_ms_ = this->decoded_source_->total_ms();
      this->state_ = ContinuousMeasurementState::DETECT;
      return;

    case ContinuousMeasurementState::DETECT: {
      if (!this->detection_service_->detect()) {
        if (this->running_) this->fail_cycle_("detection_failed");
        return;
      }
      if (!this->running_) return;

      this->current_detect_ms_ = this->detection_service_->detection_ms();
      const bool target_found = this->detection_service_->target_found();

      if (!target_found) {
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
        this->finish_cycle_(false, false);
        return;
      }

      // SEARCH/WIDE/MEDIUM/FINE servent uniquement au tracking.
      // Seul PRECISE produit une mesure geometrique.
      if (this->current_cycle_viewport_mode_ != "precise") {
        const TrackingUpdateResult tracking_result =
            this->tracking_controller_->update_after_detection(
                true, this->detection_service_->last_observation());
        if (tracking_result == TrackingUpdateResult::ERROR) {
          this->fail_cycle_("tracking_update_failed");
          return;
        }
        if (tracking_result == TrackingUpdateResult::VIEWPORT_CHANGED) {
          this->reset_local_tracking_after_viewport_change_();
        }
        this->current_compute_ms_ = 0;
        this->finish_cycle_(true, false);
        return;
      }

      this->state_ = ContinuousMeasurementState::COMPUTE;
      return;
    }

    case ContinuousMeasurementState::COMPUTE: {
      const uint32_t compute_started_ms = millis();
      TargetObservation measurement_observation =
          this->detection_service_->last_observation();
      measurement_observation =
          this->tracking_controller_->to_reference(measurement_observation);

      const bool measured = this->measurement_manager_->process(
          measurement_observation,
          this->tracking_controller_->reference_width(),
          this->tracking_controller_->reference_height(),
          millis(), true);
      if (!this->running_) return;

      this->current_compute_ms_ = millis() - compute_started_ms;
      if (!measured) {
        this->fail_cycle_("measurement_failed");
        return;
      }

      const TrackingUpdateResult tracking_result =
          this->tracking_controller_->update_after_detection(
              true, this->detection_service_->last_observation());
      if (tracking_result == TrackingUpdateResult::ERROR) {
        this->fail_cycle_("tracking_update_failed");
        return;
      }
      if (tracking_result == TrackingUpdateResult::VIEWPORT_CHANGED) {
        this->reset_local_tracking_after_viewport_change_();
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

bool ContinuousMeasurementController::running() const { return this->running_; }
uint32_t ContinuousMeasurementController::interval_ms() const { return this->interval_ms_; }
ContinuousMeasurementState ContinuousMeasurementController::state() const { return this->state_; }

const char *ContinuousMeasurementController::state_text() const {
  switch (this->state_) {
    case ContinuousMeasurementState::STOPPED: return "stopped";
    case ContinuousMeasurementState::REQUEST_CAPTURE: return "request_capture";
    case ContinuousMeasurementState::WAIT_CAPTURE: return "wait_capture";
    case ContinuousMeasurementState::DECODE: return "decode";
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
uint16_t ContinuousMeasurementController::last_cycle_viewport_x() const { return this->last_cycle_viewport_x_; }
uint16_t ContinuousMeasurementController::last_cycle_viewport_y() const { return this->last_cycle_viewport_y_; }
uint16_t ContinuousMeasurementController::last_cycle_viewport_width() const { return this->last_cycle_viewport_width_; }
uint16_t ContinuousMeasurementController::last_cycle_viewport_height() const { return this->last_cycle_viewport_height_; }
const std::string &ContinuousMeasurementController::last_error() const { return this->last_error_; }
uint32_t ContinuousMeasurementController::last_capture_ms() const { return this->last_capture_ms_; }
uint32_t ContinuousMeasurementController::last_decode_ms() const { return this->last_decode_ms_; }
uint32_t ContinuousMeasurementController::last_detect_ms() const { return this->last_detect_ms_; }
uint32_t ContinuousMeasurementController::last_compute_ms() const { return this->last_compute_ms_; }
bool ContinuousMeasurementController::last_capture_pipelined() const {
  return this->last_capture_pipelined_;
}

void ContinuousMeasurementController::begin_cycle_() {
  this->cycle_started_ms_ = millis();

  if (this->tracking_controller_ != nullptr &&
      this->tracking_controller_->viewport_controller() != nullptr) {
    const auto &viewport =
        this->tracking_controller_->viewport_controller()->snapshot();
    this->current_cycle_viewport_mode_ =
        CameraViewportController::mode_text(viewport.mode);
    this->current_cycle_viewport_x_ = viewport.window_x;
    this->current_cycle_viewport_y_ = viewport.window_y;
    this->current_cycle_viewport_width_ = viewport.window_width;
    this->current_cycle_viewport_height_ = viewport.window_height;
  }

  this->current_capture_ms_ = 0;
  this->current_decode_ms_ = 0;
  this->current_detect_ms_ = 0;
  this->current_compute_ms_ = 0;
  this->current_capture_pipelined_ = false;
}

void ContinuousMeasurementController::publish_cycle_timing_(uint32_t cycle_ms) {
  this->last_capture_ms_ = this->current_capture_ms_;
  this->last_decode_ms_ = this->current_decode_ms_;
  this->last_detect_ms_ = this->current_detect_ms_;
  this->last_compute_ms_ = this->current_compute_ms_;
  this->last_capture_pipelined_ = this->current_capture_pipelined_;
  this->last_cycle_ms_ = cycle_ms;
}

void ContinuousMeasurementController::finish_cycle_(
    bool target_found, bool measurement_valid) {
  if (!this->running_) return;

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
  if (!this->running_) return;

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
  ESP_LOGW(TAG, "Cycle continu %u en echec: %s",
           static_cast<unsigned>(this->cycle_count_),
           this->last_error_.c_str());
}

void ContinuousMeasurementController::stop_with_error_(const char *error) {
  this->running_ = false;
  this->state_ = ContinuousMeasurementState::ERROR;
  this->last_error_ =
      error != nullptr ? error : "continuous_measurement_error";
  ESP_LOGE(TAG, "Mesure continue stoppee: %s", this->last_error_.c_str());
}

bool ContinuousMeasurementController::request_capture_() {
  if (!this->running_ || this->jpeg_source_ == nullptr) return false;

  this->capture_count_before_request_ = this->jpeg_source_->capture_count();

  const bool allow_pipeline =
      !this->force_fresh_capture_ &&
      this->interval_ms_ <= MAX_PIPELINED_INTERVAL_MS;

  // force_post_request_frame=true conserve l'ancien comportement totalement
  // frais. En regime stable, false accepte la prochaine frame sequentielle
  // deja en cours d'acquisition pendant le traitement precedent.
  const bool accepted =
      this->jpeg_source_->request_capture(!allow_pipeline);
  if (accepted) {
    this->current_capture_pipelined_ = allow_pipeline;
    this->force_fresh_capture_ = false;
  }
  return accepted;
}

void ContinuousMeasurementController::reset_local_tracking_after_viewport_change_() {
  // La frame deja pre-acquise peut encore appartenir a l'ancien viewport.
  // Le prochain cycle doit donc utiliser la purge historique.
  this->force_fresh_capture_ = true;

  if (this->measurement_manager_ != nullptr) {
    this->measurement_manager_->reset_stabilization();
  }
  if (this->detection_service_ != nullptr) {
    this->detection_service_->reset_tracking();
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
