#include "target_tracking_controller.h"

#include "camera_viewport_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_tracking";
}

TargetTrackingController::TargetTrackingController(CameraViewportController *viewport_controller)
    : viewport_controller_(viewport_controller),
      enabled_(false),
      active_(false),
      lost_cycles_(3),
      recenter_threshold_pct_(70),
      current_lost_count_(0),
      transition_count_(0),
      target_locked_(false),
      last_error_() {}

bool TargetTrackingController::start() {
  this->clear_runtime_();
  this->active_ = false;
  if (!this->enabled_) {
    return true;
  }
  if (this->viewport_controller_ == nullptr || !this->viewport_controller_->supports_precise_roi()) {
    this->last_error_ = "precise_roi_unsupported";
    return false;
  }
  if (!this->viewport_controller_->apply_search()) {
    this->last_error_ = "search_viewport_failed";
    return false;
  }
  this->active_ = true;
  return true;
}

void TargetTrackingController::stop() {
  if (this->active_ && this->viewport_controller_ != nullptr) {
    this->viewport_controller_->apply_search();
  }
  this->active_ = false;
  this->clear_runtime_();
}

bool TargetTrackingController::set_enabled(bool enabled) {
  if (this->active_) {
    this->last_error_ = "tracking_active";
    return false;
  }
  this->enabled_ = enabled;
  this->clear_runtime_();
  return true;
}

bool TargetTrackingController::set_lost_cycles(uint8_t cycles) {
  if (this->active_) {
    this->last_error_ = "tracking_active";
    return false;
  }
  if (cycles < 1 || cycles > 10) return false;
  this->lost_cycles_ = cycles;
  this->last_error_.clear();
  return true;
}

bool TargetTrackingController::set_recenter_threshold_pct(uint8_t percent) {
  if (this->active_) {
    this->last_error_ = "tracking_active";
    return false;
  }
  if (percent < 50 || percent > 90) return false;
  this->recenter_threshold_pct_ = percent;
  this->last_error_.clear();
  return true;
}

bool TargetTrackingController::enabled() const { return this->enabled_; }
bool TargetTrackingController::active() const { return this->active_; }
bool TargetTrackingController::supported() const {
  return this->viewport_controller_ != nullptr && this->viewport_controller_->supports_precise_roi();
}
uint8_t TargetTrackingController::lost_cycles() const { return this->lost_cycles_; }
uint8_t TargetTrackingController::recenter_threshold_pct() const { return this->recenter_threshold_pct_; }
uint8_t TargetTrackingController::current_lost_count() const { return this->current_lost_count_; }
uint32_t TargetTrackingController::transition_count() const { return this->transition_count_; }
bool TargetTrackingController::target_locked() const { return this->target_locked_; }
const std::string &TargetTrackingController::last_error() const { return this->last_error_; }

TrackingUpdateResult TargetTrackingController::update_after_detection(
    bool target_found, const TargetObservation &local_observation) {
  if (!this->active_ || this->viewport_controller_ == nullptr) {
    return TrackingUpdateResult::NONE;
  }

  if (target_found && local_observation.valid) {
    this->target_locked_ = true;
    this->current_lost_count_ = 0;
    const TargetObservation reference = this->viewport_controller_->to_reference(local_observation);

    if (this->viewport_controller_->snapshot().mode == CameraViewportMode::SEARCH_FULL) {
      if (!this->viewport_controller_->apply_precise_roi(reference.center_x_px, reference.center_y_px)) {
        this->last_error_ = "precise_viewport_failed";
        return TrackingUpdateResult::ERROR;
      }
      this->transition_count_++;
      this->last_error_.clear();
      ESP_LOGI(TAG, "Tracking SEARCH -> PRECISE");
      return TrackingUpdateResult::VIEWPORT_CHANGED;
    }

    if (this->viewport_controller_->target_near_edge(local_observation,
                                                      this->recenter_threshold_pct_)) {
      if (!this->viewport_controller_->apply_precise_roi(reference.center_x_px, reference.center_y_px)) {
        this->last_error_ = "precise_recenter_failed";
        return TrackingUpdateResult::ERROR;
      }
      this->transition_count_++;
      this->last_error_.clear();
      ESP_LOGD(TAG, "Tracking PRECISE recentered");
      return TrackingUpdateResult::VIEWPORT_CHANGED;
    }

    this->last_error_.clear();
    return TrackingUpdateResult::NONE;
  }

  this->target_locked_ = false;
  if (this->current_lost_count_ < 255) this->current_lost_count_++;

  if (this->viewport_controller_->snapshot().mode == CameraViewportMode::PRECISE_ROI &&
      this->current_lost_count_ >= this->lost_cycles_) {
    if (!this->viewport_controller_->apply_search()) {
      this->last_error_ = "search_recovery_failed";
      return TrackingUpdateResult::ERROR;
    }
    this->current_lost_count_ = 0;
    this->transition_count_++;
    this->last_error_.clear();
    ESP_LOGI(TAG, "Tracking PRECISE -> SEARCH apres pertes cible");
    return TrackingUpdateResult::VIEWPORT_CHANGED;
  }

  return TrackingUpdateResult::NONE;
}

TargetObservation TargetTrackingController::to_reference(
    const TargetObservation &local_observation) const {
  if (this->viewport_controller_ == nullptr) return local_observation;
  return this->viewport_controller_->to_reference(local_observation);
}

uint16_t TargetTrackingController::reference_width() const {
  return CameraViewportController::REFERENCE_WIDTH;
}
uint16_t TargetTrackingController::reference_height() const {
  return CameraViewportController::REFERENCE_HEIGHT;
}

const char *TargetTrackingController::mode_text() const {
  if (this->viewport_controller_ == nullptr) return "unavailable";
  return CameraViewportController::mode_text(this->viewport_controller_->snapshot().mode);
}

CameraViewportController *TargetTrackingController::viewport_controller() {
  return this->viewport_controller_;
}
const CameraViewportController *TargetTrackingController::viewport_controller() const {
  return this->viewport_controller_;
}

void TargetTrackingController::clear_runtime_() {
  this->current_lost_count_ = 0;
  this->transition_count_ = 0;
  this->target_locked_ = false;
  this->last_error_.clear();
}

}  // namespace geometrie_camera_app
}  // namespace esphome
