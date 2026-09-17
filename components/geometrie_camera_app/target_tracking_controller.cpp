#include "target_tracking_controller.h"

#include <algorithm>
#include <cmath>

#include "camera_viewport_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_tracking";
constexpr float RECOVERY_MIN_SIDE_PX = 12.0f;
constexpr float RECOVERY_MAX_ASPECT_RATIO = 2.0f;

// Au premier passage en PRECISE, on veut vraiment ramener la cible pres du
// centre de la fenetre et pas seulement verifier qu'elle n'est pas pres du bord.
// Cela compense aussi un petit decalage constant entre le repere SEARCH
// (binning + scaling) et la fenetre native OV5640.
constexpr float PRECISE_CENTER_TOLERANCE_FRACTION = 0.06f;
constexpr float PRECISE_CENTER_MIN_TOLERANCE_PX = 24.0f;
constexpr uint8_t PRECISE_CENTER_MAX_ATTEMPTS = 3;

bool has_recovery_candidate(const TargetObservation &observation,
                            const CameraViewportSnapshot &snapshot) {
  if (observation.valid || snapshot.output_width == 0 || snapshot.output_height == 0) {
    return false;
  }
  if (!std::isfinite(observation.center_x_px) || !std::isfinite(observation.center_y_px) ||
      !std::isfinite(observation.width_px) || !std::isfinite(observation.height_px) ||
      !std::isfinite(observation.quality)) {
    return false;
  }
  if (observation.width_px < RECOVERY_MIN_SIDE_PX || observation.height_px < RECOVERY_MIN_SIDE_PX ||
      observation.quality <= 0.0f) {
    return false;
  }

  const float aspect = std::max(observation.width_px / observation.height_px,
                                observation.height_px / observation.width_px);
  if (aspect > RECOVERY_MAX_ASPECT_RATIO) {
    return false;
  }

  return observation.center_x_px >= 0.0f &&
         observation.center_y_px >= 0.0f &&
         observation.center_x_px < snapshot.output_width &&
         observation.center_y_px < snapshot.output_height;
}

bool recovery_candidate_needs_recenter(const TargetObservation &observation,
                                       const CameraViewportSnapshot &snapshot,
                                       uint8_t central_percent) {
  if (snapshot.output_width == 0 || snapshot.output_height == 0) return false;

  central_percent = std::max<uint8_t>(50, std::min<uint8_t>(90, central_percent));
  const float margin_fraction = (100.0f - central_percent) / 200.0f;
  const float margin_x = snapshot.output_width * margin_fraction;
  const float margin_y = snapshot.output_height * margin_fraction;

  return observation.center_x_px < margin_x ||
         observation.center_x_px > snapshot.output_width - margin_x ||
         observation.center_y_px < margin_y ||
         observation.center_y_px > snapshot.output_height - margin_y;
}

bool precise_centering_needed(const TargetObservation &observation,
                              const CameraViewportSnapshot &snapshot) {
  if (snapshot.output_width == 0 || snapshot.output_height == 0 ||
      !std::isfinite(observation.center_x_px) || !std::isfinite(observation.center_y_px)) {
    return false;
  }

  const float desired_x = snapshot.output_width * 0.5f;
  const float desired_y = snapshot.output_height * 0.5f;
  const float tolerance_x = std::max(PRECISE_CENTER_MIN_TOLERANCE_PX,
                                     snapshot.output_width * PRECISE_CENTER_TOLERANCE_FRACTION);
  const float tolerance_y = std::max(PRECISE_CENTER_MIN_TOLERANCE_PX,
                                     snapshot.output_height * PRECISE_CENTER_TOLERANCE_FRACTION);

  return std::fabs(observation.center_x_px - desired_x) > tolerance_x ||
         std::fabs(observation.center_y_px - desired_y) > tolerance_y;
}
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
      precise_centered_(false),
      precise_center_attempts_(0),
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
      this->precise_centered_ = false;
      this->precise_center_attempts_ = 0;
      this->transition_count_++;
      this->last_error_.clear();
      ESP_LOGI(TAG, "Tracking SEARCH -> PRECISE");
      return TrackingUpdateResult::VIEWPORT_CHANGED;
    }

    const CameraViewportSnapshot &snapshot = this->viewport_controller_->snapshot();

    // Valider le premier cadrage PRECISE par la position effectivement observee.
    // Si la cible n'est pas proche du centre (400,300), deplacer la ROI de
    // l'erreur mesuree puis verifier de nouveau au cycle suivant.
    if (!this->precise_centered_) {
      if (precise_centering_needed(local_observation, snapshot) &&
          this->precise_center_attempts_ < PRECISE_CENTER_MAX_ATTEMPTS) {
        if (!this->viewport_controller_->apply_precise_roi(reference.center_x_px, reference.center_y_px)) {
          this->last_error_ = "precise_centering_failed";
          return TrackingUpdateResult::ERROR;
        }

        this->precise_center_attempts_++;
        this->transition_count_++;
        this->last_error_.clear();
        ESP_LOGI(TAG,
                 "Tracking PRECISE centrage fin %u/%u: cible locale=(%.1f,%.1f), centre attendu=(%.1f,%.1f), ref=(%.1f,%.1f)",
                 static_cast<unsigned>(this->precise_center_attempts_),
                 static_cast<unsigned>(PRECISE_CENTER_MAX_ATTEMPTS),
                 local_observation.center_x_px, local_observation.center_y_px,
                 snapshot.output_width * 0.5f, snapshot.output_height * 0.5f,
                 reference.center_x_px, reference.center_y_px);
        return TrackingUpdateResult::VIEWPORT_CHANGED;
      }

      if (precise_centering_needed(local_observation, snapshot) &&
          this->precise_center_attempts_ >= PRECISE_CENTER_MAX_ATTEMPTS) {
        ESP_LOGW(TAG,
                 "Centrage PRECISE encore decale apres %u tentatives: cible locale=(%.1f,%.1f)",
                 static_cast<unsigned>(this->precise_center_attempts_),
                 local_observation.center_x_px, local_observation.center_y_px);
      }
      this->precise_centered_ = true;
      this->precise_center_attempts_ = 0;
    }

    if (this->viewport_controller_->target_near_edge(local_observation,
                                                      this->recenter_threshold_pct_)) {
      if (!this->viewport_controller_->apply_precise_roi(reference.center_x_px, reference.center_y_px)) {
        this->last_error_ = "precise_recenter_failed";
        return TrackingUpdateResult::ERROR;
      }
      this->precise_centered_ = false;
      this->precise_center_attempts_ = 0;
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

  const CameraViewportSnapshot &snapshot = this->viewport_controller_->snapshot();
  const bool fallback_needs_centering =
      !this->precise_centered_ && precise_centering_needed(local_observation, snapshot);
  if (snapshot.mode == CameraViewportMode::PRECISE_ROI &&
      this->current_lost_count_ < this->lost_cycles_ &&
      has_recovery_candidate(local_observation, snapshot) &&
      (fallback_needs_centering ||
       recovery_candidate_needs_recenter(local_observation, snapshot,
                                         this->recenter_threshold_pct_))) {
    const TargetObservation reference = this->viewport_controller_->to_reference(local_observation);
    if (!this->viewport_controller_->apply_precise_roi(reference.center_x_px, reference.center_y_px)) {
      this->last_error_ = "precise_recovery_recenter_failed";
      return TrackingUpdateResult::ERROR;
    }

    if (fallback_needs_centering && this->precise_center_attempts_ < PRECISE_CENTER_MAX_ATTEMPTS) {
      this->precise_center_attempts_++;
    }
    this->transition_count_++;
    this->last_error_.clear();
    ESP_LOGI(TAG,
             "Tracking PRECISE recentrage secours: local=(%.1f,%.1f) taille=%.1fx%.1f qualite=%.3f -> ref=(%.1f,%.1f), perte=%u/%u",
             local_observation.center_x_px, local_observation.center_y_px,
             local_observation.width_px, local_observation.height_px,
             local_observation.quality, reference.center_x_px, reference.center_y_px,
             static_cast<unsigned>(this->current_lost_count_),
             static_cast<unsigned>(this->lost_cycles_));
    return TrackingUpdateResult::VIEWPORT_CHANGED;
  }

  if (snapshot.mode == CameraViewportMode::PRECISE_ROI &&
      this->current_lost_count_ >= this->lost_cycles_) {
    if (!this->viewport_controller_->apply_search()) {
      this->last_error_ = "search_recovery_failed";
      return TrackingUpdateResult::ERROR;
    }
    this->current_lost_count_ = 0;
    this->precise_centered_ = false;
    this->precise_center_attempts_ = 0;
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
  this->precise_centered_ = false;
  this->precise_center_attempts_ = 0;
  this->last_error_.clear();
}

}  // namespace geometrie_camera_app
}  // namespace esphome
