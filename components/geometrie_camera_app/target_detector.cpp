#include "target_detector.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_detector";

constexpr float CONTINUITY_BONUS_MAX = 0.08f;
constexpr float IMPLAUSIBLE_JUMP_PENALTY = 0.05f;
constexpr float MAX_TRACKED_CENTER_DISTANCE_IN_TARGETS = 3.0f;
constexpr float MAX_TRACKED_SIZE_RATIO = 1.80f;
constexpr uint8_t TRACKING_RESET_AFTER_MISSES = 3;

void copy_geometry(const TargetObservation &source, TargetObservation &destination) {
  destination.top_left_px = source.top_left_px;
  destination.top_right_px = source.top_right_px;
  destination.bottom_right_px = source.bottom_right_px;
  destination.bottom_left_px = source.bottom_left_px;
}

float safe_ratio(float a, float b) {
  if (a <= 0.0f || b <= 0.0f) return 1000.0f;
  return std::max(a / b, b / a);
}
}

TargetDetector::TargetDetector()
    : candidate_finder_(),
      corner_refiner_(),
      subpixel_refiner_(),
      code_decoder_(),
      candidates_(),
      last_valid_observation_(),
      consecutive_misses_(0) {}

void TargetDetector::reset_tracking() {
  this->last_valid_observation_ = TargetObservation();
  this->consecutive_misses_ = 0;
  ESP_LOGD(TAG, "V5.7 tracking reset on viewport change");
}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) {
  TargetObservation best;
  float best_selection_score = -1000.0f;
  this->candidates_.count = 0;

  if (!this->candidate_finder_.find(frame, this->candidates_)) {
    this->update_tracking_(best);
    return best;
  }

  for (size_t index = 0; index < this->candidates_.count; ++index) {
    const TargetCandidate &candidate = this->candidates_.candidates[index];

    const TargetObservation coarse_observation = this->code_decoder_.decode(frame, candidate);
    TargetObservation candidate_best = coarse_observation;

    TargetCandidate refined_candidate = candidate;
    if (this->corner_refiner_.refine(frame, candidate, refined_candidate)) {
      const TargetObservation refined_observation =
          this->code_decoder_.decode(frame, refined_candidate);
      ESP_LOGD(TAG,
               "V6.1 candidate[%u] coarse=%.4f refined=%.4f refined_valid=%s",
               static_cast<unsigned>(index), coarse_observation.quality,
               refined_observation.quality, refined_observation.valid ? "YES" : "NO");

      if (refined_observation.quality > candidate_best.quality) {
        candidate_best = refined_observation;
      } else if (coarse_observation.valid && refined_observation.valid &&
                 coarse_observation.rotation_deg == refined_observation.rotation_deg) {
        copy_geometry(refined_observation, candidate_best);
      }

      // Le raffinement subpixel est volontairement une etape terminale :
      // il ne participe ni a la localisation grossiere ni a la decision de
      // validite de la cible. Il affine seulement les quatre coins d'une
      // cible deja reconnue par le chemin historique.
      if (refined_observation.valid) {
        TargetCandidate subpixel_candidate = refined_candidate;
        TargetSubpixelMetrics subpixel_metrics{};
        if (this->subpixel_refiner_.refine(frame, refined_candidate,
                                           subpixel_candidate,
                                           &subpixel_metrics)) {
          const TargetObservation subpixel_observation =
              this->code_decoder_.decode(frame, subpixel_candidate);

          ESP_LOGD(TAG,
                   "V5.7 candidate[%u] subpixel=%.4f valid=%s rms=%.3f max=%.3f grad=%.1f",
                   static_cast<unsigned>(index), subpixel_observation.quality,
                   subpixel_observation.valid ? "YES" : "NO",
                   subpixel_metrics.mean_rms_px,
                   subpixel_metrics.max_rms_px,
                   subpixel_metrics.mean_gradient);

          if (subpixel_observation.valid) {
            bool subpixel_geometry_used = false;
            if (!candidate_best.valid ||
                subpixel_observation.quality > candidate_best.quality) {
              candidate_best = subpixel_observation;
              subpixel_geometry_used = true;
            } else if (candidate_best.rotation_deg ==
                       subpixel_observation.rotation_deg) {
              // Conserver le score/choix du decodeur deja etabli, mais
              // remplacer la geometrie physique par les coins subpixel.
              copy_geometry(subpixel_observation, candidate_best);
              subpixel_geometry_used = true;
            }

            if (subpixel_geometry_used) {
              candidate_best.subpixel_refined = true;
              candidate_best.subpixel_rms_px = subpixel_metrics.mean_rms_px;
              candidate_best.subpixel_max_rms_px = subpixel_metrics.max_rms_px;
              candidate_best.subpixel_gradient = subpixel_metrics.mean_gradient;
              candidate_best.subpixel_width_px = subpixel_metrics.width_px;
              candidate_best.subpixel_height_px = subpixel_metrics.height_px;
              candidate_best.subpixel_width_sigma_px =
                  subpixel_metrics.width_sigma_px;
              candidate_best.subpixel_height_sigma_px =
                  subpixel_metrics.height_sigma_px;
              candidate_best.subpixel_width_gradient =
                  subpixel_metrics.width_gradient;
              candidate_best.subpixel_height_gradient =
                  subpixel_metrics.height_gradient;
            }
          }
        }
      }
    }

    const float selection_score = this->selection_score_(candidate_best);
    ESP_LOGD(TAG,
             "V5.7 candidate[%u] quality=%.4f valid=%s continuity=%.3f selection=%.4f",
             static_cast<unsigned>(index), candidate_best.quality,
             candidate_best.valid ? "YES" : "NO",
             this->continuity_score_(candidate_best), selection_score);

    if (selection_score > best_selection_score) {
      best = candidate_best;
      best_selection_score = selection_score;
    }
  }

  if (best.width_px <= 0.0f && this->candidates_.count > 0) {
    const TargetCandidate *fallback = &this->candidates_.candidates[0];
    for (size_t index = 1; index < this->candidates_.count; ++index) {
      if (this->candidates_.candidates[index].localization_score > fallback->localization_score) {
        fallback = &this->candidates_.candidates[index];
      }
    }

    best.valid = false;
    best.center_x_px = fallback->center_x;
    best.center_y_px = fallback->center_y;
    best.width_px = fallback->width;
    best.height_px = fallback->height;
    best.rotation_deg = 0.0f;
    best.quality = fallback->localization_score * 0.50f;
  }

  this->update_tracking_(best);
  return best;
}

float TargetDetector::continuity_score_(const TargetObservation &observation) const {
  if (!this->last_valid_observation_.valid || observation.width_px <= 0.0f || observation.height_px <= 0.0f) {
    return 0.0f;
  }

  const float reference_size = std::max(
      1.0f, 0.5f * (this->last_valid_observation_.width_px + this->last_valid_observation_.height_px));
  const float dx = observation.center_x_px - this->last_valid_observation_.center_x_px;
  const float dy = observation.center_y_px - this->last_valid_observation_.center_y_px;
  const float center_distance_targets = std::sqrt(dx * dx + dy * dy) / reference_size;

  const float width_ratio = safe_ratio(observation.width_px, this->last_valid_observation_.width_px);
  const float height_ratio = safe_ratio(observation.height_px, this->last_valid_observation_.height_px);
  const float worst_size_ratio = std::max(width_ratio, height_ratio);

  const float position_score = std::max(0.0f, 1.0f - center_distance_targets / 2.0f);
  const float size_score = std::max(0.0f, 1.0f - (worst_size_ratio - 1.0f) / 0.50f);
  return 0.65f * position_score + 0.35f * size_score;
}

float TargetDetector::selection_score_(const TargetObservation &observation) const {
  float score = observation.quality;
  if (!this->last_valid_observation_.valid || !observation.valid) {
    return score;
  }

  score += CONTINUITY_BONUS_MAX * this->continuity_score_(observation);

  const float reference_size = std::max(
      1.0f, 0.5f * (this->last_valid_observation_.width_px + this->last_valid_observation_.height_px));
  const float dx = observation.center_x_px - this->last_valid_observation_.center_x_px;
  const float dy = observation.center_y_px - this->last_valid_observation_.center_y_px;
  const float center_distance_targets = std::sqrt(dx * dx + dy * dy) / reference_size;
  const float worst_size_ratio = std::max(
      safe_ratio(observation.width_px, this->last_valid_observation_.width_px),
      safe_ratio(observation.height_px, this->last_valid_observation_.height_px));

  if (center_distance_targets > MAX_TRACKED_CENTER_DISTANCE_IN_TARGETS ||
      worst_size_ratio > MAX_TRACKED_SIZE_RATIO) {
    score -= IMPLAUSIBLE_JUMP_PENALTY;
  }

  return score;
}

void TargetDetector::update_tracking_(const TargetObservation &observation) {
  if (observation.valid) {
    this->last_valid_observation_ = observation;
    this->consecutive_misses_ = 0;
    return;
  }

  if (this->last_valid_observation_.valid) {
    this->consecutive_misses_++;
    if (this->consecutive_misses_ >= TRACKING_RESET_AFTER_MISSES) {
      ESP_LOGD(TAG, "V5.7 tracking reset after %u misses",
               static_cast<unsigned>(this->consecutive_misses_));
      this->last_valid_observation_ = TargetObservation();
      this->consecutive_misses_ = 0;
    }
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
