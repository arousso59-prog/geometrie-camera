#include "target_detector.h"

#include <cstring>

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
      pattern_refiner_(),
      subpixel_refiner_(),
      code_decoder_(),
      candidates_(),
      marker_best_{},
      marker_features_{},
      marker_feature_counts_{0, 0, 0},
      board_features_{},
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
  for (uint8_t m = 0; m < 3; ++m) {
    this->marker_best_[m] = TargetObservation();
    this->marker_feature_counts_[m] = 0;
  }

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

              candidate_best.subpixel_v5_width_px =
                  subpixel_metrics.v5_width_px;
              candidate_best.subpixel_v5_height_px =
                  subpixel_metrics.v5_height_px;
              candidate_best.subpixel_v5_width_sigma_px =
                  subpixel_metrics.v5_width_sigma_px;
              candidate_best.subpixel_v5_height_sigma_px =
                  subpixel_metrics.v5_height_sigma_px;
              candidate_best.subpixel_v6_width_px =
                  subpixel_metrics.v6_width_px;
              candidate_best.subpixel_v6_height_px =
                  subpixel_metrics.v6_height_px;
              candidate_best.subpixel_v6_width_sigma_px =
                  subpixel_metrics.v6_width_sigma_px;
              candidate_best.subpixel_v6_height_sigma_px =
                  subpixel_metrics.v6_height_sigma_px;

              candidate_best.subpixel_top_rms_px =
                  subpixel_metrics.top_rms_px;
              candidate_best.subpixel_right_rms_px =
                  subpixel_metrics.right_rms_px;
              candidate_best.subpixel_bottom_rms_px =
                  subpixel_metrics.bottom_rms_px;
              candidate_best.subpixel_left_rms_px =
                  subpixel_metrics.left_rms_px;
              candidate_best.subpixel_top_gradient =
                  subpixel_metrics.top_gradient;
              candidate_best.subpixel_right_gradient =
                  subpixel_metrics.right_gradient;
              candidate_best.subpixel_bottom_gradient =
                  subpixel_metrics.bottom_gradient;
              candidate_best.subpixel_left_gradient =
                  subpixel_metrics.left_gradient;

              candidate_best.subpixel_top_line.valid = true;
              candidate_best.subpixel_top_line.point.x =
                  subpixel_metrics.top_line_point.x;
              candidate_best.subpixel_top_line.point.y =
                  subpixel_metrics.top_line_point.y;
              candidate_best.subpixel_top_line.dx =
                  subpixel_metrics.top_line_dx;
              candidate_best.subpixel_top_line.dy =
                  subpixel_metrics.top_line_dy;

              candidate_best.subpixel_right_line.valid = true;
              candidate_best.subpixel_right_line.point.x =
                  subpixel_metrics.right_line_point.x;
              candidate_best.subpixel_right_line.point.y =
                  subpixel_metrics.right_line_point.y;
              candidate_best.subpixel_right_line.dx =
                  subpixel_metrics.right_line_dx;
              candidate_best.subpixel_right_line.dy =
                  subpixel_metrics.right_line_dy;

              candidate_best.subpixel_bottom_line.valid = true;
              candidate_best.subpixel_bottom_line.point.x =
                  subpixel_metrics.bottom_line_point.x;
              candidate_best.subpixel_bottom_line.point.y =
                  subpixel_metrics.bottom_line_point.y;
              candidate_best.subpixel_bottom_line.dx =
                  subpixel_metrics.bottom_line_dx;
              candidate_best.subpixel_bottom_line.dy =
                  subpixel_metrics.bottom_line_dy;

              candidate_best.subpixel_left_line.valid = true;
              candidate_best.subpixel_left_line.point.x =
                  subpixel_metrics.left_line_point.x;
              candidate_best.subpixel_left_line.point.y =
                  subpixel_metrics.left_line_point.y;
              candidate_best.subpixel_left_line.dx =
                  subpixel_metrics.left_line_dx;
              candidate_best.subpixel_left_line.dy =
                  subpixel_metrics.left_line_dy;

              // Pose V3 : exploiter les transitions internes du motif 7x7.
              // Ce raffinement ne remplace jamais les coins/droites utilisés
              // par la distance V6.1 ; il produit uniquement une homographie
              // canonique dédiée à l'orientation.
              TargetPatternMetrics pattern_metrics;
              const uint8_t rotation_quarters = static_cast<uint8_t>(
                  static_cast<int>(
                      std::lround(candidate_best.rotation_deg / 90.0f)) &
                  0x03);
              if (this->pattern_refiner_.refine(
                      frame, subpixel_candidate, candidate_best.marker_id,
                      rotation_quarters, pattern_metrics)) {
                candidate_best.pattern_refined = true;
                candidate_best.pattern_feature_count =
                    pattern_metrics.feature_count;
                candidate_best.pattern_inlier_count =
                    pattern_metrics.inlier_count;
                candidate_best.pattern_rms_px =
                    pattern_metrics.rms_px;
                candidate_best.pattern_max_residual_px =
                    pattern_metrics.max_residual_px;
                candidate_best.pattern_features =
                    pattern_metrics.features;
                candidate_best.pattern_features_count =
                    pattern_metrics.features_count;
                for (uint8_t h = 0; h < 9; ++h) {
                  candidate_best.pattern_homography[h] =
                      pattern_metrics.homography[h];
                }
              }
            }
          }
        }
      }
    }

    const int marker_index =
        target_r1_marker_index(candidate_best.marker_id);
    ESP_LOGD(TAG,
             "R1 candidate[%u] marker=%d quality=%.4f valid=%s subpixel=%s",
             static_cast<unsigned>(index), marker_index,
             candidate_best.quality,
             candidate_best.valid ? "YES" : "NO",
             candidate_best.subpixel_refined ? "YES" : "NO");

    if (candidate_best.valid && marker_index >= 0 &&
        (this->marker_best_[marker_index].valid == false ||
         candidate_best.quality > this->marker_best_[marker_index].quality)) {
      this->marker_best_[marker_index] = candidate_best;

      // PatternRefiner reutilise son buffer au candidat suivant : copier
      // immediatement les points de ce marqueur dans le stockage persistant
      // du detecteur.
      this->marker_feature_counts_[marker_index] = 0;
      if (candidate_best.pattern_refined &&
          candidate_best.pattern_features != nullptr) {
        const uint16_t count = std::min<uint16_t>(
            candidate_best.pattern_features_count,
            MAX_MARKER_PATTERN_FEATURES);
        for (uint16_t i = 0; i < count; ++i) {
          this->marker_features_[marker_index][i] =
              candidate_best.pattern_features[i];
        }
        this->marker_feature_counts_[marker_index] = count;
        this->marker_best_[marker_index].pattern_features =
            this->marker_features_[marker_index];
        this->marker_best_[marker_index].pattern_features_count = count;
      }
    }

    const float selection_score = this->selection_score_(candidate_best);
    if (selection_score > best_selection_score) {
      best = candidate_best;
      best_selection_score = selection_score;
    }
  }

  TargetObservation board;
  if (build_target_r1_observation(
          this->marker_best_,
          this->board_features_,
          MAX_BOARD_PATTERN_FEATURES,
          board)) {
    ESP_LOGI(TAG,
             "R1 board detectee: markers=%u mask=0x%02X complete=%s size=%.1fx%.1f px rms=%.3f",
             static_cast<unsigned>(board.board_marker_count),
             static_cast<unsigned>(board.board_marker_mask),
             board.board_complete ? "YES" : "NO",
             board.width_px, board.height_px,
             board.pattern_rms_px);
    best = board;
  } else if (best.valid) {
    // Un marqueur R1 isole ne constitue plus une cible valide. On conserve
    // sa geometrie comme information de recuperation, mais l'ancienne cible
    // 50x50 seule ne peut plus verrouiller le systeme.
    best.valid = false;
    best.board_complete = false;
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
