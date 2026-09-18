#pragma once

#include "target_candidate_finder.h"
#include "target_board_model.h"
#include "target_code_decoder.h"
#include "target_corner_refiner.h"
#include "target_pattern_refiner.h"
#include "target_subpixel_refiner.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetDetector {
 public:
  TargetDetector();

  TargetObservation detect(const GrayFrameView &frame);
  void reset_tracking();

 private:
  float selection_score_(const TargetObservation &observation) const;
  float continuity_score_(const TargetObservation &observation) const;
  void update_tracking_(const TargetObservation &observation);

  TargetCandidateFinder candidate_finder_;
  TargetCornerRefiner corner_refiner_;
  TargetPatternRefiner pattern_refiner_;
  TargetSubpixelRefiner subpixel_refiner_;
  TargetCodeDecoder code_decoder_;
  TargetCandidateSet candidates_;

  static constexpr uint16_t MAX_MARKER_PATTERN_FEATURES = 128;
  static constexpr uint16_t MAX_BOARD_PATTERN_FEATURES = 384;
  TargetObservation marker_best_[3];
  PatternFeature marker_features_[3][MAX_MARKER_PATTERN_FEATURES];
  uint16_t marker_feature_counts_[3];
  PatternFeature board_features_[MAX_BOARD_PATTERN_FEATURES];

  TargetObservation last_valid_observation_;
  uint8_t consecutive_misses_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
