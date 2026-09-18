#pragma once

#include "target_candidate_finder.h"
#include "target_code_decoder.h"
#include "target_corner_refiner.h"
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
  TargetSubpixelRefiner subpixel_refiner_;
  TargetCodeDecoder code_decoder_;
  TargetCandidateSet candidates_;

  TargetObservation last_valid_observation_;
  uint8_t consecutive_misses_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
