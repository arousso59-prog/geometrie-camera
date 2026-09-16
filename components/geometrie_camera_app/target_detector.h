#pragma once

#include "target_candidate_finder.h"
#include "target_code_decoder.h"
#include "target_corner_refiner.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetDetector {
 public:
  TargetDetector();

  TargetObservation detect(const GrayFrameView &frame);

 private:
  TargetCandidateFinder candidate_finder_;
  TargetCornerRefiner corner_refiner_;
  TargetCodeDecoder code_decoder_;
  TargetCandidateSet candidates_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
