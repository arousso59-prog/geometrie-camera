#pragma once

#include <cstdint>

#include "target_candidate_finder.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetCodeDecoder {
 public:
  TargetCodeDecoder();

  TargetObservation decode(const GrayFrameView &frame, const TargetCandidate &candidate) const;

 private:
  TargetPoint project_(const TargetCandidate &candidate, float u, float v) const;
  uint8_t sample_point_(const GrayFrameView &frame, const TargetPoint &point) const;
  uint8_t sample_cell_(const GrayFrameView &frame, const TargetCandidate &candidate,
                       uint8_t row, uint8_t column) const;
  uint8_t expected_cell_(uint8_t row, uint8_t column, uint8_t rotation_quarters) const;
  float outside_mean_(const GrayFrameView &frame, const TargetCandidate &candidate) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
