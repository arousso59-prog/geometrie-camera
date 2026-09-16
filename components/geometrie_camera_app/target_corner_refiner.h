#pragma once

#include <cstdint>

#include "target_candidate_finder.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetCornerRefiner {
 public:
  TargetCornerRefiner();

  bool refine(const GrayFrameView &frame, const TargetCandidate &input, TargetCandidate &output) const;

 private:
  bool refine_corner_(const GrayFrameView &frame, const TargetPoint &origin,
                      const TargetPoint &neighbor_a, const TargetPoint &neighbor_b,
                      uint16_t radius, TargetPoint &refined) const;
  float corner_score_(const GrayFrameView &frame, const TargetPoint &point,
                      const TargetPoint &origin, const TargetPoint &neighbor_a,
                      const TargetPoint &neighbor_b) const;
  uint8_t sample_(const GrayFrameView &frame, float x, float y) const;
  bool geometry_valid_(const TargetCandidate &candidate) const;
  void update_geometry_(TargetCandidate &candidate) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
