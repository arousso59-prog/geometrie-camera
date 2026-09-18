#pragma once

#include <cstdint>

#include "target_candidate_finder.h"

namespace esphome {
namespace geometrie_camera_app {

struct TargetPatternMetrics {
  TargetPatternMetrics();

  bool valid;
  uint16_t feature_count;
  uint16_t inlier_count;
  float rms_px;
  float max_residual_px;
  float homography[9];
};

class TargetPatternRefiner {
 public:
  TargetPatternRefiner();

  bool refine(const GrayFrameView &frame,
              const TargetCandidate &candidate,
              uint8_t rotation_quarters,
              TargetPatternMetrics &metrics) const;

 private:
  struct Feature {
    float u;
    float v;
    float x;
    float y;
    float strength;
    float residual;
  };

  static constexpr uint16_t MAX_FEATURES = 128;

  TargetPoint project_candidate_(const TargetCandidate &candidate,
                                 float u, float v) const;
  float sample_bilinear_(const GrayFrameView &frame,
                         float x, float y) const;
  uint8_t expected_cell_(uint8_t row, uint8_t column,
                         uint8_t rotation_quarters) const;
  void canonical_uv_(float observed_u, float observed_v,
                     uint8_t rotation_quarters,
                     float &canonical_u, float &canonical_v) const;
  bool find_transition_(const GrayFrameView &frame,
                        const TargetCandidate &candidate,
                        float observed_u, float observed_v,
                        bool along_u,
                        int expected_sign,
                        uint8_t rotation_quarters,
                        Feature &feature) const;
  bool fit_homography_(Feature *features, uint16_t count,
                       float homography[9],
                       uint16_t &inlier_count,
                       float &rms_px,
                       float &max_residual_px) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
