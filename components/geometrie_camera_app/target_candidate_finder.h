#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

struct TargetPoint {
  TargetPoint();

  float x;
  float y;
};

struct TargetCandidate {
  TargetCandidate();

  TargetPoint top_left;
  TargetPoint top_right;
  TargetPoint bottom_right;
  TargetPoint bottom_left;
  float center_x;
  float center_y;
  float width;
  float height;
  float localization_score;
};

struct TargetCandidateSet {
  TargetCandidateSet();

  static constexpr size_t MAX_CANDIDATES = 24;
  TargetCandidate candidates[MAX_CANDIDATES];
  size_t count;
};

class TargetCandidateFinder {
 public:
  TargetCandidateFinder();

  bool find(const GrayFrameView &frame, TargetCandidateSet &result);

 private:
  bool build_reduced_image_(const GrayFrameView &frame, uint16_t scale);
  void build_local_threshold_map_();
  void collect_components_(const GrayFrameView &frame, uint16_t scale, TargetCandidateSet &result);
  void insert_candidate_(TargetCandidateSet &result, const TargetCandidate &candidate) const;

  uint16_t reduced_width_;
  uint16_t reduced_height_;
  uint16_t tile_columns_;
  uint16_t tile_rows_;
  std::vector<uint8_t> reduced_;
  std::vector<uint16_t> tile_means_;
  std::vector<uint32_t> queue_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
