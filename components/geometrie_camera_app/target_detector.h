#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetDetector {
 public:
  TargetDetector();

  TargetObservation detect(const GrayFrameView &frame) const;

 private:
  bool passes_prefilter_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size) const;
  float score_candidate_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size,
                         uint8_t &best_rotation_quarters) const;
  uint8_t expected_cell_(uint8_t row, uint8_t column, uint8_t rotation_quarters) const;
  uint8_t sample_cell_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size,
                       uint8_t row, uint8_t column) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
