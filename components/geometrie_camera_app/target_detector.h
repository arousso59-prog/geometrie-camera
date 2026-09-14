#pragma once

#include <cstddef>
#include <cstdint>

#include "geometry_measurement.h"

namespace esphome {
namespace geometrie_camera_app {

struct GrayFrameView {
  const uint8_t *data{nullptr};
  uint16_t width{0};
  uint16_t height{0};
  size_t stride{0};
};

class TargetDetector {
 public:
  TargetObservation detect(const GrayFrameView &frame) const;

 private:
  TargetObservation detect_placeholder_(const GrayFrameView &frame) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
