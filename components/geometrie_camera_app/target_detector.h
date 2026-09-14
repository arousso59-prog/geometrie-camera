#pragma once

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetDetector {
 public:
  TargetDetector();

  TargetObservation detect(const GrayFrameView &frame) const;

 private:
  TargetObservation detect_placeholder_(const GrayFrameView &frame) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
