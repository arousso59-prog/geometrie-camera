#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegFilteredDiagnostic;
class TargetDetector;

class TargetDetectionService {
 public:
  TargetDetectionService(JpegFilteredDiagnostic *source, TargetDetector *detector);

  bool detect(bool high_precision = false,
              bool center_marker_only = false);
  void reset_tracking();

  bool ready() const;
  bool target_found() const;
  uint32_t detection_count() const;
  uint32_t source_process_count() const;
  uint32_t detection_ms() const;
  const TargetObservation &last_observation() const;

 private:
  JpegFilteredDiagnostic *source_;
  TargetDetector *detector_;
  TargetObservation last_observation_;
  uint32_t detection_count_;
  uint32_t source_process_count_;
  uint32_t detection_ms_;
  bool ready_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
