#pragma once

#include <cstdint>
#include <string>

#include "camera_viewport_controller.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

enum class TrackingUpdateResult : uint8_t {
  NONE,
  VIEWPORT_CHANGED,
  ERROR,
};

class TargetTrackingController {
 public:
  explicit TargetTrackingController(CameraViewportController *viewport_controller);

  bool start();
  void stop();

  bool set_enabled(bool enabled);
  bool set_lost_cycles(uint8_t cycles);
  bool set_recenter_threshold_pct(uint8_t percent);

  bool enabled() const;
  bool active() const;
  bool supported() const;
  uint8_t lost_cycles() const;
  uint8_t recenter_threshold_pct() const;
  uint8_t current_lost_count() const;
  uint32_t transition_count() const;
  bool target_locked() const;
  const std::string &last_error() const;

  TrackingUpdateResult update_after_detection(bool target_found,
                                              const TargetObservation &local_observation);
  TargetObservation to_reference(const TargetObservation &local_observation) const;
  uint16_t reference_width() const;
  uint16_t reference_height() const;
  const char *mode_text() const;

  CameraViewportController *viewport_controller();
  const CameraViewportController *viewport_controller() const;

 private:
  void clear_runtime_();

  CameraViewportController *viewport_controller_;
  bool enabled_;
  bool active_;
  uint8_t lost_cycles_;
  uint8_t recenter_threshold_pct_;
  uint8_t current_lost_count_;
  uint32_t transition_count_;
  bool target_locked_;
  bool precise_centered_;
  uint8_t precise_center_attempts_;
  uint8_t stage_recenter_attempts_;
  std::string last_error_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
