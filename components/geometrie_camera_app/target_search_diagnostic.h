#pragma once

#include <cstdint>
#include <memory>

#include "esphome/components/camera/camera.h"
#include "types.h"

namespace esphome {
namespace esp32_camera {
class ESP32Camera;
}
namespace geometrie_camera_app {

class GrayscaleDiagnostic;
class TargetDetector;

class TargetSearchDiagnostic : public camera::CameraListener {
 public:
  TargetSearchDiagnostic(TargetDetector *detector, GrayscaleDiagnostic *visualization);

  void set_camera(esp32_camera::ESP32Camera *camera);
  bool request_search();
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  bool ready() const;
  bool search_pending() const;
  bool target_found() const;
  uint32_t search_count() const;
  uint32_t request_started_ms() const;
  uint32_t frame_received_ms() const;
  uint32_t acquisition_ms() const;
  uint32_t detection_ms() const;
  uint32_t visualization_ms() const;
  uint32_t total_cycle_ms() const;
  const TargetObservation &last_observation() const;

 private:
  esp32_camera::ESP32Camera *camera_;
  TargetDetector *detector_;
  GrayscaleDiagnostic *visualization_;
  TargetObservation last_observation_;
  bool ready_;
  bool search_pending_;
  uint32_t search_count_;
  uint32_t request_started_ms_;
  uint32_t frame_received_ms_;
  uint32_t acquisition_ms_;
  uint32_t detection_ms_;
  uint32_t visualization_ms_;
  uint32_t total_cycle_ms_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
