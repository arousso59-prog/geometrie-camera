#pragma once

#include <cstdint>

#include "image_provider.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraManager {
 public:
  explicit CameraManager(ImageProvider *image_provider);

  void setup();
  void loop();

  bool ready() const;
  bool physical_camera_ready() const;
  bool placeholder_mode() const;
  uint32_t capture_count() const;
  const CameraFrameInfo &last_frame_info() const;
  const ImageBufferView &current_image() const;

  bool request_capture();

 private:
  ImageProvider *image_provider_;
  uint32_t capture_count_;
  CameraFrameInfo last_frame_;
  ImageBufferView current_image_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
