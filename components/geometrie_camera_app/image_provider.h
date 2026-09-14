#pragma once

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class ImageProvider {
 public:
  ImageProvider();
  virtual ~ImageProvider();

  virtual void setup() = 0;
  virtual void loop() = 0;
  virtual bool ready() const = 0;
  virtual bool is_physical_camera() const = 0;
  virtual bool capture(ImageBufferView &image, uint32_t &timestamp_ms) = 0;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
