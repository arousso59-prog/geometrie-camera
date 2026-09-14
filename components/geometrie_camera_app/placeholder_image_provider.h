#pragma once

#include "image_provider.h"

namespace esphome {
namespace geometrie_camera_app {

class PlaceholderImageProvider : public ImageProvider {
 public:
  PlaceholderImageProvider();
  ~PlaceholderImageProvider() override;

  void setup() override;
  void loop() override;
  bool ready() const override;
  bool is_physical_camera() const override;
  bool capture(ImageBufferView &image, uint32_t &timestamp_ms) override;

 private:
  bool ready_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
