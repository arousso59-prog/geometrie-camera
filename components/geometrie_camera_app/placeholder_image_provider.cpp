#include "placeholder_image_provider.h"

#include "esphome/core/hal.h"
#include "placeholder_image.h"

namespace esphome {
namespace geometrie_camera_app {

PlaceholderImageProvider::PlaceholderImageProvider() : ready_(false) {}

PlaceholderImageProvider::~PlaceholderImageProvider() {}

void PlaceholderImageProvider::setup() {
  this->ready_ = true;
}

void PlaceholderImageProvider::loop() {}

bool PlaceholderImageProvider::ready() const {
  return this->ready_;
}

bool PlaceholderImageProvider::is_physical_camera() const {
  return false;
}

bool PlaceholderImageProvider::capture(ImageBufferView &image, uint32_t &timestamp_ms) {
  if (!this->ready_) {
    return false;
  }

  image.data = PLACEHOLDER_IMAGE_JPEG;
  image.size_bytes = PLACEHOLDER_IMAGE_JPEG_SIZE;
  image.width = PLACEHOLDER_IMAGE_WIDTH;
  image.height = PLACEHOLDER_IMAGE_HEIGHT;
  image.mime_type = "image/jpeg";
  timestamp_ms = millis();

  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
