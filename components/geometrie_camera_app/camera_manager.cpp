#include "camera_manager.h"

namespace esphome {
namespace geometrie_camera_app {

CameraManager::CameraManager(ImageProvider *image_provider)
    : image_provider_(image_provider), capture_count_(0), last_frame_(), current_image_() {}

void CameraManager::setup() {
  if (this->image_provider_ == nullptr) {
    return;
  }

  this->image_provider_->setup();
  this->request_capture();
}

void CameraManager::loop() {
  if (this->image_provider_ != nullptr) {
    this->image_provider_->loop();
  }
}

bool CameraManager::ready() const {
  return this->image_provider_ != nullptr && this->image_provider_->ready();
}

bool CameraManager::physical_camera_ready() const {
  return this->ready() && this->image_provider_->is_physical_camera();
}

bool CameraManager::placeholder_mode() const {
  return this->ready() && !this->image_provider_->is_physical_camera();
}

uint32_t CameraManager::capture_count() const {
  return this->capture_count_;
}

const CameraFrameInfo &CameraManager::last_frame_info() const {
  return this->last_frame_;
}

const ImageBufferView &CameraManager::current_image() const {
  return this->current_image_;
}

bool CameraManager::request_capture() {
  if (!this->ready()) {
    return false;
  }

  ImageBufferView image;
  uint32_t timestamp_ms = 0;

  if (!this->image_provider_->capture(image, timestamp_ms)) {
    return false;
  }

  this->current_image_ = image;
  this->capture_count_++;

  this->last_frame_.valid = true;
  this->last_frame_.width = image.width;
  this->last_frame_.height = image.height;
  this->last_frame_.size_bytes = image.size_bytes;
  this->last_frame_.timestamp_ms = timestamp_ms;

  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
