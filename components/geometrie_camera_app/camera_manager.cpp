#include "camera_manager.h"

#include "esphome/core/hal.h"
#include "placeholder_image.h"

namespace esphome {
namespace geometrie_camera_app {

void CameraManager::setup() {
  // V0 : le service camera est disponible en mode bouchon.
  // La vraie camera physique reste explicitement marquee non prete.
  this->ready_ = true;
  this->physical_camera_ready_ = false;
  this->placeholder_mode_ = true;

  this->last_frame_.valid = true;
  this->last_frame_.width = PLACEHOLDER_IMAGE_WIDTH;
  this->last_frame_.height = PLACEHOLDER_IMAGE_HEIGHT;
  this->last_frame_.size_bytes = PLACEHOLDER_IMAGE_JPEG_SIZE;
  this->last_frame_.timestamp_ms = 0;
}

void CameraManager::loop() {
  // Reserve pour la future gestion non bloquante des acquisitions OV3660.
}

bool CameraManager::ready() const {
  return this->ready_;
}

bool CameraManager::physical_camera_ready() const {
  return this->physical_camera_ready_;
}

bool CameraManager::placeholder_mode() const {
  return this->placeholder_mode_;
}

uint32_t CameraManager::capture_count() const {
  return this->capture_count_;
}

const CameraFrameInfo &CameraManager::last_frame_info() const {
  return this->last_frame_;
}

bool CameraManager::request_capture() {
  if (!this->ready_) {
    return false;
  }

  // Capture bouchon : on conserve les memes metadonnees d'image et on met
  // seulement a jour l'identifiant logique et l'horodatage de la capture.
  this->capture_count_++;
  this->last_frame_.valid = true;
  this->last_frame_.width = PLACEHOLDER_IMAGE_WIDTH;
  this->last_frame_.height = PLACEHOLDER_IMAGE_HEIGHT;
  this->last_frame_.size_bytes = PLACEHOLDER_IMAGE_JPEG_SIZE;
  this->last_frame_.timestamp_ms = millis();

  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
