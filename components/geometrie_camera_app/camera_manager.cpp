#include "camera_manager.h"

namespace esphome {
namespace geometrie_camera_app {

void CameraManager::setup() {
  // La camera n'est pas encore reliee ici. On attend la validation du pinout.
  this->ready_ = false;
}

void CameraManager::loop() {
  // Reserve pour la future gestion non bloquante des acquisitions.
}

bool CameraManager::ready() const {
  return this->ready_;
}

const CameraFrameInfo &CameraManager::last_frame_info() const {
  return this->last_frame_;
}

bool CameraManager::request_capture() {
  // Ne retourne jamais une fausse reussite tant que la camera n'est pas branchee.
  return false;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
