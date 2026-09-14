#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetector::TargetDetector() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) const {
  // V0 : l'architecture est prete mais l'algorithme de vision sera ajoute
  // apres validation de la camera et de la cible physique.
  return this->detect_placeholder_(frame);
}

TargetObservation TargetDetector::detect_placeholder_(const GrayFrameView &frame) const {
  TargetObservation result;

  if (frame.data == nullptr || frame.width == 0 || frame.height == 0) {
    return result;
  }

  // Placeholder volontaire : aucune fausse mesure n'est publiee.
  result.valid = false;
  result.center_x_px = static_cast<float>(frame.width) * 0.5f;
  result.center_y_px = static_cast<float>(frame.height) * 0.5f;
  result.quality = 0.0f;
  return result;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
