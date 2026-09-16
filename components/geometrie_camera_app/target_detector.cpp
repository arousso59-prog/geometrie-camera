#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetector::TargetDetector() : candidate_finder_(), code_decoder_(), candidates_() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) {
  TargetObservation best;
  this->candidates_.count = 0;

  if (!this->candidate_finder_.find(frame, this->candidates_)) {
    return best;
  }

  for (size_t index = 0; index < this->candidates_.count; ++index) {
    const TargetObservation observation = this->code_decoder_.decode(frame, this->candidates_.candidates[index]);
    if (observation.quality > best.quality) {
      best = observation;
    }
  }

  // Si aucun code n'a passe les gardes du decodeur, conserver au moins le
  // meilleur candidat de localisation dans la preview. Il reste explicitement
  // invalide et son score est volontairement borne sous le seuil metier.
  if (best.width_px <= 0.0f && this->candidates_.count > 0) {
    const TargetCandidate &candidate = this->candidates_.candidates[0];
    best.valid = false;
    best.center_x_px = candidate.center_x;
    best.center_y_px = candidate.center_y;
    best.width_px = candidate.width;
    best.height_px = candidate.height;
    best.rotation_deg = 0.0f;
    best.quality = candidate.localization_score * 0.50f;
  }

  return best;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
