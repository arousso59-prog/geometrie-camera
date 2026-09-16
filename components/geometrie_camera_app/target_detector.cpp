#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetector::TargetDetector() : candidate_finder_(), code_decoder_() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) {
  TargetObservation best;
  TargetCandidateSet candidates;

  if (!this->candidate_finder_.find(frame, candidates)) {
    return best;
  }

  for (size_t index = 0; index < candidates.count; ++index) {
    const TargetObservation observation = this->code_decoder_.decode(frame, candidates.candidates[index]);
    if (observation.quality > best.quality) {
      best = observation;
    }
  }

  // Si aucun code n'a passe les gardes du decodeur, conserver au moins le
  // meilleur candidat de localisation dans la preview. Il reste explicitement
  // invalide et son score est volontairement borne sous le seuil metier.
  if (best.width_px <= 0.0f && candidates.count > 0) {
    const TargetCandidate &candidate = candidates.candidates[0];
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
