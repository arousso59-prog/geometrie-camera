#include "target_detector.h"

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_detector";
}

TargetDetector::TargetDetector()
    : candidate_finder_(), corner_refiner_(), code_decoder_(), candidates_() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) {
  TargetObservation best;
  this->candidates_.count = 0;

  if (!this->candidate_finder_.find(frame, this->candidates_)) {
    return best;
  }

  for (size_t index = 0; index < this->candidates_.count; ++index) {
    const TargetCandidate &candidate = this->candidates_.candidates[index];

    // Toujours conserver le decodage du candidat brut comme filet de securite.
    // Le raffinement pleine resolution ne doit jamais rendre une detection
    // auparavant valide moins robuste.
    const TargetObservation coarse_observation = this->code_decoder_.decode(frame, candidate);
    if (coarse_observation.quality > best.quality) {
      best = coarse_observation;
    }

    TargetCandidate refined_candidate = candidate;
    if (this->corner_refiner_.refine(frame, candidate, refined_candidate)) {
      const TargetObservation refined_observation = this->code_decoder_.decode(frame, refined_candidate);
      ESP_LOGD(TAG,
               "V5.4 candidate[%u] coarse=%.4f refined=%.4f refined_valid=%s",
               static_cast<unsigned>(index), coarse_observation.quality,
               refined_observation.quality, refined_observation.valid ? "YES" : "NO");
      if (refined_observation.quality > best.quality) {
        best = refined_observation;
      }
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
