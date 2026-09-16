#include "target_detector.h"

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_detector";

void copy_geometry(const TargetObservation &source, TargetObservation &destination) {
  destination.top_left_px = source.top_left_px;
  destination.top_right_px = source.top_right_px;
  destination.bottom_right_px = source.bottom_right_px;
  destination.bottom_left_px = source.bottom_left_px;
}
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

    // Le candidat brut reste le filet de securite pour le decodage.
    const TargetObservation coarse_observation = this->code_decoder_.decode(frame, candidate);
    TargetObservation candidate_best = coarse_observation;

    TargetCandidate refined_candidate = candidate;
    if (this->corner_refiner_.refine(frame, candidate, refined_candidate)) {
      const TargetObservation refined_observation = this->code_decoder_.decode(frame, refined_candidate);
      ESP_LOGD(TAG,
               "V5.4 candidate[%u] coarse=%.4f refined=%.4f refined_valid=%s",
               static_cast<unsigned>(index), coarse_observation.quality,
               refined_observation.quality, refined_observation.valid ? "YES" : "NO");

      if (refined_observation.quality > candidate_best.quality) {
        candidate_best = refined_observation;
      } else if (coarse_observation.valid && refined_observation.valid &&
                 coarse_observation.rotation_deg == refined_observation.rotation_deg) {
        // Le score du code peut etre legerement meilleur sur le quadrilatere
        // brut alors que les coins pleine resolution sont geometriquement plus
        // precis. Garder le score/validation du brut mais utiliser les coins
        // raffines pour la future distance et la pose.
        copy_geometry(refined_observation, candidate_best);
      }
    }

    if (candidate_best.quality > best.quality) {
      best = candidate_best;
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
