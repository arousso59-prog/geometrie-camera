#include "target_detector.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr uint8_t TARGET_GRID[7][7] = {
    {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 1, 1},
    {1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 1, 1, 1, 1},
    {1, 1, 0, 0, 1, 0, 1},
    {1, 1, 1, 1, 1, 1, 1},
};

// Le motif contient 7 cellules par cote. En dessous de 14 px, chaque cellule
// aurait moins de 2 px et la reconnaissance devient trop fragile.
constexpr uint16_t MIN_TARGET_SIZE_ABSOLUTE_PX = 14;

// La recherche initiale reste volontairement large, mais on n'explore plus les
// carrés gigantesques qui ne correspondent pas a une cible de geometrie.
constexpr uint16_t MIN_TARGET_SIZE_DIVISOR = 100;
constexpr uint16_t MAX_TARGET_SIZE_DIVISOR = 4;

// V2 : balayage plus grossier que la premiere version, suivi d'un raffinement
// local autour du meilleur candidat. Cela reduit fortement le nombre de positions
// testees sans sacrifier la precision finale.
constexpr uint16_t MIN_SPATIAL_STEP_PX = 6;
constexpr uint16_t SPATIAL_STEP_DIVISOR = 8;
constexpr uint16_t MIN_SCALE_STEP_PX = 6;
constexpr uint16_t SCALE_STEP_DIVISOR = 8;
constexpr uint16_t REFINE_STEP_PX = 2;

// La vraie cible observee apres decodage/correction est nettement moins contrastee
// que les images synthetiques utilisees au depart. Le code 7x7 doit donc porter
// l'essentiel du score ; le contraste est un garde-fou, pas un multiplicateur fort.
constexpr float MIN_ACCEPTED_SCORE = 0.78f;
constexpr int MIN_CONTRAST = 12;
constexpr int MIN_PREFILTER_CONTRAST = 12;
constexpr uint8_t MIN_PREFILTER_BRIGHT_CELLS = 3;
}

TargetDetector::TargetDetector() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) const {
  TargetObservation best;

  if (frame.data == nullptr || frame.stride < frame.width) {
    return best;
  }

  const uint16_t short_side = std::min<uint16_t>(frame.width, frame.height);
  if (short_side < MIN_TARGET_SIZE_ABSOLUTE_PX) {
    return best;
  }

  const uint16_t relative_minimum =
      std::max<uint16_t>(1, static_cast<uint16_t>(short_side / MIN_TARGET_SIZE_DIVISOR));
  const uint16_t minimum_size = std::max<uint16_t>(MIN_TARGET_SIZE_ABSOLUTE_PX, relative_minimum);
  const uint16_t maximum_size =
      std::max<uint16_t>(minimum_size, static_cast<uint16_t>(short_side / MAX_TARGET_SIZE_DIVISOR));

  float best_score = 0.0f;
  uint16_t best_x = 0;
  uint16_t best_y = 0;
  uint16_t best_size = 0;
  uint8_t best_rotation = 0;

  uint16_t size = minimum_size;
  while (size <= maximum_size) {
    const uint16_t spatial_step =
        std::max<uint16_t>(MIN_SPATIAL_STEP_PX, static_cast<uint16_t>(size / SPATIAL_STEP_DIVISOR));

    for (uint16_t y = 0; static_cast<uint32_t>(y) + size <= frame.height; y += spatial_step) {
      for (uint16_t x = 0; static_cast<uint32_t>(x) + size <= frame.width; x += spatial_step) {
        if (!this->passes_prefilter_(frame, x, y, size)) {
          continue;
        }

        uint8_t candidate_rotation = 0;
        const float score = this->score_candidate_(frame, x, y, size, candidate_rotation);
        if (score > best_score) {
          best_score = score;
          best_x = x;
          best_y = y;
          best_size = size;
          best_rotation = candidate_rotation;
        }
      }
    }

    const uint16_t scale_step =
        std::max<uint16_t>(MIN_SCALE_STEP_PX, static_cast<uint16_t>(size / SCALE_STEP_DIVISOR));
    if (static_cast<uint32_t>(size) + scale_step > maximum_size) {
      break;
    }
    size = static_cast<uint16_t>(size + scale_step);
  }

  if (size != maximum_size && maximum_size >= minimum_size) {
    const uint16_t spatial_step =
        std::max<uint16_t>(MIN_SPATIAL_STEP_PX, static_cast<uint16_t>(maximum_size / SPATIAL_STEP_DIVISOR));
    for (uint16_t y = 0; static_cast<uint32_t>(y) + maximum_size <= frame.height; y += spatial_step) {
      for (uint16_t x = 0; static_cast<uint32_t>(x) + maximum_size <= frame.width; x += spatial_step) {
        if (!this->passes_prefilter_(frame, x, y, maximum_size)) {
          continue;
        }

        uint8_t candidate_rotation = 0;
        const float score = this->score_candidate_(frame, x, y, maximum_size, candidate_rotation);
        if (score > best_score) {
          best_score = score;
          best_x = x;
          best_y = y;
          best_size = maximum_size;
          best_rotation = candidate_rotation;
        }
      }
    }
  }

  // Raffinement local : une fois une zone prometteuse trouvee, on reteste autour
  // de sa position et de sa taille avec un pas de 2 px. Cela permet au balayage
  // global de rester rapide tout en retrouvant correctement les centres de cellules.
  if (best_size != 0) {
    const uint16_t coarse_spatial_step =
        std::max<uint16_t>(MIN_SPATIAL_STEP_PX, static_cast<uint16_t>(best_size / SPATIAL_STEP_DIVISOR));
    const uint16_t coarse_scale_step =
        std::max<uint16_t>(MIN_SCALE_STEP_PX, static_cast<uint16_t>(best_size / SCALE_STEP_DIVISOR));

    const uint16_t refine_min_size =
        best_size > coarse_scale_step ? std::max<uint16_t>(minimum_size, best_size - coarse_scale_step)
                                      : minimum_size;
    const uint16_t refine_max_size =
        std::min<uint16_t>(maximum_size, static_cast<uint16_t>(best_size + coarse_scale_step));
    const uint16_t refine_min_x = best_x > coarse_spatial_step ? best_x - coarse_spatial_step : 0;
    const uint16_t refine_min_y = best_y > coarse_spatial_step ? best_y - coarse_spatial_step : 0;
    const uint32_t refine_max_x = static_cast<uint32_t>(best_x) + coarse_spatial_step;
    const uint32_t refine_max_y = static_cast<uint32_t>(best_y) + coarse_spatial_step;

    for (uint16_t refine_size = refine_min_size; refine_size <= refine_max_size;) {
      for (uint32_t y = refine_min_y; y <= refine_max_y && y + refine_size <= frame.height;
           y += REFINE_STEP_PX) {
        for (uint32_t x = refine_min_x; x <= refine_max_x && x + refine_size <= frame.width;
             x += REFINE_STEP_PX) {
          if (!this->passes_prefilter_(frame, static_cast<uint16_t>(x), static_cast<uint16_t>(y), refine_size)) {
            continue;
          }

          uint8_t candidate_rotation = 0;
          const float score = this->score_candidate_(frame, static_cast<uint16_t>(x),
                                                     static_cast<uint16_t>(y), refine_size,
                                                     candidate_rotation);
          if (score > best_score) {
            best_score = score;
            best_x = static_cast<uint16_t>(x);
            best_y = static_cast<uint16_t>(y);
            best_size = refine_size;
            best_rotation = candidate_rotation;
          }
        }
      }

      if (static_cast<uint32_t>(refine_size) + REFINE_STEP_PX > refine_max_size) {
        break;
      }
      refine_size = static_cast<uint16_t>(refine_size + REFINE_STEP_PX);
    }
  }

  if (best_size == 0) {
    return best;
  }

  // Toujours exposer le meilleur candidat pour le diagnostic, meme s'il reste
  // sous le seuil d'acceptation. target_found/valid indique seul si la cible est
  // consideree comme reconnue.
  best.valid = best_score >= MIN_ACCEPTED_SCORE;
  best.center_x_px = static_cast<float>(best_x) + static_cast<float>(best_size) * 0.5f;
  best.center_y_px = static_cast<float>(best_y) + static_cast<float>(best_size) * 0.5f;
  best.width_px = static_cast<float>(best_size);
  best.height_px = static_cast<float>(best_size);
  best.rotation_deg = static_cast<float>(best_rotation) * 90.0f;
  best.quality = best_score;
  return best;
}

bool TargetDetector::passes_prefilter_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size) const {
  uint32_t border_sum = 0;
  uint8_t border_count = 0;

  for (uint8_t column = 0; column < 7; column += 2) {
    border_sum += this->sample_cell_(frame, x, y, size, 0, column);
    border_sum += this->sample_cell_(frame, x, y, size, 6, column);
    border_count += 2;
  }
  for (uint8_t row = 2; row <= 4; row += 2) {
    border_sum += this->sample_cell_(frame, x, y, size, row, 0);
    border_sum += this->sample_cell_(frame, x, y, size, row, 6);
    border_count += 2;
  }

  if (border_count == 0) {
    return false;
  }

  const int border_mean = static_cast<int>(border_sum / border_count);
  uint8_t bright_inner_cells = 0;

  for (uint8_t row = 1; row <= 5; row++) {
    for (uint8_t column = 1; column <= 5; column++) {
      const int value = static_cast<int>(this->sample_cell_(frame, x, y, size, row, column));
      if (value - border_mean >= MIN_PREFILTER_CONTRAST) {
        bright_inner_cells++;
        if (bright_inner_cells >= MIN_PREFILTER_BRIGHT_CELLS) {
          return true;
        }
      }
    }
  }

  return false;
}

float TargetDetector::score_candidate_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size,
                                       uint8_t &best_rotation_quarters) const {
  uint8_t samples[7][7];
  for (uint8_t row = 0; row < 7; row++) {
    for (uint8_t column = 0; column < 7; column++) {
      samples[row][column] = this->sample_cell_(frame, x, y, size, row, column);
    }
  }

  float best_score = 0.0f;
  best_rotation_quarters = 0;

  for (uint8_t rotation = 0; rotation < 4; rotation++) {
    uint32_t black_sum = 0;
    uint32_t white_sum = 0;
    uint16_t black_count = 0;
    uint16_t white_count = 0;

    for (uint8_t row = 0; row < 7; row++) {
      for (uint8_t column = 0; column < 7; column++) {
        const uint8_t value = samples[row][column];
        if (this->expected_cell_(row, column, rotation) != 0) {
          black_sum += value;
          black_count++;
        } else {
          white_sum += value;
          white_count++;
        }
      }
    }

    if (black_count == 0 || white_count == 0) {
      continue;
    }

    const int black_mean = static_cast<int>(black_sum / black_count);
    const int white_mean = static_cast<int>(white_sum / white_count);
    const int contrast = white_mean - black_mean;
    if (contrast < MIN_CONTRAST) {
      continue;
    }

    const int threshold = (white_mean + black_mean) / 2;
    uint16_t correct = 0;

    for (uint8_t row = 0; row < 7; row++) {
      for (uint8_t column = 0; column < 7; column++) {
        const bool expected_black = this->expected_cell_(row, column, rotation) != 0;
        const bool measured_black = static_cast<int>(samples[row][column]) < threshold;
        if (expected_black == measured_black) {
          correct++;
        }
      }
    }

    // Le contraste a deja ete valide ci-dessus. On ne le penalise plus une
    // seconde fois : le score exprime directement la proportion du code 7x7
    // correctement classee.
    const float score = static_cast<float>(correct) / 49.0f;

    if (score > best_score) {
      best_score = score;
      best_rotation_quarters = rotation;
    }
  }

  return best_score;
}

uint8_t TargetDetector::expected_cell_(uint8_t row, uint8_t column, uint8_t rotation_quarters) const {
  rotation_quarters &= 0x03;

  switch (rotation_quarters) {
    case 1:
      return TARGET_GRID[6 - column][row];
    case 2:
      return TARGET_GRID[6 - row][6 - column];
    case 3:
      return TARGET_GRID[column][6 - row];
    default:
      return TARGET_GRID[row][column];
  }
}

uint8_t TargetDetector::sample_cell_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size,
                                     uint8_t row, uint8_t column) const {
  const uint32_t sample_x = static_cast<uint32_t>(x) +
                            (static_cast<uint32_t>(2U * column + 1U) * size) / 14U;
  const uint32_t sample_y = static_cast<uint32_t>(y) +
                            (static_cast<uint32_t>(2U * row + 1U) * size) / 14U;

  const uint32_t clamped_x = std::min<uint32_t>(sample_x, frame.width - 1U);
  const uint32_t clamped_y = std::min<uint32_t>(sample_y, frame.height - 1U);
  return frame.data[clamped_y * frame.stride + clamped_x];
}

}  // namespace geometrie_camera_app
}  // namespace esphome
