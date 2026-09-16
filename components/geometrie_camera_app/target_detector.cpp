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

constexpr uint16_t MIN_TARGET_SIZE_ABSOLUTE_PX = 14;

// V3 : la cible reelle observee represente quelques pourcents du petit cote.
// On exclut les tres grands carres qui ont produit le faux positif V2 a 190 px
// sur une image 1600x1200. A QSXGA, cette borne monte naturellement a 192 px.
constexpr uint16_t MIN_TARGET_SIZE_DIVISOR = 100;
constexpr uint16_t MAX_TARGET_SIZE_DIVISOR = 10;

// Balayage global grossier, puis raffinement local autour du meilleur candidat.
constexpr uint16_t MIN_SPATIAL_STEP_PX = 6;
constexpr uint16_t SPATIAL_STEP_DIVISOR = 8;
constexpr uint16_t MIN_SCALE_STEP_PX = 5;
constexpr uint16_t SCALE_STEP_DIVISOR = 10;
constexpr uint16_t REFINE_STEP_PX = 1;

// Le motif 7x7 porte maintenant l'essentiel de la validation. Le contraste est
// seulement un garde-fou car l'image JPEG corrigee est volontairement douce.
constexpr float MIN_ACCEPTED_SCORE = 0.86f;
constexpr float MIN_BORDER_BLACK_RATIO = 0.88f;
constexpr int MIN_CONTRAST = 10;
constexpr int MIN_PREFILTER_CONTRAST = 10;
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

  // Raffinement local fin autour du meilleur candidat du balayage global.
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

  // Toujours exposer le meilleur candidat pour le diagnostic, meme s'il est
  // refuse. Le champ valid/target_found reste la seule indication d'acceptation.
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
    uint16_t border_black = 0;
    uint16_t border_total = 0;

    for (uint8_t row = 0; row < 7; row++) {
      for (uint8_t column = 0; column < 7; column++) {
        const bool expected_black = this->expected_cell_(row, column, rotation) != 0;
        const bool measured_black = static_cast<int>(samples[row][column]) < threshold;
        if (expected_black == measured_black) {
          correct++;
        }

        if (row == 0 || row == 6 || column == 0 || column == 6) {
          border_total++;
          if (measured_black) {
            border_black++;
          }
        }
      }
    }

    if (border_total == 0) {
      continue;
    }

    const float border_ratio = static_cast<float>(border_black) / static_cast<float>(border_total);
    if (border_ratio < MIN_BORDER_BLACK_RATIO) {
      continue;
    }

    const float pattern_score = static_cast<float>(correct) / 49.0f;
    // Le cadre noir fait partie du code et sert de garde-fou anti-faux-positifs.
    // On donne encore 85 % du poids au motif complet et 15 % a la coherence du cadre.
    const float score = 0.85f * pattern_score + 0.15f * border_ratio;

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

  // V3 : un pixel unique est trop sensible au flou JPEG, au bruit residuel et aux
  // petits artefacts. On moyenne une zone 3x3 ou 5x5 qui reste largement a
  // l'interieur de la cellule 7x7.
  const uint16_t cell_size = std::max<uint16_t>(1, static_cast<uint16_t>(size / 7U));
  const uint16_t radius = std::min<uint16_t>(2, std::max<uint16_t>(1, static_cast<uint16_t>(cell_size / 5U)));

  const uint32_t min_x = sample_x > radius ? sample_x - radius : 0;
  const uint32_t min_y = sample_y > radius ? sample_y - radius : 0;
  const uint32_t max_x = std::min<uint32_t>(frame.width - 1U, sample_x + radius);
  const uint32_t max_y = std::min<uint32_t>(frame.height - 1U, sample_y + radius);

  uint32_t sum = 0;
  uint16_t count = 0;
  for (uint32_t py = min_y; py <= max_y; ++py) {
    const uint8_t *line = frame.data + py * frame.stride;
    for (uint32_t px = min_x; px <= max_x; ++px) {
      sum += line[px];
      count++;
    }
  }

  return count == 0 ? 0 : static_cast<uint8_t>(sum / count);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
