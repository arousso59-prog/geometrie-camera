#include "target_code_decoder.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_code_decoder";

constexpr uint8_t TARGET_GRID[7][7] = {
    {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 1, 1},
    {1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 1, 1, 1, 1},
    {1, 1, 0, 0, 1, 0, 1},
    {1, 1, 1, 1, 1, 1, 1},
};

constexpr float EXPANSION_FACTORS[] = {0.92f, 1.00f, 1.08f, 1.16f, 1.24f, 1.32f};

constexpr float MIN_ACCEPTED_SCORE = 0.82f;
constexpr float MIN_BORDER_BLACK_RATIO = 0.84f;
constexpr int MIN_CODE_CONTRAST = 8;
constexpr int MIN_OUTSIDE_BLACK_SEPARATION = 5;

float distance_between(const TargetPoint &a, const TargetPoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

TargetPoint expand_point(const TargetPoint &point, float center_x, float center_y, float factor) {
  TargetPoint expanded;
  expanded.x = center_x + (point.x - center_x) * factor;
  expanded.y = center_y + (point.y - center_y) * factor;
  return expanded;
}
}

TargetCodeDecoder::TargetCodeDecoder() {}

TargetObservation TargetCodeDecoder::decode(const GrayFrameView &frame,
                                            const TargetCandidate &candidate) const {
  TargetObservation best;
  float best_score = 0.0f;
  float best_pattern_score = 0.0f;
  float best_border_ratio = 0.0f;
  float best_outside_mean = 0.0f;
  float best_expansion = 0.0f;
  int best_contrast = 0;
  int best_black_mean = 0;
  uint8_t best_rotation = 0;

  if (frame.data == nullptr || frame.width == 0 || frame.height == 0 || frame.stride < frame.width) {
    return best;
  }

  for (float expansion : EXPANSION_FACTORS) {
    TargetCandidate adjusted = candidate;
    adjusted.top_left = expand_point(candidate.top_left, candidate.center_x, candidate.center_y, expansion);
    adjusted.top_right = expand_point(candidate.top_right, candidate.center_x, candidate.center_y, expansion);
    adjusted.bottom_right = expand_point(candidate.bottom_right, candidate.center_x, candidate.center_y, expansion);
    adjusted.bottom_left = expand_point(candidate.bottom_left, candidate.center_x, candidate.center_y, expansion);
    adjusted.center_x = candidate.center_x;
    adjusted.center_y = candidate.center_y;
    adjusted.width = 0.5f * (distance_between(adjusted.top_left, adjusted.top_right) +
                             distance_between(adjusted.bottom_left, adjusted.bottom_right));
    adjusted.height = 0.5f * (distance_between(adjusted.top_left, adjusted.bottom_left) +
                              distance_between(adjusted.top_right, adjusted.bottom_right));

    if (adjusted.width < 10.0f || adjusted.height < 10.0f) {
      continue;
    }

    uint8_t samples[7][7];
    for (uint8_t row = 0; row < 7; ++row) {
      for (uint8_t column = 0; column < 7; ++column) {
        samples[row][column] = this->sample_cell_(frame, adjusted, row, column);
      }
    }

    const float outside_mean = this->outside_mean_(frame, adjusted);

    for (uint8_t rotation = 0; rotation < 4; ++rotation) {
      uint32_t black_sum = 0;
      uint32_t white_sum = 0;
      uint16_t black_count = 0;
      uint16_t white_count = 0;

      for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t column = 0; column < 7; ++column) {
          if (this->expected_cell_(row, column, rotation) != 0) {
            black_sum += samples[row][column];
            black_count++;
          } else {
            white_sum += samples[row][column];
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
      if (contrast < MIN_CODE_CONTRAST) {
        continue;
      }

      if (outside_mean - static_cast<float>(black_mean) < MIN_OUTSIDE_BLACK_SEPARATION) {
        continue;
      }

      const int threshold = (black_mean + white_mean) / 2;
      uint16_t correct = 0;
      uint16_t border_black = 0;
      uint16_t border_total = 0;

      for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t column = 0; column < 7; ++column) {
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
      const float contrast_score = std::min(1.0f, static_cast<float>(contrast) / 45.0f);
      const float score = 0.86f * pattern_score + 0.10f * border_ratio + 0.04f * contrast_score;

      if (score > best_score) {
        best_score = score;
        best.valid = score >= MIN_ACCEPTED_SCORE;
        best.center_x_px = adjusted.center_x;
        best.center_y_px = adjusted.center_y;
        best.width_px = adjusted.width;
        best.height_px = adjusted.height;
        best.rotation_deg = static_cast<float>(rotation) * 90.0f;
        best.quality = score;

        // Les facteurs d'expansion servent uniquement a echantillonner le code.
        // Pour la mesure de pose, conserver la geometrie du candidat d'entree
        // (raffinee quand TargetCornerRefiner a reussi) afin que la taille
        // physique ne saute pas de 8 % quand le meilleur facteur change.
        best.top_left_px.x = candidate.top_left.x;
        best.top_left_px.y = candidate.top_left.y;
        best.top_right_px.x = candidate.top_right.x;
        best.top_right_px.y = candidate.top_right.y;
        best.bottom_right_px.x = candidate.bottom_right.x;
        best.bottom_right_px.y = candidate.bottom_right.y;
        best.bottom_left_px.x = candidate.bottom_left.x;
        best.bottom_left_px.y = candidate.bottom_left.y;

        best_pattern_score = pattern_score;
        best_border_ratio = border_ratio;
        best_outside_mean = outside_mean;
        best_expansion = expansion;
        best_contrast = contrast;
        best_black_mean = black_mean;
        best_rotation = rotation;
      }
    }
  }

  if (best_score > 0.0f) {
    ESP_LOGD(TAG,
             "V5.4 candidate center=(%.1f,%.1f) size=%.1fx%.1f rot=%u score=%.4f valid=%s "
             "pattern=%.4f border=%.4f contrast=%d outside=%.1f black=%d expansion=%.2f",
             candidate.center_x, candidate.center_y, best.width_px, best.height_px,
             static_cast<unsigned>(best_rotation) * 90U, best_score, best.valid ? "YES" : "NO",
             best_pattern_score, best_border_ratio, best_contrast, best_outside_mean,
             best_black_mean, best_expansion);
  }

  return best;
}

TargetPoint TargetCodeDecoder::project_(const TargetCandidate &candidate, float u, float v) const {
  const float x0 = candidate.top_left.x;
  const float y0 = candidate.top_left.y;
  const float x1 = candidate.top_right.x;
  const float y1 = candidate.top_right.y;
  const float x2 = candidate.bottom_right.x;
  const float y2 = candidate.bottom_right.y;
  const float x3 = candidate.bottom_left.x;
  const float y3 = candidate.bottom_left.y;

  const float sx = x0 - x1 + x2 - x3;
  const float sy = y0 - y1 + y2 - y3;
  const float dx1 = x1 - x2;
  const float dx2 = x3 - x2;
  const float dy1 = y1 - y2;
  const float dy2 = y3 - y2;

  float g = 0.0f;
  float h = 0.0f;
  const float denominator = dx1 * dy2 - dx2 * dy1;
  if ((std::fabs(sx) > 0.0001f || std::fabs(sy) > 0.0001f) && std::fabs(denominator) > 0.0001f) {
    g = (sx * dy2 - dx2 * sy) / denominator;
    h = (dx1 * sy - sx * dy1) / denominator;
  }

  const float a = x1 - x0 + g * x1;
  const float b = x3 - x0 + h * x3;
  const float c = x0;
  const float d = y1 - y0 + g * y1;
  const float e = y3 - y0 + h * y3;
  const float f = y0;
  const float w = g * u + h * v + 1.0f;

  TargetPoint point;
  if (std::fabs(w) > 0.0001f) {
    point.x = (a * u + b * v + c) / w;
    point.y = (d * u + e * v + f) / w;
    return point;
  }

  const float top_x = x0 + (x1 - x0) * u;
  const float top_y = y0 + (y1 - y0) * u;
  const float bottom_x = x3 + (x2 - x3) * u;
  const float bottom_y = y3 + (y2 - y3) * u;
  point.x = top_x + (bottom_x - top_x) * v;
  point.y = top_y + (bottom_y - top_y) * v;
  return point;
}

uint8_t TargetCodeDecoder::sample_point_(const GrayFrameView &frame, const TargetPoint &point) const {
  const int x = std::max(0, std::min<int>(frame.width - 1, static_cast<int>(std::lround(point.x))));
  const int y = std::max(0, std::min<int>(frame.height - 1, static_cast<int>(std::lround(point.y))));
  return frame.data[static_cast<size_t>(y) * frame.stride + x];
}

uint8_t TargetCodeDecoder::sample_cell_(const GrayFrameView &frame, const TargetCandidate &candidate,
                                        uint8_t row, uint8_t column) const {
  const float center_u = (static_cast<float>(column) + 0.5f) / 7.0f;
  const float center_v = (static_cast<float>(row) + 0.5f) / 7.0f;
  const float delta = 0.20f / 7.0f;

  const float offsets[5][2] = {
      {0.0f, 0.0f},
      {-delta, 0.0f},
      {delta, 0.0f},
      {0.0f, -delta},
      {0.0f, delta},
  };

  uint16_t sum = 0;
  for (const auto &offset : offsets) {
    const TargetPoint point = this->project_(candidate, center_u + offset[0], center_v + offset[1]);
    sum += this->sample_point_(frame, point);
  }
  return static_cast<uint8_t>(sum / 5U);
}

uint8_t TargetCodeDecoder::expected_cell_(uint8_t row, uint8_t column, uint8_t rotation_quarters) const {
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

float TargetCodeDecoder::outside_mean_(const GrayFrameView &frame,
                                       const TargetCandidate &candidate) const {
  constexpr float OUTSIDE = 0.10f;
  constexpr float POSITIONS[5] = {0.10f, 0.30f, 0.50f, 0.70f, 0.90f};
  uint32_t sum = 0;
  uint16_t count = 0;

  for (float position : POSITIONS) {
    sum += this->sample_point_(frame, this->project_(candidate, position, -OUTSIDE));
    sum += this->sample_point_(frame, this->project_(candidate, position, 1.0f + OUTSIDE));
    sum += this->sample_point_(frame, this->project_(candidate, -OUTSIDE, position));
    sum += this->sample_point_(frame, this->project_(candidate, 1.0f + OUTSIDE, position));
    count += 4;
  }

  return count == 0 ? 0.0f : static_cast<float>(sum) / static_cast<float>(count);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
