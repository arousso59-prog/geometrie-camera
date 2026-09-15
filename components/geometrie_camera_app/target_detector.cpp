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

constexpr uint16_t MIN_TARGET_SIZE_PX = 28;
constexpr uint16_t MAX_TARGET_SIZE_PX = 168;
constexpr uint16_t TARGET_SIZE_STEP_PX = 7;
constexpr float MIN_ACCEPTED_SCORE = 0.78f;
constexpr int MIN_CONTRAST = 40;
}

TargetDetector::TargetDetector() {}

TargetObservation TargetDetector::detect(const GrayFrameView &frame) const {
  TargetObservation best;

  if (frame.data == nullptr || frame.width < MIN_TARGET_SIZE_PX || frame.height < MIN_TARGET_SIZE_PX ||
      frame.stride < frame.width) {
    return best;
  }

  float best_score = 0.0f;
  uint16_t best_x = 0;
  uint16_t best_y = 0;
  uint16_t best_size = 0;
  uint8_t best_rotation = 0;

  const uint16_t maximum_size = std::min<uint16_t>(
      MAX_TARGET_SIZE_PX, std::min<uint16_t>(frame.width, frame.height));

  for (uint16_t size = MIN_TARGET_SIZE_PX; size <= maximum_size; size += TARGET_SIZE_STEP_PX) {
    const uint16_t step = std::max<uint16_t>(4, size / 12U);

    for (uint16_t y = 0; static_cast<uint32_t>(y) + size <= frame.height; y += step) {
      for (uint16_t x = 0; static_cast<uint32_t>(x) + size <= frame.width; x += step) {
        for (uint8_t rotation = 0; rotation < 4; rotation++) {
          const float score = this->score_candidate_(frame, x, y, size, rotation);
          if (score > best_score) {
            best_score = score;
            best_x = x;
            best_y = y;
            best_size = size;
            best_rotation = rotation;
          }
        }
      }
    }
  }

  if (best_score < MIN_ACCEPTED_SCORE || best_size == 0) {
    return best;
  }

  best.valid = true;
  best.center_x_px = static_cast<float>(best_x) + static_cast<float>(best_size) * 0.5f;
  best.center_y_px = static_cast<float>(best_y) + static_cast<float>(best_size) * 0.5f;
  best.width_px = static_cast<float>(best_size);
  best.height_px = static_cast<float>(best_size);
  best.rotation_deg = static_cast<float>(best_rotation) * 90.0f;
  best.quality = best_score;
  return best;
}

float TargetDetector::score_candidate_(const GrayFrameView &frame, uint16_t x, uint16_t y, uint16_t size,
                                       uint8_t rotation_quarters) const {
  uint32_t black_sum = 0;
  uint32_t white_sum = 0;
  uint16_t black_count = 0;
  uint16_t white_count = 0;

  for (uint8_t row = 0; row < 7; row++) {
    for (uint8_t column = 0; column < 7; column++) {
      const uint8_t value = this->sample_cell_(frame, x, y, size, row, column);
      if (this->expected_cell_(row, column, rotation_quarters) != 0) {
        black_sum += value;
        black_count++;
      } else {
        white_sum += value;
        white_count++;
      }
    }
  }

  if (black_count == 0 || white_count == 0) {
    return 0.0f;
  }

  const int black_mean = static_cast<int>(black_sum / black_count);
  const int white_mean = static_cast<int>(white_sum / white_count);
  const int contrast = white_mean - black_mean;
  if (contrast < MIN_CONTRAST) {
    return 0.0f;
  }

  const int threshold = (white_mean + black_mean) / 2;
  uint16_t correct = 0;

  for (uint8_t row = 0; row < 7; row++) {
    for (uint8_t column = 0; column < 7; column++) {
      const uint8_t value = this->sample_cell_(frame, x, y, size, row, column);
      const bool expected_black = this->expected_cell_(row, column, rotation_quarters) != 0;
      const bool measured_black = static_cast<int>(value) < threshold;
      if (expected_black == measured_black) {
        correct++;
      }
    }
  }

  const float pattern_score = static_cast<float>(correct) / 49.0f;
  const float contrast_score = std::min(1.0f, static_cast<float>(contrast) / 110.0f);
  return pattern_score * (0.75f + 0.25f * contrast_score);
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
