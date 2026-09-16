#include "jpeg_artifact_corrector.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr uint8_t GREEN_MIN_VALUE = 48;
constexpr uint8_t GREEN_DOMINANCE_DELTA = 28;
constexpr uint8_t GREEN_MEAN_DELTA = 22;
constexpr uint8_t THIN_GREEN_VERTICAL_RADIUS = 2;
constexpr uint8_t GREEN_EXPAND_X = 3;
constexpr uint8_t GREEN_EXPAND_Y = 1;
constexpr uint8_t REFERENCE_SEARCH_RADIUS = 6;
constexpr uint8_t DARK_IMPULSE_DELTA = 38;
constexpr uint8_t MAX_REFERENCE_DIFFERENCE = 48;

inline bool mask_get(const uint8_t *mask, uint16_t width, uint16_t height, int x, int y) {
  if (mask == nullptr || x < 0 || y < 0 || x >= width || y >= height) {
    return false;
  }
  const size_t index = static_cast<size_t>(y) * width + static_cast<size_t>(x);
  return (mask[index >> 3] & static_cast<uint8_t>(1U << (index & 7U))) != 0;
}

bool is_thin_green(const uint8_t *mask, uint16_t width, uint16_t height, int x, int y) {
  if (!mask_get(mask, width, height, x, y)) {
    return false;
  }

  uint8_t vertical_neighbors = 0;
  for (int dy = -THIN_GREEN_VERTICAL_RADIUS; dy <= THIN_GREEN_VERTICAL_RADIUS; ++dy) {
    if (dy == 0) {
      continue;
    }
    if (mask_get(mask, width, height, x, y + dy)) {
      vertical_neighbors++;
    }
  }

  // The observed defect is one or two pixels high. A real green surface tends
  // to continue over several neighbouring rows and is deliberately rejected.
  return vertical_neighbors <= 1;
}

bool near_thin_green(const uint8_t *mask, uint16_t width, uint16_t height, int x, int y) {
  for (int dy = -GREEN_EXPAND_Y; dy <= GREEN_EXPAND_Y; ++dy) {
    for (int dx = -GREEN_EXPAND_X; dx <= GREEN_EXPAND_X; ++dx) {
      if (is_thin_green(mask, width, height, x + dx, y + dy)) {
        return true;
      }
    }
  }
  return false;
}

bool find_reference_value(const uint8_t *grayscale, size_t row_stride,
                          const std::vector<uint8_t> &affected_rows,
                          uint16_t width, uint16_t height, int x, int y,
                          int direction, uint8_t *value) {
  if (grayscale == nullptr || value == nullptr || x < 0 || x >= width || direction == 0) {
    return false;
  }

  for (int distance = 1; distance <= REFERENCE_SEARCH_RADIUS; ++distance) {
    const int yy = y + direction * distance;
    if (yy < 0 || yy >= height) {
      break;
    }
    if (affected_rows[yy] != 0) {
      continue;
    }
    *value = grayscale[static_cast<size_t>(yy) * row_stride + static_cast<size_t>(x)];
    return true;
  }
  return false;
}
}

JpegArtifactCorrector::JpegArtifactCorrector() { this->reset_stats_(); }

bool JpegArtifactCorrector::is_green_seed(uint8_t red, uint8_t green, uint8_t blue) const {
  const int max_other = std::max<int>(red, blue);
  const int mean_other = (static_cast<int>(red) + static_cast<int>(blue)) / 2;
  return green >= GREEN_MIN_VALUE &&
         static_cast<int>(green) - max_other >= GREEN_DOMINANCE_DELTA &&
         static_cast<int>(green) - mean_other >= GREEN_MEAN_DELTA;
}

bool JpegArtifactCorrector::correct(uint8_t *grayscale, size_t row_stride,
                                    const uint8_t *green_mask, uint16_t width,
                                    uint16_t height) {
  this->reset_stats_();
  if (grayscale == nullptr || green_mask == nullptr || width == 0 || height == 0 ||
      row_stride < width) {
    return false;
  }

  std::vector<uint8_t> affected_rows(height, 0);
  std::vector<uint8_t> expanded_rows(height, 0);

  const uint16_t row_seed_threshold = std::max<uint16_t>(4, width / 200);

  for (uint16_t y = 0; y < height; ++y) {
    uint16_t thin_count = 0;
    for (uint16_t x = 0; x < width; ++x) {
      if (mask_get(green_mask, width, height, x, y)) {
        this->stats_.green_seed_pixels++;
      }
      if (is_thin_green(green_mask, width, height, x, y)) {
        thin_count++;
        this->stats_.thin_green_pixels++;
      }
    }
    if (thin_count >= row_seed_threshold) {
      affected_rows[y] = 1;
    }
  }

  // Include the immediately adjacent row because JPEG ringing around the
  // green/black dash can spill by one pixel vertically.
  for (uint16_t y = 0; y < height; ++y) {
    if (affected_rows[y] == 0) {
      continue;
    }
    expanded_rows[y] = 1;
    if (y > 0) {
      expanded_rows[y - 1] = 1;
    }
    if (y + 1 < height) {
      expanded_rows[y + 1] = 1;
    }
  }
  affected_rows.swap(expanded_rows);

  for (uint16_t y = 0; y < height; ++y) {
    if (affected_rows[y] != 0) {
      this->stats_.affected_rows++;
    }
  }

  for (uint16_t y = 0; y < height; ++y) {
    if (affected_rows[y] == 0) {
      continue;
    }

    for (uint16_t x = 0; x < width; ++x) {
      uint8_t above = 0;
      uint8_t below = 0;
      const bool have_above = find_reference_value(grayscale, row_stride, affected_rows,
                                                   width, height, x, y, -1, &above);
      const bool have_below = find_reference_value(grayscale, row_stride, affected_rows,
                                                   width, height, x, y, +1, &below);
      if (!have_above && !have_below) {
        continue;
      }

      const size_t pixel_index = static_cast<size_t>(y) * row_stride + x;
      const uint8_t current = grayscale[pixel_index];
      const bool green_artifact = near_thin_green(green_mask, width, height, x, y);

      bool dark_impulse = false;
      if (have_above && have_below) {
        const int reference_min = std::min<int>(above, below);
        const int reference_difference = std::abs(static_cast<int>(above) - static_cast<int>(below));
        dark_impulse = reference_difference <= MAX_REFERENCE_DIFFERENCE &&
                       static_cast<int>(current) + DARK_IMPULSE_DELTA < reference_min;
      }

      if (!green_artifact && !dark_impulse) {
        continue;
      }

      uint8_t replacement = current;
      if (have_above && have_below) {
        replacement = static_cast<uint8_t>((static_cast<uint16_t>(above) + below + 1U) / 2U);
      } else if (have_above) {
        replacement = above;
      } else {
        replacement = below;
      }

      grayscale[pixel_index] = replacement;
      if (green_artifact) {
        this->stats_.corrected_green_pixels++;
      } else {
        this->stats_.corrected_dark_pixels++;
      }
      this->stats_.corrected_total_pixels++;
    }
  }

  return true;
}

const JpegArtifactCorrectionStats &JpegArtifactCorrector::stats() const { return this->stats_; }

void JpegArtifactCorrector::reset_stats_() {
  this->stats_.green_seed_pixels = 0;
  this->stats_.thin_green_pixels = 0;
  this->stats_.corrected_green_pixels = 0;
  this->stats_.corrected_dark_pixels = 0;
  this->stats_.corrected_total_pixels = 0;
  this->stats_.affected_rows = 0;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
