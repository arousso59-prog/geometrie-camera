#include "jpeg_artifact_corrector.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr uint8_t GREEN_MIN_VALUE = 48;
constexpr uint8_t GREEN_DOMINANCE_DELTA = 28;
constexpr uint8_t GREEN_MEAN_DELTA = 22;
constexpr uint8_t THIN_GREEN_VERTICAL_RADIUS = 2;
constexpr uint8_t CANDIDATE_VERTICAL_RADIUS = 1;
constexpr uint8_t REFERENCE_SEARCH_RADIUS = 4;
constexpr uint8_t DARK_IMPULSE_DELTA = 26;
constexpr uint8_t MAX_REFERENCE_DIFFERENCE = 52;
constexpr uint8_t GREEN_NEIGHBOR_X = 2;

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

  // The observed defect is made of one/two-pixel-high dashes. Continuous
  // green objects are intentionally rejected here.
  return vertical_neighbors <= 1;
}

bool has_thin_green_near_row(const uint8_t *mask, uint16_t width, uint16_t height,
                             int x, int y) {
  for (int dy = -CANDIDATE_VERTICAL_RADIUS; dy <= CANDIDATE_VERTICAL_RADIUS; ++dy) {
    if (is_thin_green(mask, width, height, x, y + dy)) {
      return true;
    }
  }
  return false;
}

bool has_green_seed_near_x(const uint8_t *mask, uint16_t width, uint16_t height,
                           int x, int y) {
  for (int dx = -GREEN_NEIGHBOR_X; dx <= GREEN_NEIGHBOR_X; ++dx) {
    if (mask_get(mask, width, height, x + dx, y)) {
      return true;
    }
  }
  return false;
}

bool find_reference_value(const uint8_t *grayscale, size_t row_stride,
                          const uint8_t *green_mask, uint16_t width, uint16_t height,
                          int x, int y, int direction, uint8_t *value) {
  if (grayscale == nullptr || green_mask == nullptr || value == nullptr || x < 0 ||
      x >= width || direction == 0) {
    return false;
  }

  for (int distance = 1; distance <= REFERENCE_SEARCH_RADIUS; ++distance) {
    const int yy = y + direction * distance;
    if (yy < 0 || yy >= height) {
      break;
    }

    // Never use an obviously corrupted green sample as a reference.
    if (has_green_seed_near_x(green_mask, width, height, x, yy)) {
      continue;
    }

    *value = grayscale[static_cast<size_t>(yy) * row_stride + static_cast<size_t>(x)];
    return true;
  }
  return false;
}

uint16_t horizontal_expansion(uint16_t width) {
  // The dash length scales with the selected sensor resolution. About 10 px
  // at 1600-wide and 16 px at 2560-wide covers the observed black tail while
  // keeping the correction local.
  return std::max<uint16_t>(6, width / 160);
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

  // Count raw green seeds once. The previous V1 repeatedly searched large
  // neighbourhoods for every pixel and then searched up/down again for every
  // pixel of almost every row. On a 1600x1200 frame that made correction take
  // several seconds. V2 instead discovers short candidate intervals once per
  // row and only evaluates pixels inside those intervals.
  const size_t pixel_count = static_cast<size_t>(width) * height;
  for (size_t index = 0; index < pixel_count; ++index) {
    if ((green_mask[index >> 3] & static_cast<uint8_t>(1U << (index & 7U))) != 0) {
      this->stats_.green_seed_pixels++;
    }
  }

  const uint16_t expand_x = horizontal_expansion(width);

  for (uint16_t y = 0; y < height; ++y) {
    bool row_affected = false;
    int x = 0;

    while (x < width) {
      // Find the next thin green seed on this row or one neighbouring row.
      while (x < width && !has_thin_green_near_row(green_mask, width, height, x, y)) {
        ++x;
      }
      if (x >= width) {
        break;
      }

      const int first_seed = x;
      int last_seed = x;
      int gap = 0;

      // Merge close seed pixels into one dash. JPEG chroma ringing may create
      // one- or two-pixel holes inside what is visually one artefact.
      ++x;
      while (x < width && gap <= 3) {
        if (has_thin_green_near_row(green_mask, width, height, x, y)) {
          last_seed = x;
          gap = 0;
        } else {
          ++gap;
        }
        ++x;
      }

      const int interval_start = std::max<int>(0, first_seed - expand_x);
      const int interval_end = std::min<int>(width - 1, last_seed + expand_x);
      row_affected = true;

      for (int px = interval_start; px <= interval_end; ++px) {
        if (is_thin_green(green_mask, width, height, px, y)) {
          this->stats_.thin_green_pixels++;
        }

        uint8_t above = 0;
        uint8_t below = 0;
        const bool have_above = find_reference_value(grayscale, row_stride, green_mask,
                                                     width, height, px, y, -1, &above);
        const bool have_below = find_reference_value(grayscale, row_stride, green_mask,
                                                     width, height, px, y, +1, &below);
        if (!have_above && !have_below) {
          continue;
        }

        const size_t pixel_index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(px);
        const uint8_t current = grayscale[pixel_index];

        // Green corruption can spread by a couple of horizontal pixels after
        // JPEG decoding, hence the small local seed check.
        const bool green_artifact = has_green_seed_near_x(green_mask, width, height, px, y);

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

    if (row_affected) {
      this->stats_.affected_rows++;
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
