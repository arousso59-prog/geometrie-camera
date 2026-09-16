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
constexpr uint8_t REFERENCE_SEARCH_RADIUS = 5;
constexpr uint8_t DARK_CORE_DELTA = 16;
constexpr uint8_t DARK_MIN_DELTA = 9;
constexpr uint8_t DARK_EDGE_DELTA = 7;
constexpr uint8_t MAX_REFERENCE_DIFFERENCE = 60;
constexpr uint8_t GREEN_NEIGHBOR_X = 2;
constexpr uint8_t DARK_EDGE_EXPANSION = 2;

struct ReferenceSample {
  bool have_above;
  bool have_below;
  uint8_t above;
  uint8_t below;
  uint8_t replacement;
  int average;
  int minimum;
  int difference;
};

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

bool get_reference_sample(const uint8_t *grayscale, size_t row_stride,
                          const uint8_t *green_mask, uint16_t width, uint16_t height,
                          int x, int y, ReferenceSample *sample) {
  if (sample == nullptr) {
    return false;
  }

  sample->above = 0;
  sample->below = 0;
  sample->have_above = find_reference_value(grayscale, row_stride, green_mask,
                                            width, height, x, y, -1, &sample->above);
  sample->have_below = find_reference_value(grayscale, row_stride, green_mask,
                                            width, height, x, y, +1, &sample->below);
  if (!sample->have_above && !sample->have_below) {
    return false;
  }

  if (sample->have_above && sample->have_below) {
    sample->average = (static_cast<int>(sample->above) + static_cast<int>(sample->below) + 1) / 2;
    sample->minimum = std::min<int>(sample->above, sample->below);
    sample->difference = std::abs(static_cast<int>(sample->above) - static_cast<int>(sample->below));
    sample->replacement = static_cast<uint8_t>(sample->average);
  } else if (sample->have_above) {
    sample->average = sample->above;
    sample->minimum = sample->above;
    sample->difference = 0;
    sample->replacement = sample->above;
  } else {
    sample->average = sample->below;
    sample->minimum = sample->below;
    sample->difference = 0;
    sample->replacement = sample->below;
  }

  return true;
}

bool is_dark_core(uint8_t current, const ReferenceSample &sample) {
  if (!sample.have_above || !sample.have_below || sample.difference > MAX_REFERENCE_DIFFERENCE) {
    return false;
  }
  return static_cast<int>(current) + DARK_CORE_DELTA < sample.average &&
         static_cast<int>(current) + DARK_MIN_DELTA < sample.minimum;
}

bool is_dark_edge(uint8_t current, const ReferenceSample &sample) {
  if (!sample.have_above || !sample.have_below || sample.difference > MAX_REFERENCE_DIFFERENCE) {
    return false;
  }
  return static_cast<int>(current) + DARK_EDGE_DELTA < sample.average;
}

uint16_t horizontal_expansion(uint16_t width) {
  // V3 searches a little farther around each green dash because the black
  // component can be displaced from the chroma corruption. This is still a
  // very small local window: 16 px at 1600 and 25 px at 2560.
  return std::max<uint16_t>(8, width / 100);
}

uint16_t maximum_dark_run_length(uint16_t width) {
  // The defect scales with image resolution. Reject long dark structures so
  // genuine target edges or scene objects are not repaired as artefacts.
  return std::max<uint16_t>(8, width / 64);
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

  // Count raw green seeds once. Processing remains restricted to compact
  // horizontal intervals around thin green dashes; there is no full-frame
  // neighbourhood search during correction.
  const size_t pixel_count = static_cast<size_t>(width) * height;
  for (size_t index = 0; index < pixel_count; ++index) {
    if ((green_mask[index >> 3] & static_cast<uint8_t>(1U << (index & 7U))) != 0) {
      this->stats_.green_seed_pixels++;
    }
  }

  const uint16_t expand_x = horizontal_expansion(width);
  const uint16_t max_dark_run = maximum_dark_run_length(width);

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

      // First pass: repair the green component. It is strongly identified by
      // chroma and can therefore be corrected independently of black dashes.
      for (int px = interval_start; px <= interval_end; ++px) {
        if (is_thin_green(green_mask, width, height, px, y)) {
          this->stats_.thin_green_pixels++;
        }

        if (!has_green_seed_near_x(green_mask, width, height, px, y)) {
          continue;
        }

        ReferenceSample sample{};
        if (!get_reference_sample(grayscale, row_stride, green_mask,
                                  width, height, px, y, &sample)) {
          continue;
        }

        grayscale[static_cast<size_t>(y) * row_stride + static_cast<size_t>(px)] = sample.replacement;
        this->stats_.corrected_green_pixels++;
        this->stats_.corrected_total_pixels++;
      }

      // Second pass: detect complete short dark runs. The centre must be a
      // strong vertical luminance impulse, while up to two softer edge pixels
      // are accepted on either side. A real black feature generally continues
      // vertically, making the above/below references dark as well and thus
      // failing this impulse test.
      int px = interval_start;
      while (px <= interval_end) {
        ReferenceSample core_sample{};
        const size_t core_index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(px);
        if (has_green_seed_near_x(green_mask, width, height, px, y) ||
            !get_reference_sample(grayscale, row_stride, green_mask,
                                  width, height, px, y, &core_sample) ||
            !is_dark_core(grayscale[core_index], core_sample)) {
          ++px;
          continue;
        }

        int run_start = px;
        int run_end = px;

        // Grow through contiguous core-dark pixels first.
        int probe = px + 1;
        while (probe <= interval_end) {
          ReferenceSample sample{};
          const size_t index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(probe);
          if (has_green_seed_near_x(green_mask, width, height, probe, y) ||
              !get_reference_sample(grayscale, row_stride, green_mask,
                                    width, height, probe, y, &sample) ||
              !is_dark_core(grayscale[index], sample)) {
            break;
          }
          run_end = probe;
          ++probe;
        }

        // Include antialiased/ringing edges that are less dark than the core.
        for (uint8_t edge = 0; edge < DARK_EDGE_EXPANSION && run_start > interval_start; ++edge) {
          const int candidate = run_start - 1;
          ReferenceSample sample{};
          const size_t index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(candidate);
          if (has_green_seed_near_x(green_mask, width, height, candidate, y) ||
              !get_reference_sample(grayscale, row_stride, green_mask,
                                    width, height, candidate, y, &sample) ||
              !is_dark_edge(grayscale[index], sample)) {
            break;
          }
          run_start = candidate;
        }

        for (uint8_t edge = 0; edge < DARK_EDGE_EXPANSION && run_end < interval_end; ++edge) {
          const int candidate = run_end + 1;
          ReferenceSample sample{};
          const size_t index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(candidate);
          if (has_green_seed_near_x(green_mask, width, height, candidate, y) ||
              !get_reference_sample(grayscale, row_stride, green_mask,
                                    width, height, candidate, y, &sample) ||
              !is_dark_edge(grayscale[index], sample)) {
            break;
          }
          run_end = candidate;
        }

        const uint16_t run_length = static_cast<uint16_t>(run_end - run_start + 1);
        if (run_length <= max_dark_run) {
          for (int repair_x = run_start; repair_x <= run_end; ++repair_x) {
            ReferenceSample sample{};
            if (!get_reference_sample(grayscale, row_stride, green_mask,
                                      width, height, repair_x, y, &sample)) {
              continue;
            }
            grayscale[static_cast<size_t>(y) * row_stride + static_cast<size_t>(repair_x)] = sample.replacement;
            this->stats_.corrected_dark_pixels++;
            this->stats_.corrected_total_pixels++;
          }
        }

        px = std::max(probe, run_end + 1);
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
