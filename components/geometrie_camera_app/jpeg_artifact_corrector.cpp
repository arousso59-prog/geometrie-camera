#include "jpeg_artifact_corrector.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr uint8_t THIN_GREEN_VERTICAL_RADIUS = 2;
constexpr uint8_t CANDIDATE_VERTICAL_RADIUS = 1;
constexpr uint8_t REFERENCE_SEARCH_RADIUS = 5;
constexpr uint8_t DARK_CORE_DELTA = 16;
constexpr uint8_t DARK_MIN_DELTA = 9;
constexpr uint8_t DARK_EDGE_DELTA = 7;
constexpr uint8_t MAX_REFERENCE_DIFFERENCE = 60;
constexpr uint8_t GREEN_NEIGHBOR_X = 2;
constexpr uint8_t DARK_EDGE_EXPANSION = 2;
constexpr uint8_t MAX_DASH_GAP = 3;

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

inline void mask_set_linear(uint8_t *mask, size_t index) {
  mask[index >> 3] |= static_cast<uint8_t>(1U << (index & 7U));
}

uint8_t mask_window_byte(const uint8_t *mask, uint16_t width, uint16_t height,
                         int y, uint16_t x_base) {
  if (mask == nullptr || y < 0 || y >= height || x_base >= width) {
    return 0;
  }

  const size_t total_bits = static_cast<size_t>(width) * height;
  const size_t total_bytes = (total_bits + 7U) / 8U;
  const size_t bit_offset = static_cast<size_t>(y) * width + x_base;
  const size_t byte_index = bit_offset >> 3;
  const uint8_t bit_shift = static_cast<uint8_t>(bit_offset & 7U);

  uint16_t word = mask[byte_index];
  if (bit_shift != 0 && byte_index + 1U < total_bytes) {
    word |= static_cast<uint16_t>(mask[byte_index + 1U]) << 8;
  }

  uint8_t value = static_cast<uint8_t>((word >> bit_shift) & 0xFFU);
  const uint16_t remaining = static_cast<uint16_t>(width - x_base);
  if (remaining < 8U) {
    value &= static_cast<uint8_t>((1U << remaining) - 1U);
  }
  return value;
}

int find_next_thin_green_x(const uint8_t *thin_green_mask, uint16_t width,
                           uint16_t height, int y, int start_x) {
  if (thin_green_mask == nullptr || width == 0 || height == 0 || y < 0 || y >= height ||
      start_x >= width) {
    return -1;
  }

  int search_x = std::max(0, start_x);
  uint16_t x_base = static_cast<uint16_t>(search_x & ~7);

  while (x_base < width) {
    uint8_t candidates = 0;
    for (int dy = -CANDIDATE_VERTICAL_RADIUS; dy <= CANDIDATE_VERTICAL_RADIUS; ++dy) {
      candidates |= mask_window_byte(thin_green_mask, width, height, y + dy, x_base);
    }

    if (search_x > x_base) {
      const uint8_t skip = static_cast<uint8_t>(search_x - x_base);
      candidates &= static_cast<uint8_t>(0xFFU << skip);
    }

    if (candidates != 0) {
      const uint8_t bit = static_cast<uint8_t>(__builtin_ctz(static_cast<unsigned>(candidates)));
      const int candidate_x = static_cast<int>(x_base) + bit;
      if (candidate_x < width) {
        return candidate_x;
      }
    }

    x_base = static_cast<uint16_t>(x_base + 8U);
    search_x = x_base;
  }

  return -1;
}

bool find_reference_value(const uint8_t *grayscale, size_t row_stride,
                          const uint8_t *near_green_mask, uint16_t width, uint16_t height,
                          int x, int y, int direction, uint8_t *value) {
  if (grayscale == nullptr || near_green_mask == nullptr || value == nullptr || x < 0 ||
      x >= width || direction == 0) {
    return false;
  }

  for (int distance = 1; distance <= REFERENCE_SEARCH_RADIUS; ++distance) {
    const int yy = y + direction * distance;
    if (yy < 0 || yy >= height) {
      break;
    }

    if (mask_get(near_green_mask, width, height, x, yy)) {
      continue;
    }

    *value = grayscale[static_cast<size_t>(yy) * row_stride + static_cast<size_t>(x)];
    return true;
  }
  return false;
}

bool get_reference_sample(const uint8_t *grayscale, size_t row_stride,
                          const uint8_t *near_green_mask, uint16_t width, uint16_t height,
                          int x, int y, ReferenceSample *sample) {
  if (sample == nullptr) {
    return false;
  }

  sample->above = 0;
  sample->below = 0;
  sample->have_above = find_reference_value(grayscale, row_stride, near_green_mask,
                                            width, height, x, y, -1, &sample->above);
  sample->have_below = find_reference_value(grayscale, row_stride, near_green_mask,
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
  return std::max<uint16_t>(8, width / 100);
}

uint16_t maximum_dark_run_length(uint16_t width) {
  return std::max<uint16_t>(8, width / 64);
}
}

JpegArtifactCorrector::JpegArtifactCorrector()
    : near_green_mask_(nullptr), thin_green_mask_(nullptr), mask_capacity_(0) {
  this->reset_stats_();
}

JpegArtifactCorrector::~JpegArtifactCorrector() { this->clear_workspace_(); }

bool JpegArtifactCorrector::correct(uint8_t *grayscale, size_t row_stride,
                                    const uint8_t *green_mask, uint16_t width,
                                    uint16_t height, uint32_t green_seed_count) {
  this->reset_stats_();
  if (grayscale == nullptr || green_mask == nullptr || width == 0 || height == 0 ||
      row_stride < width) {
    return false;
  }

  this->stats_.green_seed_pixels = green_seed_count;

  if (!this->build_fast_masks_(green_mask, width, height)) {
    return false;
  }

  const uint16_t expand_x = horizontal_expansion(width);
  const uint16_t max_dark_run = maximum_dark_run_length(width);

  for (uint16_t y = 0; y < height; ++y) {
    bool row_affected = false;
    int search_x = 0;

    while (search_x < width) {
      const int first_seed = find_next_thin_green_x(this->thin_green_mask_, width, height, y, search_x);
      if (first_seed < 0) {
        break;
      }

      int last_seed = first_seed;
      int next_seed = find_next_thin_green_x(this->thin_green_mask_, width, height, y, last_seed + 1);
      while (next_seed >= 0 && next_seed - last_seed <= static_cast<int>(MAX_DASH_GAP) + 1) {
        last_seed = next_seed;
        next_seed = find_next_thin_green_x(this->thin_green_mask_, width, height, y, last_seed + 1);
      }

      search_x = next_seed >= 0 ? next_seed : width;

      const int interval_start = std::max<int>(0, first_seed - expand_x);
      const int interval_end = std::min<int>(width - 1, last_seed + expand_x);
      row_affected = true;

      for (int px = interval_start; px <= interval_end; ++px) {
        if (mask_get(this->thin_green_mask_, width, height, px, y)) {
          this->stats_.thin_green_pixels++;
        }

        if (!mask_get(this->near_green_mask_, width, height, px, y)) {
          continue;
        }

        ReferenceSample sample{};
        if (!get_reference_sample(grayscale, row_stride, this->near_green_mask_,
                                  width, height, px, y, &sample)) {
          continue;
        }

        grayscale[static_cast<size_t>(y) * row_stride + static_cast<size_t>(px)] = sample.replacement;
        this->stats_.corrected_green_pixels++;
        this->stats_.corrected_total_pixels++;
      }

      int px = interval_start;
      while (px <= interval_end) {
        ReferenceSample core_sample{};
        const size_t core_index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(px);
        if (mask_get(this->near_green_mask_, width, height, px, y) ||
            !get_reference_sample(grayscale, row_stride, this->near_green_mask_,
                                  width, height, px, y, &core_sample) ||
            !is_dark_core(grayscale[core_index], core_sample)) {
          ++px;
          continue;
        }

        int run_start = px;
        int run_end = px;

        int probe = px + 1;
        while (probe <= interval_end) {
          ReferenceSample sample{};
          const size_t index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(probe);
          if (mask_get(this->near_green_mask_, width, height, probe, y) ||
              !get_reference_sample(grayscale, row_stride, this->near_green_mask_,
                                    width, height, probe, y, &sample) ||
              !is_dark_core(grayscale[index], sample)) {
            break;
          }
          run_end = probe;
          ++probe;
        }

        for (uint8_t edge = 0; edge < DARK_EDGE_EXPANSION && run_start > interval_start; ++edge) {
          const int candidate = run_start - 1;
          ReferenceSample sample{};
          const size_t index = static_cast<size_t>(y) * row_stride + static_cast<size_t>(candidate);
          if (mask_get(this->near_green_mask_, width, height, candidate, y) ||
              !get_reference_sample(grayscale, row_stride, this->near_green_mask_,
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
          if (mask_get(this->near_green_mask_, width, height, candidate, y) ||
              !get_reference_sample(grayscale, row_stride, this->near_green_mask_,
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
            if (!get_reference_sample(grayscale, row_stride, this->near_green_mask_,
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

bool JpegArtifactCorrector::ensure_workspace_(size_t mask_size) {
  if (this->near_green_mask_ != nullptr && this->thin_green_mask_ != nullptr &&
      this->mask_capacity_ >= mask_size) {
    return true;
  }

  auto *new_near = static_cast<uint8_t *>(
      heap_caps_malloc(mask_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (new_near == nullptr) {
    return false;
  }

  auto *new_thin = static_cast<uint8_t *>(
      heap_caps_malloc(mask_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (new_thin == nullptr) {
    heap_caps_free(new_near);
    return false;
  }

  this->clear_workspace_();
  this->near_green_mask_ = new_near;
  this->thin_green_mask_ = new_thin;
  this->mask_capacity_ = mask_size;
  return true;
}

bool JpegArtifactCorrector::build_fast_masks_(const uint8_t *green_mask,
                                               uint16_t width, uint16_t height) {
  const size_t pixel_count = static_cast<size_t>(width) * height;
  const size_t mask_size = (pixel_count + 7U) / 8U;
  if (!this->ensure_workspace_(mask_size)) {
    return false;
  }

  std::memset(this->near_green_mask_, 0, mask_size);
  std::memset(this->thin_green_mask_, 0, mask_size);

  for (size_t byte_index = 0; byte_index < mask_size; ++byte_index) {
    uint8_t bits = green_mask[byte_index];
    while (bits != 0) {
      const uint8_t bit = static_cast<uint8_t>(__builtin_ctz(static_cast<unsigned>(bits)));
      const size_t index = (byte_index << 3) + bit;
      if (index >= pixel_count) {
        break;
      }

      const uint16_t y = static_cast<uint16_t>(index / width);
      const uint16_t x = static_cast<uint16_t>(index - static_cast<size_t>(y) * width);

      const int start_x = std::max<int>(0, static_cast<int>(x) - GREEN_NEIGHBOR_X);
      const int end_x = std::min<int>(width - 1, static_cast<int>(x) + GREEN_NEIGHBOR_X);
      const size_t row_base = static_cast<size_t>(y) * width;
      for (int xx = start_x; xx <= end_x; ++xx) {
        mask_set_linear(this->near_green_mask_, row_base + static_cast<size_t>(xx));
      }

      uint8_t vertical_neighbors = 0;
      for (int dy = -THIN_GREEN_VERTICAL_RADIUS; dy <= THIN_GREEN_VERTICAL_RADIUS; ++dy) {
        if (dy == 0) {
          continue;
        }
        if (mask_get(green_mask, width, height, x, static_cast<int>(y) + dy)) {
          vertical_neighbors++;
        }
      }
      if (vertical_neighbors <= 1) {
        mask_set_linear(this->thin_green_mask_, index);
      }

      bits &= static_cast<uint8_t>(bits - 1U);
    }
  }

  return true;
}

void JpegArtifactCorrector::clear_workspace_() {
  if (this->near_green_mask_ != nullptr) {
    heap_caps_free(this->near_green_mask_);
  }
  if (this->thin_green_mask_ != nullptr) {
    heap_caps_free(this->thin_green_mask_);
  }
  this->near_green_mask_ = nullptr;
  this->thin_green_mask_ = nullptr;
  this->mask_capacity_ = 0;
}

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
