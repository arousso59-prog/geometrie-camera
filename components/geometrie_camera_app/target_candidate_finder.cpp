#include "target_candidate_finder.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "esp_heap_caps.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr uint16_t MAX_REDUCED_DIMENSION = 400;
constexpr uint16_t LOCAL_TILE_SIZE = 24;
constexpr uint8_t LOCAL_DARK_MARGIN = 8;
constexpr uint32_t MIN_COMPONENT_PIXELS = 6;
constexpr float MIN_ASPECT_RATIO = 0.50f;
constexpr float MAX_ASPECT_RATIO = 2.00f;
constexpr float MIN_FILL_RATIO = 0.12f;
constexpr float MAX_FILL_RATIO = 0.92f;
constexpr uint16_t MIN_TARGET_SIDE_PX = 12;
constexpr uint16_t MAX_TARGET_SIDE_DIVISOR = 4;

float distance_between(const TargetPoint &a, const TargetPoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}
}

TargetPoint::TargetPoint() : x(0.0f), y(0.0f) {}

TargetCandidate::TargetCandidate()
    : top_left(),
      top_right(),
      bottom_right(),
      bottom_left(),
      center_x(0.0f),
      center_y(0.0f),
      width(0.0f),
      height(0.0f),
      localization_score(0.0f) {}

TargetCandidateSet::TargetCandidateSet() : candidates(), count(0) {}

TargetCandidateFinder::TargetCandidateFinder()
    : reduced_width_(0),
      reduced_height_(0),
      tile_columns_(0),
      tile_rows_(0),
      reduced_(nullptr),
      tile_means_(nullptr),
      queue_(nullptr),
      reduced_capacity_(0),
      tile_capacity_(0),
      queue_capacity_(0) {}

TargetCandidateFinder::~TargetCandidateFinder() { this->clear_workspace_(); }

bool TargetCandidateFinder::find(const GrayFrameView &frame, TargetCandidateSet &result) {
  result = TargetCandidateSet();

  if (frame.data == nullptr || frame.width == 0 || frame.height == 0 || frame.stride < frame.width) {
    return false;
  }

  const uint16_t largest_dimension = std::max<uint16_t>(frame.width, frame.height);
  const uint16_t scale = std::max<uint16_t>(
      1, static_cast<uint16_t>((static_cast<uint32_t>(largest_dimension) + MAX_REDUCED_DIMENSION - 1U) /
                               MAX_REDUCED_DIMENSION));

  if (!this->build_reduced_image_(frame, scale)) {
    return false;
  }

  if (!this->build_local_threshold_map_()) {
    return false;
  }

  this->collect_components_(frame, scale, result);
  return true;
}

bool TargetCandidateFinder::ensure_workspace_(size_t pixel_count, size_t tile_count) {
  if (pixel_count == 0 || tile_count == 0) {
    return false;
  }

  if (this->reduced_capacity_ < pixel_count) {
    auto *new_buffer = static_cast<uint8_t *>(
        heap_caps_malloc(pixel_count * sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_buffer == nullptr) {
      return false;
    }
    if (this->reduced_ != nullptr) {
      heap_caps_free(this->reduced_);
    }
    this->reduced_ = new_buffer;
    this->reduced_capacity_ = pixel_count;
  }

  if (this->queue_capacity_ < pixel_count) {
    auto *new_queue = static_cast<uint32_t *>(
        heap_caps_malloc(pixel_count * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_queue == nullptr) {
      return false;
    }
    if (this->queue_ != nullptr) {
      heap_caps_free(this->queue_);
    }
    this->queue_ = new_queue;
    this->queue_capacity_ = pixel_count;
  }

  if (this->tile_capacity_ < tile_count) {
    auto *new_tiles = static_cast<uint16_t *>(
        heap_caps_malloc(tile_count * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_tiles == nullptr) {
      return false;
    }
    if (this->tile_means_ != nullptr) {
      heap_caps_free(this->tile_means_);
    }
    this->tile_means_ = new_tiles;
    this->tile_capacity_ = tile_count;
  }

  return this->reduced_ != nullptr && this->queue_ != nullptr && this->tile_means_ != nullptr;
}

void TargetCandidateFinder::clear_workspace_() {
  if (this->reduced_ != nullptr) {
    heap_caps_free(this->reduced_);
  }
  if (this->tile_means_ != nullptr) {
    heap_caps_free(this->tile_means_);
  }
  if (this->queue_ != nullptr) {
    heap_caps_free(this->queue_);
  }

  this->reduced_ = nullptr;
  this->tile_means_ = nullptr;
  this->queue_ = nullptr;
  this->reduced_capacity_ = 0;
  this->tile_capacity_ = 0;
  this->queue_capacity_ = 0;
}

bool TargetCandidateFinder::build_reduced_image_(const GrayFrameView &frame, uint16_t scale) {
  this->reduced_width_ = static_cast<uint16_t>(
      (static_cast<uint32_t>(frame.width) + scale - 1U) / scale);
  this->reduced_height_ = static_cast<uint16_t>(
      (static_cast<uint32_t>(frame.height) + scale - 1U) / scale);

  this->tile_columns_ = static_cast<uint16_t>(
      (static_cast<uint32_t>(this->reduced_width_) + LOCAL_TILE_SIZE - 1U) / LOCAL_TILE_SIZE);
  this->tile_rows_ = static_cast<uint16_t>(
      (static_cast<uint32_t>(this->reduced_height_) + LOCAL_TILE_SIZE - 1U) / LOCAL_TILE_SIZE);

  const size_t pixel_count = static_cast<size_t>(this->reduced_width_) * this->reduced_height_;
  const size_t tile_count = static_cast<size_t>(this->tile_columns_) * this->tile_rows_;
  if (!this->ensure_workspace_(pixel_count, tile_count)) {
    return false;
  }

  for (uint16_t ry = 0; ry < this->reduced_height_; ++ry) {
    const uint32_t source_y0 = static_cast<uint32_t>(ry) * scale;
    const uint32_t source_y1 = std::min<uint32_t>(frame.height, source_y0 + scale);

    for (uint16_t rx = 0; rx < this->reduced_width_; ++rx) {
      const uint32_t source_x0 = static_cast<uint32_t>(rx) * scale;
      const uint32_t source_x1 = std::min<uint32_t>(frame.width, source_x0 + scale);
      uint32_t sum = 0;
      uint16_t count = 0;

      for (uint32_t y = source_y0; y < source_y1; ++y) {
        const uint8_t *line = frame.data + static_cast<size_t>(y) * frame.stride;
        for (uint32_t x = source_x0; x < source_x1; ++x) {
          sum += line[x];
          count++;
        }
      }

      this->reduced_[static_cast<size_t>(ry) * this->reduced_width_ + rx] =
          count == 0 ? 0 : static_cast<uint8_t>(sum / count);
    }
  }

  return true;
}

bool TargetCandidateFinder::build_local_threshold_map_() {
  if (this->reduced_ == nullptr || this->tile_means_ == nullptr || this->reduced_width_ == 0 ||
      this->reduced_height_ == 0 || this->tile_columns_ == 0 || this->tile_rows_ == 0) {
    return false;
  }

  for (uint16_t ty = 0; ty < this->tile_rows_; ++ty) {
    const uint16_t y0 = static_cast<uint16_t>(ty * LOCAL_TILE_SIZE);
    const uint16_t y1 = std::min<uint16_t>(this->reduced_height_, static_cast<uint16_t>(y0 + LOCAL_TILE_SIZE));

    for (uint16_t tx = 0; tx < this->tile_columns_; ++tx) {
      const uint16_t x0 = static_cast<uint16_t>(tx * LOCAL_TILE_SIZE);
      const uint16_t x1 = std::min<uint16_t>(this->reduced_width_, static_cast<uint16_t>(x0 + LOCAL_TILE_SIZE));
      uint32_t sum = 0;
      uint32_t count = 0;

      for (uint16_t y = y0; y < y1; ++y) {
        const size_t row_offset = static_cast<size_t>(y) * this->reduced_width_;
        for (uint16_t x = x0; x < x1; ++x) {
          sum += this->reduced_[row_offset + x];
          count++;
        }
      }

      this->tile_means_[static_cast<size_t>(ty) * this->tile_columns_ + tx] =
          count == 0 ? 0 : static_cast<uint16_t>(sum / count);
    }
  }

  for (uint16_t y = 0; y < this->reduced_height_; ++y) {
    const uint16_t tile_y = static_cast<uint16_t>(y / LOCAL_TILE_SIZE);
    const uint16_t tile_y0 = tile_y == 0 ? 0 : static_cast<uint16_t>(tile_y - 1U);
    const uint16_t tile_y1 = std::min<uint16_t>(this->tile_rows_ - 1U, static_cast<uint16_t>(tile_y + 1U));

    for (uint16_t x = 0; x < this->reduced_width_; ++x) {
      const uint16_t tile_x = static_cast<uint16_t>(x / LOCAL_TILE_SIZE);
      const uint16_t tile_x0 = tile_x == 0 ? 0 : static_cast<uint16_t>(tile_x - 1U);
      const uint16_t tile_x1 = std::min<uint16_t>(this->tile_columns_ - 1U, static_cast<uint16_t>(tile_x + 1U));
      uint32_t local_sum = 0;
      uint16_t local_count = 0;

      for (uint16_t ty = tile_y0; ty <= tile_y1; ++ty) {
        for (uint16_t tx = tile_x0; tx <= tile_x1; ++tx) {
          local_sum += this->tile_means_[static_cast<size_t>(ty) * this->tile_columns_ + tx];
          local_count++;
        }
      }

      const uint16_t local_mean = local_count == 0 ? 0 : static_cast<uint16_t>(local_sum / local_count);
      const size_t index = static_cast<size_t>(y) * this->reduced_width_ + x;
      const uint8_t value = this->reduced_[index];
      this->reduced_[index] = static_cast<uint16_t>(value) + LOCAL_DARK_MARGIN < local_mean ? 1 : 0;
    }
  }

  return true;
}

void TargetCandidateFinder::collect_components_(const GrayFrameView &frame, uint16_t scale,
                                                TargetCandidateSet &result) {
  if (this->reduced_ == nullptr || this->queue_ == nullptr) {
    return;
  }

  const uint16_t maximum_side = std::max<uint16_t>(
      MIN_TARGET_SIDE_PX,
      static_cast<uint16_t>(std::min<uint16_t>(frame.width, frame.height) / MAX_TARGET_SIDE_DIVISOR));

  for (uint16_t start_y = 0; start_y < this->reduced_height_; ++start_y) {
    for (uint16_t start_x = 0; start_x < this->reduced_width_; ++start_x) {
      const size_t start_index = static_cast<size_t>(start_y) * this->reduced_width_ + start_x;
      if (this->reduced_[start_index] != 1) {
        continue;
      }

      size_t queue_size = 0;
      size_t queue_position = 0;
      this->queue_[queue_size++] = static_cast<uint32_t>(start_index);
      this->reduced_[start_index] = 2;

      uint32_t component_count = 0;
      uint16_t min_x = start_x;
      uint16_t max_x = start_x;
      uint16_t min_y = start_y;
      uint16_t max_y = start_y;

      int32_t min_sum = std::numeric_limits<int32_t>::max();
      int32_t max_sum = std::numeric_limits<int32_t>::min();
      int32_t min_diff = std::numeric_limits<int32_t>::max();
      int32_t max_diff = std::numeric_limits<int32_t>::min();
      uint16_t min_sum_x = start_x, min_sum_y = start_y;
      uint16_t max_sum_x = start_x, max_sum_y = start_y;
      uint16_t min_diff_x = start_x, min_diff_y = start_y;
      uint16_t max_diff_x = start_x, max_diff_y = start_y;

      while (queue_position < queue_size) {
        const uint32_t index = this->queue_[queue_position++];
        const uint16_t y = static_cast<uint16_t>(index / this->reduced_width_);
        const uint16_t x = static_cast<uint16_t>(index - static_cast<uint32_t>(y) * this->reduced_width_);
        component_count++;

        min_x = std::min<uint16_t>(min_x, x);
        max_x = std::max<uint16_t>(max_x, x);
        min_y = std::min<uint16_t>(min_y, y);
        max_y = std::max<uint16_t>(max_y, y);

        const int32_t sum = static_cast<int32_t>(x) + static_cast<int32_t>(y);
        const int32_t diff = static_cast<int32_t>(x) - static_cast<int32_t>(y);
        if (sum < min_sum) {
          min_sum = sum;
          min_sum_x = x;
          min_sum_y = y;
        }
        if (sum > max_sum) {
          max_sum = sum;
          max_sum_x = x;
          max_sum_y = y;
        }
        if (diff < min_diff) {
          min_diff = diff;
          min_diff_x = x;
          min_diff_y = y;
        }
        if (diff > max_diff) {
          max_diff = diff;
          max_diff_x = x;
          max_diff_y = y;
        }

        const int dx[4] = {-1, 1, 0, 0};
        const int dy[4] = {0, 0, -1, 1};
        for (uint8_t direction = 0; direction < 4; ++direction) {
          const int nx = static_cast<int>(x) + dx[direction];
          const int ny = static_cast<int>(y) + dy[direction];
          if (nx < 0 || ny < 0 || nx >= this->reduced_width_ || ny >= this->reduced_height_) {
            continue;
          }

          const size_t neighbor_index = static_cast<size_t>(ny) * this->reduced_width_ + nx;
          if (this->reduced_[neighbor_index] == 1 && queue_size < this->queue_capacity_) {
            this->reduced_[neighbor_index] = 2;
            this->queue_[queue_size++] = static_cast<uint32_t>(neighbor_index);
          }
        }
      }

      if (component_count < MIN_COMPONENT_PIXELS) {
        continue;
      }

      const uint16_t reduced_box_width = static_cast<uint16_t>(max_x - min_x + 1U);
      const uint16_t reduced_box_height = static_cast<uint16_t>(max_y - min_y + 1U);
      const uint16_t full_box_width = static_cast<uint16_t>(reduced_box_width * scale);
      const uint16_t full_box_height = static_cast<uint16_t>(reduced_box_height * scale);
      const uint16_t min_side = std::min<uint16_t>(full_box_width, full_box_height);
      const uint16_t max_side = std::max<uint16_t>(full_box_width, full_box_height);

      if (min_side < MIN_TARGET_SIDE_PX || max_side > maximum_side) {
        continue;
      }

      const float aspect = static_cast<float>(full_box_width) / static_cast<float>(full_box_height);
      if (aspect < MIN_ASPECT_RATIO || aspect > MAX_ASPECT_RATIO) {
        continue;
      }

      const float fill_ratio = static_cast<float>(component_count) /
                               static_cast<float>(static_cast<uint32_t>(reduced_box_width) * reduced_box_height);
      if (fill_ratio < MIN_FILL_RATIO || fill_ratio > MAX_FILL_RATIO) {
        continue;
      }

      auto map_point = [&](uint16_t rx, uint16_t ry) {
        TargetPoint point;
        point.x = std::min<float>(frame.width - 1.0f, (static_cast<float>(rx) + 0.5f) * scale);
        point.y = std::min<float>(frame.height - 1.0f, (static_cast<float>(ry) + 0.5f) * scale);
        return point;
      };

      TargetCandidate candidate;
      candidate.top_left = map_point(min_sum_x, min_sum_y);
      candidate.top_right = map_point(max_diff_x, max_diff_y);
      candidate.bottom_right = map_point(max_sum_x, max_sum_y);
      candidate.bottom_left = map_point(min_diff_x, min_diff_y);
      candidate.center_x = (candidate.top_left.x + candidate.top_right.x + candidate.bottom_right.x +
                            candidate.bottom_left.x) * 0.25f;
      candidate.center_y = (candidate.top_left.y + candidate.top_right.y + candidate.bottom_right.y +
                            candidate.bottom_left.y) * 0.25f;
      candidate.width = 0.5f * (distance_between(candidate.top_left, candidate.top_right) +
                                distance_between(candidate.bottom_left, candidate.bottom_right));
      candidate.height = 0.5f * (distance_between(candidate.top_left, candidate.bottom_left) +
                                 distance_between(candidate.top_right, candidate.bottom_right));

      if (candidate.width < MIN_TARGET_SIDE_PX || candidate.height < MIN_TARGET_SIDE_PX) {
        continue;
      }

      const float geometric_ratio = std::min(candidate.width, candidate.height) /
                                    std::max(candidate.width, candidate.height);
      const float density_score = std::min(1.0f, fill_ratio / 0.45f);
      candidate.localization_score = 0.72f * geometric_ratio + 0.28f * density_score;
      this->insert_candidate_(result, candidate);
    }
  }
}

void TargetCandidateFinder::insert_candidate_(TargetCandidateSet &result,
                                              const TargetCandidate &candidate) const {
  size_t insertion = result.count;
  if (insertion < TargetCandidateSet::MAX_CANDIDATES) {
    result.count++;
  } else {
    insertion = TargetCandidateSet::MAX_CANDIDATES - 1U;
    if (candidate.localization_score <= result.candidates[insertion].localization_score) {
      return;
    }
  }

  while (insertion > 0 &&
         candidate.localization_score > result.candidates[insertion - 1U].localization_score) {
    if (insertion < TargetCandidateSet::MAX_CANDIDATES) {
      result.candidates[insertion] = result.candidates[insertion - 1U];
    }
    insertion--;
  }

  result.candidates[insertion] = candidate;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
