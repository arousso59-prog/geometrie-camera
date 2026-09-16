#include "target_detection_preview.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"
#include "jpeg_filtered_diagnostic.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr size_t BMP_FILE_HEADER_SIZE = 14;
constexpr size_t BMP_DIB_HEADER_SIZE = 40;
constexpr size_t BMP_PALETTE_SIZE = 256 * 4;
constexpr size_t BMP_PIXEL_OFFSET = BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE + BMP_PALETTE_SIZE;
constexpr uint16_t MAX_PREVIEW_WIDTH = 640;

void write_u16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFFU);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

void write_u32(uint8_t *buffer, size_t offset, uint32_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFFU);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}
}

TargetDetectionPreview::TargetDetectionPreview()
    : bmp_buffer_(nullptr), bmp_size_(0), bmp_capacity_(0), width_(0), height_(0) {}

TargetDetectionPreview::~TargetDetectionPreview() { this->clear_buffer_(); }

bool TargetDetectionPreview::render(const JpegFilteredDiagnostic *source,
                                    const TargetObservation &observation) {
  if (source == nullptr || !source->ready() || source->grayscale_data() == nullptr ||
      source->width() == 0 || source->height() == 0 || source->grayscale_stride() < source->width()) {
    return false;
  }

  const uint16_t source_width = source->width();
  const uint16_t source_height = source->height();
  const uint16_t preview_width = std::min<uint16_t>(source_width, MAX_PREVIEW_WIDTH);
  const uint16_t preview_height = std::max<uint16_t>(
      1, static_cast<uint16_t>((static_cast<uint32_t>(source_height) * preview_width) / source_width));
  const size_t row_stride = (static_cast<size_t>(preview_width) + 3U) & ~static_cast<size_t>(3U);
  const size_t required_size = BMP_PIXEL_OFFSET + row_stride * preview_height;

  if (!this->ensure_buffer_(required_size)) {
    return false;
  }

  this->bmp_size_ = required_size;
  this->width_ = preview_width;
  this->height_ = preview_height;
  this->build_bmp_header_(preview_width, preview_height, row_stride);

  uint8_t *pixels = this->bmp_buffer_ + BMP_PIXEL_OFFSET;
  const uint8_t *source_pixels = source->grayscale_data();
  const size_t source_stride = source->grayscale_stride();

  for (uint16_t y = 0; y < preview_height; ++y) {
    const uint32_t source_y = std::min<uint32_t>(
        source_height - 1U,
        (static_cast<uint32_t>(y) * source_height) / preview_height);
    uint8_t *destination = pixels + static_cast<size_t>(y) * row_stride;
    const uint8_t *source_line = source_pixels + static_cast<size_t>(source_y) * source_stride;

    for (uint16_t x = 0; x < preview_width; ++x) {
      const uint32_t source_x = std::min<uint32_t>(
          source_width - 1U,
          (static_cast<uint32_t>(x) * source_width) / preview_width);
      destination[x] = source_line[source_x];
    }
  }

  if (observation.width_px > 0.0f && observation.height_px > 0.0f) {
    const float scale_x = static_cast<float>(preview_width) / static_cast<float>(source_width);
    const float scale_y = static_cast<float>(preview_height) / static_cast<float>(source_height);
    const float half_width = observation.width_px * 0.5f;
    const float half_height = observation.height_px * 0.5f;

    const int x0 = static_cast<int>(std::lround((observation.center_x_px - half_width) * scale_x));
    const int y0 = static_cast<int>(std::lround((observation.center_y_px - half_height) * scale_y));
    const int x1 = static_cast<int>(std::lround((observation.center_x_px + half_width) * scale_x));
    const int y1 = static_cast<int>(std::lround((observation.center_y_px + half_height) * scale_y));
    this->draw_rectangle_(pixels, row_stride, preview_width, preview_height, x0, y0, x1, y1);
  }

  return true;
}

const uint8_t *TargetDetectionPreview::bmp_data() const { return this->bmp_buffer_; }
size_t TargetDetectionPreview::bmp_size() const { return this->bmp_size_; }
uint16_t TargetDetectionPreview::width() const { return this->width_; }
uint16_t TargetDetectionPreview::height() const { return this->height_; }

bool TargetDetectionPreview::ensure_buffer_(size_t required_size) {
  if (this->bmp_buffer_ != nullptr && this->bmp_capacity_ >= required_size) {
    return true;
  }

  auto *new_buffer = static_cast<uint8_t *>(
      heap_caps_malloc(required_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (new_buffer == nullptr) {
    return false;
  }

  if (this->bmp_buffer_ != nullptr) {
    heap_caps_free(this->bmp_buffer_);
  }
  this->bmp_buffer_ = new_buffer;
  this->bmp_capacity_ = required_size;
  return true;
}

void TargetDetectionPreview::clear_buffer_() {
  if (this->bmp_buffer_ != nullptr) {
    heap_caps_free(this->bmp_buffer_);
  }
  this->bmp_buffer_ = nullptr;
  this->bmp_size_ = 0;
  this->bmp_capacity_ = 0;
  this->width_ = 0;
  this->height_ = 0;
}

void TargetDetectionPreview::build_bmp_header_(uint16_t width, uint16_t height, size_t row_stride) {
  std::memset(this->bmp_buffer_, 0, this->bmp_size_);
  this->bmp_buffer_[0] = 'B';
  this->bmp_buffer_[1] = 'M';
  write_u32(this->bmp_buffer_, 2, static_cast<uint32_t>(this->bmp_size_));
  write_u32(this->bmp_buffer_, 10, static_cast<uint32_t>(BMP_PIXEL_OFFSET));
  write_u32(this->bmp_buffer_, 14, static_cast<uint32_t>(BMP_DIB_HEADER_SIZE));
  write_u32(this->bmp_buffer_, 18, width);
  write_u32(this->bmp_buffer_, 22, static_cast<uint32_t>(-static_cast<int32_t>(height)));
  write_u16(this->bmp_buffer_, 26, 1);
  write_u16(this->bmp_buffer_, 28, 8);
  write_u32(this->bmp_buffer_, 34, static_cast<uint32_t>(row_stride * height));
  write_u32(this->bmp_buffer_, 38, 2835);
  write_u32(this->bmp_buffer_, 42, 2835);
  write_u32(this->bmp_buffer_, 46, 256);
  write_u32(this->bmp_buffer_, 50, 256);

  uint8_t *palette = this->bmp_buffer_ + BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE;
  for (uint16_t value = 0; value < 256; ++value) {
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = 0;
  }
}

void TargetDetectionPreview::draw_rectangle_(uint8_t *pixels, size_t row_stride,
                                             uint16_t width, uint16_t height,
                                             int x0, int y0, int x1, int y1) const {
  if (pixels == nullptr || width == 0 || height == 0) {
    return;
  }

  x0 = std::max(0, std::min<int>(x0, width - 1));
  y0 = std::max(0, std::min<int>(y0, height - 1));
  x1 = std::max(0, std::min<int>(x1, width - 1));
  y1 = std::max(0, std::min<int>(y1, height - 1));
  if (x1 <= x0 || y1 <= y0) {
    return;
  }

  // Double trait noir/blanc pour rester visible sur fond clair comme sur fond sombre.
  for (int layer = 0; layer < 2; ++layer) {
    const int left = std::max(0, x0 - layer);
    const int top = std::max(0, y0 - layer);
    const int right = std::min<int>(width - 1, x1 + layer);
    const int bottom = std::min<int>(height - 1, y1 + layer);
    const uint8_t value = layer == 0 ? 0 : 255;

    for (int x = left; x <= right; ++x) {
      pixels[static_cast<size_t>(top) * row_stride + x] = value;
      pixels[static_cast<size_t>(bottom) * row_stride + x] = value;
    }
    for (int y = top; y <= bottom; ++y) {
      pixels[static_cast<size_t>(y) * row_stride + left] = value;
      pixels[static_cast<size_t>(y) * row_stride + right] = value;
    }
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
