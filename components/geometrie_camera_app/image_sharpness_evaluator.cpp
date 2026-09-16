#include "image_sharpness_evaluator.h"

#include <algorithm>
#include <cstring>

#include "esp_heap_caps.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "jpeg_diagnostic.h"
#include "rom/tjpgd.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "image_sharpness";
constexpr size_t JPEG_WORK_BUFFER_SIZE = 4096;
constexpr uint8_t JPEG_SCALE = 3;  // 1/8 dans chaque dimension.

struct SharpnessDecodeContext {
  const uint8_t *jpeg;
  size_t jpeg_size;
  size_t offset;
  uint8_t *pixels;
  uint16_t width;
  uint16_t height;
};

UINT sharpness_input_callback(JDEC *decoder, BYTE *buffer, UINT requested) {
  if (decoder == nullptr || decoder->device == nullptr) {
    return 0;
  }

  auto *context = static_cast<SharpnessDecodeContext *>(decoder->device);
  if (context->jpeg == nullptr || context->offset >= context->jpeg_size) {
    return 0;
  }

  const size_t remaining = context->jpeg_size - context->offset;
  const UINT available = static_cast<UINT>(std::min<size_t>(remaining, requested));
  if (buffer != nullptr && available > 0) {
    std::memcpy(buffer, context->jpeg + context->offset, available);
  }
  context->offset += available;
  return available;
}

UINT sharpness_output_callback(JDEC *decoder, void *bitmap, JRECT *rect) {
  if (decoder == nullptr || decoder->device == nullptr || bitmap == nullptr || rect == nullptr) {
    return 0;
  }

  auto *context = static_cast<SharpnessDecodeContext *>(decoder->device);
  if (context->pixels == nullptr || rect->right >= context->width || rect->bottom >= context->height) {
    return 0;
  }

  const uint16_t block_width = static_cast<uint16_t>(rect->right - rect->left + 1U);
  const uint16_t block_height = static_cast<uint16_t>(rect->bottom - rect->top + 1U);
  const uint8_t *rgb = static_cast<const uint8_t *>(bitmap);

  for (uint16_t local_y = 0; local_y < block_height; ++local_y) {
    const uint16_t y = static_cast<uint16_t>(rect->top + local_y);
    uint8_t *destination = context->pixels + static_cast<size_t>(y) * context->width + rect->left;

    for (uint16_t local_x = 0; local_x < block_width; ++local_x) {
      const uint8_t red = *rgb++;
      const uint8_t green = *rgb++;
      const uint8_t blue = *rgb++;
      const uint32_t luminance = 77U * red + 150U * green + 29U * blue + 128U;
      destination[local_x] = static_cast<uint8_t>(luminance >> 8);
    }
  }

  return 1;
}
}

ImageSharpnessEvaluator::ImageSharpnessEvaluator(JpegDiagnostic *source)
    : source_(source),
      grayscale_buffer_(nullptr),
      grayscale_capacity_(0),
      jpeg_work_buffer_(nullptr),
      width_(0),
      height_(0),
      score_x100_(0),
      evaluation_ms_(0),
      ready_(false) {}

ImageSharpnessEvaluator::~ImageSharpnessEvaluator() { this->clear_buffers_(); }

bool ImageSharpnessEvaluator::evaluate() {
  this->ready_ = false;
  this->score_x100_ = 0;
  this->evaluation_ms_ = 0;

  if (this->source_ == nullptr || !this->source_->ready() || this->source_->jpeg_data() == nullptr ||
      this->source_->jpeg_size() == 0 || !this->source_->has_soi() || !this->source_->has_eoi()) {
    return false;
  }

  if (this->jpeg_work_buffer_ == nullptr) {
    this->jpeg_work_buffer_ = static_cast<uint8_t *>(
        heap_caps_malloc(JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (this->jpeg_work_buffer_ == nullptr) {
      this->jpeg_work_buffer_ = static_cast<uint8_t *>(heap_caps_malloc(JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_8BIT));
    }
    if (this->jpeg_work_buffer_ == nullptr) {
      return false;
    }
  }

  const uint32_t started_ms = millis();
  SharpnessDecodeContext context{};
  context.jpeg = this->source_->jpeg_data();
  context.jpeg_size = this->source_->jpeg_size();
  context.offset = 0;

  JDEC decoder{};
  JRESULT result = jd_prepare(&decoder, sharpness_input_callback, this->jpeg_work_buffer_,
                              static_cast<UINT>(JPEG_WORK_BUFFER_SIZE), &context);
  if (result != JDR_OK) {
    return false;
  }

  const uint16_t width = static_cast<uint16_t>(decoder.width >> JPEG_SCALE);
  const uint16_t height = static_cast<uint16_t>(decoder.height >> JPEG_SCALE);
  if (width < 3 || height < 3 || !this->ensure_buffers_(width, height)) {
    return false;
  }

  context.pixels = this->grayscale_buffer_;
  context.width = width;
  context.height = height;
  std::memset(this->grayscale_buffer_, 0, static_cast<size_t>(width) * height);

  result = jd_decomp(&decoder, sharpness_output_callback, JPEG_SCALE);
  if (result != JDR_OK) {
    ESP_LOGW(TAG, "Evaluation nettete: decodage JPEG reduit en echec (%d)", static_cast<int>(result));
    return false;
  }

  this->width_ = width;
  this->height_ = height;
  this->score_x100_ = this->compute_score_x100_();
  this->evaluation_ms_ = millis() - started_ms;
  this->ready_ = true;

  ESP_LOGD(TAG, "Nettete JPEG reduite %ux%u: score_x100=%u, temps=%u ms",
           static_cast<unsigned>(this->width_), static_cast<unsigned>(this->height_),
           static_cast<unsigned>(this->score_x100_), static_cast<unsigned>(this->evaluation_ms_));
  return true;
}

bool ImageSharpnessEvaluator::ready() const { return this->ready_; }
uint32_t ImageSharpnessEvaluator::score_x100() const { return this->score_x100_; }
uint32_t ImageSharpnessEvaluator::evaluation_ms() const { return this->evaluation_ms_; }
uint16_t ImageSharpnessEvaluator::preview_width() const { return this->width_; }
uint16_t ImageSharpnessEvaluator::preview_height() const { return this->height_; }

bool ImageSharpnessEvaluator::ensure_buffers_(uint16_t width, uint16_t height) {
  const size_t required = static_cast<size_t>(width) * height;
  if (this->grayscale_buffer_ != nullptr && this->grayscale_capacity_ >= required) {
    return true;
  }

  auto *new_buffer = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (new_buffer == nullptr) {
    new_buffer = static_cast<uint8_t *>(heap_caps_malloc(required, MALLOC_CAP_8BIT));
  }
  if (new_buffer == nullptr) {
    return false;
  }

  if (this->grayscale_buffer_ != nullptr) {
    heap_caps_free(this->grayscale_buffer_);
  }
  this->grayscale_buffer_ = new_buffer;
  this->grayscale_capacity_ = required;
  return true;
}

void ImageSharpnessEvaluator::clear_buffers_() {
  if (this->grayscale_buffer_ != nullptr) {
    heap_caps_free(this->grayscale_buffer_);
  }
  if (this->jpeg_work_buffer_ != nullptr) {
    heap_caps_free(this->jpeg_work_buffer_);
  }
  this->grayscale_buffer_ = nullptr;
  this->grayscale_capacity_ = 0;
  this->jpeg_work_buffer_ = nullptr;
}

uint32_t ImageSharpnessEvaluator::compute_score_x100_() const {
  if (this->grayscale_buffer_ == nullptr || this->width_ < 3 || this->height_ < 3) {
    return 0;
  }

  uint64_t sum = 0;
  uint32_t count = 0;
  const size_t stride = this->width_;

  for (uint16_t y = 1; y + 1 < this->height_; ++y) {
    const uint8_t *row = this->grayscale_buffer_ + static_cast<size_t>(y) * stride;
    const uint8_t *row_up = row - stride;
    const uint8_t *row_down = row + stride;
    for (uint16_t x = 1; x + 1 < this->width_; ++x) {
      const int center = row[x];
      int laplacian = 4 * center - row[x - 1] - row[x + 1] - row_up[x] - row_down[x];
      if (laplacian < 0) {
        laplacian = -laplacian;
      }
      sum += static_cast<uint32_t>(laplacian);
      count++;
    }
  }

  if (count == 0) {
    return 0;
  }
  return static_cast<uint32_t>((sum * 100U) / count);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
