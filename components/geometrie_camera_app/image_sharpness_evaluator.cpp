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
constexpr uint8_t JPEG_SCALE = 2;  // 1/4 dans chaque dimension pour conserver du detail sur la cible.
constexpr uint16_t LAPLACIAN_HISTOGRAM_BINS = 256;
constexpr uint8_t LAPLACIAN_BIN_SHIFT = 2;  // |Laplacien| 0..1020 -> histogramme 0..255.
constexpr uint32_t STRONG_EDGE_PERCENT = 20;

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

bool ImageSharpnessEvaluator::evaluate_region(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
  this->ready_ = false;
  this->score_x100_ = 0;
  this->evaluation_ms_ = 0;

  if (this->source_ == nullptr || !this->source_->ready() || this->source_->jpeg_data() == nullptr ||
      this->source_->jpeg_size() == 0 || !this->source_->has_soi() || !this->source_->has_eoi()) {
    return false;
  }

  const uint16_t source_width = this->source_->width();
  const uint16_t source_height = this->source_->height();
  if (source_width == 0 || source_height == 0 || width == 0 || height == 0 || x >= source_width || y >= source_height) {
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

  const uint16_t reduced_width = static_cast<uint16_t>(decoder.width >> JPEG_SCALE);
  const uint16_t reduced_height = static_cast<uint16_t>(decoder.height >> JPEG_SCALE);
  if (reduced_width < 3 || reduced_height < 3 || !this->ensure_buffers_(reduced_width, reduced_height)) {
    return false;
  }

  context.pixels = this->grayscale_buffer_;
  context.width = reduced_width;
  context.height = reduced_height;
  std::memset(this->grayscale_buffer_, 0, static_cast<size_t>(reduced_width) * reduced_height);

  result = jd_decomp(&decoder, sharpness_output_callback, JPEG_SCALE);
  if (result != JDR_OK) {
    ESP_LOGW(TAG, "Evaluation nettete ROI: decodage JPEG reduit en echec (%d)", static_cast<int>(result));
    return false;
  }

  this->width_ = reduced_width;
  this->height_ = reduced_height;

  const uint32_t scale = 1U << JPEG_SCALE;
  const uint16_t roi_x = static_cast<uint16_t>(x / scale);
  const uint16_t roi_y = static_cast<uint16_t>(y / scale);
  const uint16_t roi_right = static_cast<uint16_t>(std::min<uint32_t>(
      reduced_width, (static_cast<uint32_t>(x) + width + scale - 1U) / scale));
  const uint16_t roi_bottom = static_cast<uint16_t>(std::min<uint32_t>(
      reduced_height, (static_cast<uint32_t>(y) + height + scale - 1U) / scale));

  if (roi_right <= roi_x + 2U || roi_bottom <= roi_y + 2U) {
    return false;
  }

  this->score_x100_ = this->compute_score_x100_(
      roi_x, roi_y, static_cast<uint16_t>(roi_right - roi_x), static_cast<uint16_t>(roi_bottom - roi_y));
  this->evaluation_ms_ = millis() - started_ms;
  this->ready_ = true;

  ESP_LOGD(TAG, "Nettete ROI reduite %ux%u @(%u,%u) %ux%u: score_edges_x100=%u, temps=%u ms",
           static_cast<unsigned>(this->width_), static_cast<unsigned>(this->height_),
           static_cast<unsigned>(roi_x), static_cast<unsigned>(roi_y),
           static_cast<unsigned>(roi_right - roi_x), static_cast<unsigned>(roi_bottom - roi_y),
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

uint32_t ImageSharpnessEvaluator::compute_score_x100_(uint16_t x, uint16_t y,
                                                       uint16_t width, uint16_t height) const {
  if (this->grayscale_buffer_ == nullptr || this->width_ < 3 || this->height_ < 3 || width < 3 || height < 3) {
    return 0;
  }

  const uint16_t start_x = std::max<uint16_t>(1, x);
  const uint16_t start_y = std::max<uint16_t>(1, y);
  const uint16_t end_x = std::min<uint16_t>(static_cast<uint16_t>(this->width_ - 1U),
                                            static_cast<uint16_t>(x + width));
  const uint16_t end_y = std::min<uint16_t>(static_cast<uint16_t>(this->height_ - 1U),
                                            static_cast<uint16_t>(y + height));
  if (end_x <= start_x || end_y <= start_y) {
    return 0;
  }

  // On ne moyenne plus tout le mur contenu dans la ROI. Un histogramme compact
  // permet de ne conserver que les 20 % de reponses Laplaciennes les plus fortes,
  // donc principalement les transitions noir/blanc de la cible.
  uint16_t histogram[LAPLACIAN_HISTOGRAM_BINS] = {};
  uint32_t count = 0;
  const size_t stride = this->width_;

  for (uint16_t py = start_y; py < end_y; ++py) {
    const uint8_t *row = this->grayscale_buffer_ + static_cast<size_t>(py) * stride;
    const uint8_t *row_up = row - stride;
    const uint8_t *row_down = row + stride;
    for (uint16_t px = start_x; px < end_x; ++px) {
      const int center = row[px];
      int laplacian = 4 * center - row[px - 1] - row[px + 1] - row_up[px] - row_down[px];
      if (laplacian < 0) {
        laplacian = -laplacian;
      }
      const uint16_t bin = static_cast<uint16_t>(std::min<int>(
          LAPLACIAN_HISTOGRAM_BINS - 1U, laplacian >> LAPLACIAN_BIN_SHIFT));
      if (histogram[bin] != 0xFFFFU) {
        histogram[bin]++;
      }
      count++;
    }
  }

  if (count == 0) {
    return 0;
  }

  const uint32_t wanted = std::max<uint32_t>(1U, (count * STRONG_EDGE_PERCENT + 99U) / 100U);
  uint32_t remaining = wanted;
  uint64_t weighted_sum = 0;

  for (int bin = static_cast<int>(LAPLACIAN_HISTOGRAM_BINS) - 1; bin >= 0 && remaining > 0; --bin) {
    const uint32_t available = histogram[bin];
    if (available == 0) {
      continue;
    }
    const uint32_t take = std::min<uint32_t>(available, remaining);
    const uint32_t approximated_laplacian = (static_cast<uint32_t>(bin) << LAPLACIAN_BIN_SHIFT) +
                                             (1U << (LAPLACIAN_BIN_SHIFT - 1U));
    weighted_sum += static_cast<uint64_t>(approximated_laplacian) * take;
    remaining -= take;
  }

  const uint32_t selected = wanted - remaining;
  if (selected == 0) {
    return 0;
  }
  return static_cast<uint32_t>((weighted_sum * 100U) / selected);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
