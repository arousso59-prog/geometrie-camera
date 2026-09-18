#include "jpeg_filtered_diagnostic.h"

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
static const char *const TAG = "jpeg_decode";
constexpr size_t BMP_FILE_HEADER_SIZE = 14;
constexpr size_t BMP_DIB_HEADER_SIZE = 40;
constexpr size_t BMP_PALETTE_SIZE = 256 * 4;
constexpr size_t BMP_PIXEL_OFFSET =
    BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE + BMP_PALETTE_SIZE;
constexpr size_t JPEG_WORK_BUFFER_SIZE = 4096;

struct JpegDecodeContext {
  const uint8_t *jpeg;
  size_t jpeg_size;
  size_t offset;
  uint8_t *pixels;
  size_t row_stride;
  uint16_t width;
  uint16_t height;
};

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

UINT jpeg_input_callback(JDEC *decoder, BYTE *buffer, UINT requested) {
  if (decoder == nullptr || decoder->device == nullptr) return 0;
  auto *context = static_cast<JpegDecodeContext *>(decoder->device);
  if (context->jpeg == nullptr || context->offset >= context->jpeg_size) return 0;

  const size_t remaining = context->jpeg_size - context->offset;
  const UINT available =
      static_cast<UINT>(std::min<size_t>(remaining, requested));
  if (buffer != nullptr && available > 0) {
    std::memcpy(buffer, context->jpeg + context->offset, available);
  }
  context->offset += available;
  return available;
}

UINT jpeg_output_callback(JDEC *decoder, void *bitmap, JRECT *rect) {
  if (decoder == nullptr || decoder->device == nullptr ||
      bitmap == nullptr || rect == nullptr) {
    return 0;
  }

  auto *context = static_cast<JpegDecodeContext *>(decoder->device);
  if (context->pixels == nullptr ||
      rect->right >= context->width || rect->bottom >= context->height) {
    return 0;
  }

  const uint16_t block_width =
      static_cast<uint16_t>(rect->right - rect->left + 1U);
  const uint16_t block_height =
      static_cast<uint16_t>(rect->bottom - rect->top + 1U);
  const uint8_t *rgb = static_cast<const uint8_t *>(bitmap);

  for (uint16_t local_y = 0; local_y < block_height; ++local_y) {
    const uint16_t y = static_cast<uint16_t>(rect->top + local_y);
    uint8_t *destination =
        context->pixels + static_cast<size_t>(y) * context->row_stride +
        rect->left;

    for (uint16_t local_x = 0; local_x < block_width; ++local_x) {
      const uint8_t red = *rgb++;
      const uint8_t green = *rgb++;
      const uint8_t blue = *rgb++;
      const uint32_t luminance =
          77U * red + 150U * green + 29U * blue + 128U;
      destination[local_x] = static_cast<uint8_t>(luminance >> 8);
    }
  }

  return 1;
}
}  // namespace

JpegFilteredDiagnostic::JpegFilteredDiagnostic(JpegDiagnostic *source)
    : source_(source),
      bmp_buffer_(nullptr),
      bmp_size_(0),
      bmp_capacity_(0),
      jpeg_work_buffer_(nullptr),
      width_(0),
      height_(0),
      process_count_(0),
      source_capture_count_(0),
      ready_(false),
      decode_result_(-1),
      decode_ms_(0),
      total_ms_(0) {}

JpegFilteredDiagnostic::~JpegFilteredDiagnostic() { this->clear_buffers_(); }

bool JpegFilteredDiagnostic::process() {
  this->ready_ = false;
  this->decode_result_ = -1;
  this->decode_ms_ = 0;
  this->total_ms_ = 0;

  if (this->source_ == nullptr || !this->source_->ready() ||
      this->source_->jpeg_data() == nullptr ||
      this->source_->jpeg_size() == 0 ||
      !this->source_->has_soi() || !this->source_->has_eoi()) {
    ESP_LOGW(TAG, "Aucun JPEG valide disponible pour le decodage");
    return false;
  }

  const uint16_t width = this->source_->width();
  const uint16_t height = this->source_->height();
  if (width == 0 || height == 0) {
    ESP_LOGE(TAG, "Dimensions JPEG invalides");
    return false;
  }

  if (!this->ensure_buffers_(width, height) ||
      !this->ensure_jpeg_work_buffer_()) {
    ESP_LOGE(TAG, "Allocation impossible pour le decodage JPEG %ux%u",
             static_cast<unsigned>(width),
             static_cast<unsigned>(height));
    return false;
  }

  const uint32_t total_started = millis();
  const size_t row_stride =
      (static_cast<size_t>(width) + 3U) & ~static_cast<size_t>(3U);
  this->build_bmp_header_(width, height, row_stride);

  JpegDecodeContext context{};
  context.jpeg = this->source_->jpeg_data();
  context.jpeg_size = this->source_->jpeg_size();
  context.offset = 0;
  context.pixels = this->bmp_buffer_ + BMP_PIXEL_OFFSET;
  context.row_stride = row_stride;
  context.width = width;
  context.height = height;

  JDEC decoder{};
  const uint32_t decode_started = millis();
  JRESULT result =
      jd_prepare(&decoder, jpeg_input_callback, this->jpeg_work_buffer_,
                 static_cast<UINT>(JPEG_WORK_BUFFER_SIZE), &context);
  if (result == JDR_OK &&
      (decoder.width != width || decoder.height != height)) {
    ESP_LOGE(TAG, "Dimensions decodees inattendues: %ux%u au lieu de %ux%u",
             static_cast<unsigned>(decoder.width),
             static_cast<unsigned>(decoder.height),
             static_cast<unsigned>(width),
             static_cast<unsigned>(height));
    result = JDR_PAR;
  }
  if (result == JDR_OK) {
    result = jd_decomp(&decoder, jpeg_output_callback, 0);
  }

  this->decode_ms_ = millis() - decode_started;
  this->decode_result_ = static_cast<int>(result);
  if (result != JDR_OK) {
    ESP_LOGE(TAG, "Decodage JPEG en echec: %d", static_cast<int>(result));
    return false;
  }

  this->width_ = width;
  this->height_ = height;
  this->source_capture_count_ = this->source_->capture_count();
  this->process_count_++;
  this->total_ms_ = millis() - total_started;
  this->ready_ = true;

  ESP_LOGD(TAG, "JPEG gris pret: %ux%u, decode=%u ms total=%u ms",
           static_cast<unsigned>(this->width_),
           static_cast<unsigned>(this->height_),
           static_cast<unsigned>(this->decode_ms_),
           static_cast<unsigned>(this->total_ms_));
  return true;
}

void JpegFilteredDiagnostic::release_buffers() {
  this->clear_buffers_();
  this->width_ = 0;
  this->height_ = 0;
  this->ready_ = false;
  this->bmp_size_ = 0;
}

bool JpegFilteredDiagnostic::ready() const { return this->ready_; }
uint32_t JpegFilteredDiagnostic::process_count() const { return this->process_count_; }
uint32_t JpegFilteredDiagnostic::source_capture_count() const { return this->source_capture_count_; }
uint16_t JpegFilteredDiagnostic::width() const { return this->width_; }
uint16_t JpegFilteredDiagnostic::height() const { return this->height_; }

const uint8_t *JpegFilteredDiagnostic::grayscale_data() const {
  if (!this->ready_ || this->bmp_buffer_ == nullptr) return nullptr;
  return this->bmp_buffer_ + BMP_PIXEL_OFFSET;
}

size_t JpegFilteredDiagnostic::grayscale_stride() const {
  return (static_cast<size_t>(this->width_) + 3U) &
         ~static_cast<size_t>(3U);
}

const uint8_t *JpegFilteredDiagnostic::bmp_data() const {
  return this->bmp_buffer_;
}
size_t JpegFilteredDiagnostic::bmp_size() const { return this->bmp_size_; }
int JpegFilteredDiagnostic::decode_result() const { return this->decode_result_; }
uint32_t JpegFilteredDiagnostic::decode_ms() const { return this->decode_ms_; }
uint32_t JpegFilteredDiagnostic::total_ms() const { return this->total_ms_; }

bool JpegFilteredDiagnostic::ensure_buffers_(uint16_t width, uint16_t height) {
  const size_t row_stride =
      (static_cast<size_t>(width) + 3U) & ~static_cast<size_t>(3U);
  const size_t required_bmp =
      BMP_PIXEL_OFFSET + row_stride * height;

  if (this->bmp_buffer_ == nullptr ||
      this->bmp_capacity_ < required_bmp) {
    auto *new_bmp = static_cast<uint8_t *>(
        heap_caps_malloc(required_bmp,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (new_bmp == nullptr) return false;
    if (this->bmp_buffer_ != nullptr) heap_caps_free(this->bmp_buffer_);
    this->bmp_buffer_ = new_bmp;
    this->bmp_capacity_ = required_bmp;
  }

  this->bmp_size_ = required_bmp;
  return true;
}

bool JpegFilteredDiagnostic::ensure_jpeg_work_buffer_() {
  if (this->jpeg_work_buffer_ != nullptr) return true;

  this->jpeg_work_buffer_ = static_cast<uint8_t *>(
      heap_caps_malloc(JPEG_WORK_BUFFER_SIZE,
                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (this->jpeg_work_buffer_ == nullptr) {
    this->jpeg_work_buffer_ = static_cast<uint8_t *>(
        heap_caps_malloc(JPEG_WORK_BUFFER_SIZE, MALLOC_CAP_8BIT));
  }
  return this->jpeg_work_buffer_ != nullptr;
}

void JpegFilteredDiagnostic::clear_buffers_() {
  if (this->bmp_buffer_ != nullptr) heap_caps_free(this->bmp_buffer_);
  if (this->jpeg_work_buffer_ != nullptr) heap_caps_free(this->jpeg_work_buffer_);
  this->bmp_buffer_ = nullptr;
  this->bmp_size_ = 0;
  this->bmp_capacity_ = 0;
  this->jpeg_work_buffer_ = nullptr;
}

void JpegFilteredDiagnostic::build_bmp_header_(
    uint16_t width, uint16_t height, size_t row_stride) {
  std::memset(this->bmp_buffer_, 0, BMP_PIXEL_OFFSET);
  this->bmp_buffer_[0] = 'B';
  this->bmp_buffer_[1] = 'M';
  write_u32(this->bmp_buffer_, 2, static_cast<uint32_t>(this->bmp_size_));
  write_u32(this->bmp_buffer_, 10, static_cast<uint32_t>(BMP_PIXEL_OFFSET));

  write_u32(this->bmp_buffer_, 14, static_cast<uint32_t>(BMP_DIB_HEADER_SIZE));
  write_u32(this->bmp_buffer_, 18, width);
  write_u32(this->bmp_buffer_, 22,
            static_cast<uint32_t>(-static_cast<int32_t>(height)));
  write_u16(this->bmp_buffer_, 26, 1);
  write_u16(this->bmp_buffer_, 28, 8);
  write_u32(this->bmp_buffer_, 30, 0);
  write_u32(this->bmp_buffer_, 34,
            static_cast<uint32_t>(row_stride * height));
  write_u32(this->bmp_buffer_, 38, 2835);
  write_u32(this->bmp_buffer_, 42, 2835);
  write_u32(this->bmp_buffer_, 46, 256);
  write_u32(this->bmp_buffer_, 50, 256);

  uint8_t *palette =
      this->bmp_buffer_ + BMP_FILE_HEADER_SIZE + BMP_DIB_HEADER_SIZE;
  for (uint16_t value = 0; value < 256; ++value) {
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = static_cast<uint8_t>(value);
    *palette++ = 0;
  }

  if (row_stride > width) {
    const size_t padding = row_stride - width;
    uint8_t *pixels = this->bmp_buffer_ + BMP_PIXEL_OFFSET;
    for (uint16_t y = 0; y < height; ++y) {
      std::memset(pixels + static_cast<size_t>(y) * row_stride + width,
                  0, padding);
    }
  }
}

}  // namespace geometrie_camera_app
}  // namespace esphome
