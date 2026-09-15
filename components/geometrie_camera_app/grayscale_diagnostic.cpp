#include "grayscale_diagnostic.h"

#include <cstring>

#include "esp_heap_caps.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "grayscale_diag";
constexpr size_t BMP_FILE_HEADER_SIZE = 14;
constexpr size_t BMP_INFO_HEADER_SIZE = 40;
constexpr size_t BMP_PALETTE_SIZE = 256 * 4;
constexpr size_t BMP_PIXEL_OFFSET = BMP_FILE_HEADER_SIZE + BMP_INFO_HEADER_SIZE + BMP_PALETTE_SIZE;

void write_u16_le(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void write_u32_le(uint8_t *buffer, size_t offset, uint32_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}
}  // namespace

GrayscaleDiagnostic::GrayscaleDiagnostic()
    : camera_(nullptr),
      bmp_buffer_(nullptr),
      bmp_size_(0),
      bmp_capacity_(0),
      width_(0),
      height_(0),
      capture_count_(0),
      last_capture_ms_(0),
      capture_pending_(false),
      ready_(false),
      request_started_ms_(0),
      frame_received_ms_(0),
      acquisition_ms_(0),
      diagnostic_processing_ms_(0),
      total_cycle_ms_(0),
      raw_min_(0),
      raw_max_(0),
      raw_mean_(0.0f),
      raw_zero_count_(0),
      raw_full_count_(0),
      raw_pixel_count_(0) {}

GrayscaleDiagnostic::~GrayscaleDiagnostic() {
  this->clear_buffer_();
}

void GrayscaleDiagnostic::set_camera(esp32_camera::ESP32Camera *camera) {
  if (camera == nullptr || this->camera_ == camera) {
    return;
  }

  this->camera_ = camera;
  this->camera_->add_listener(this);
  ESP_LOGI(TAG, "Diagnostic grayscale relie a ESP32Camera");
}

bool GrayscaleDiagnostic::request_capture() {
  if (this->camera_ == nullptr || this->capture_pending_) {
    return false;
  }

  this->request_started_ms_ = millis();
  this->frame_received_ms_ = 0;
  this->acquisition_ms_ = 0;
  this->diagnostic_processing_ms_ = 0;
  this->total_cycle_ms_ = 0;
  this->capture_pending_ = true;
  this->camera_->request_image(camera::WEB_REQUESTER);
  ESP_LOGD(TAG, "Capture grayscale brute demandee a %u ms", static_cast<unsigned>(this->request_started_ms_));
  return true;
}

void GrayscaleDiagnostic::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->capture_pending_ || image == nullptr) {
    return;
  }

  this->frame_received_ms_ = millis();
  this->acquisition_ms_ = this->frame_received_ms_ - this->request_started_ms_;
  const uint32_t processing_started_ms = this->frame_received_ms_;

  auto esp_image = std::static_pointer_cast<esp32_camera::ESP32CameraImage>(image);
  camera_fb_t *frame = esp_image->get_raw_buffer();
  if (frame == nullptr || frame->buf == nullptr) {
    ESP_LOGE(TAG, "Framebuffer grayscale indisponible");
    this->capture_pending_ = false;
    return;
  }

  if (frame->format != PIXFORMAT_GRAYSCALE) {
    ESP_LOGE(TAG, "Format inattendu pour le diagnostic: %d", static_cast<int>(frame->format));
    this->capture_pending_ = false;
    return;
  }

  const size_t expected_pixels = static_cast<size_t>(frame->width) * static_cast<size_t>(frame->height);
  if (frame->len < expected_pixels) {
    ESP_LOGE(TAG, "Framebuffer trop court pour statistiques: %u < %u", static_cast<unsigned>(frame->len),
             static_cast<unsigned>(expected_pixels));
    this->capture_pending_ = false;
    return;
  }

  this->calculate_statistics_(frame->buf, expected_pixels);

  if (!this->build_bmp_(frame->buf, frame->len, frame->width, frame->height)) {
    ESP_LOGE(TAG, "Construction BMP diagnostic impossible");
    this->capture_pending_ = false;
    return;
  }

  this->width_ = frame->width;
  this->height_ = frame->height;
  this->capture_count_++;
  this->last_capture_ms_ = millis();
  this->diagnostic_processing_ms_ = this->last_capture_ms_ - processing_started_ms;
  this->total_cycle_ms_ = this->last_capture_ms_ - this->request_started_ms_;
  this->ready_ = true;
  this->capture_pending_ = false;

  ESP_LOGI(TAG, "Capture brute recue: %ux%u, %u octets source, BMP %u octets",
           static_cast<unsigned>(frame->width), static_cast<unsigned>(frame->height),
           static_cast<unsigned>(frame->len), static_cast<unsigned>(this->bmp_size_));
  ESP_LOGI(TAG, "Temps: acquisition=%u ms, diagnostic=%u ms, total=%u ms",
           static_cast<unsigned>(this->acquisition_ms_),
           static_cast<unsigned>(this->diagnostic_processing_ms_),
           static_cast<unsigned>(this->total_cycle_ms_));
  ESP_LOGI(TAG, "Stats brutes: min=%u max=%u moyenne=%.2f zero=%u full255=%u pixels=%u",
           static_cast<unsigned>(this->raw_min_), static_cast<unsigned>(this->raw_max_),
           static_cast<double>(this->raw_mean_), static_cast<unsigned>(this->raw_zero_count_),
           static_cast<unsigned>(this->raw_full_count_), static_cast<unsigned>(this->raw_pixel_count_));
}

bool GrayscaleDiagnostic::ready() const {
  return this->ready_;
}

bool GrayscaleDiagnostic::capture_pending() const {
  return this->capture_pending_;
}

uint32_t GrayscaleDiagnostic::capture_count() const {
  return this->capture_count_;
}

uint32_t GrayscaleDiagnostic::last_capture_ms() const {
  return this->last_capture_ms_;
}

uint16_t GrayscaleDiagnostic::width() const {
  return this->width_;
}

uint16_t GrayscaleDiagnostic::height() const {
  return this->height_;
}

const uint8_t *GrayscaleDiagnostic::bmp_data() const {
  return this->bmp_buffer_;
}

size_t GrayscaleDiagnostic::bmp_size() const {
  return this->bmp_size_;
}

uint32_t GrayscaleDiagnostic::request_started_ms() const {
  return this->request_started_ms_;
}

uint32_t GrayscaleDiagnostic::frame_received_ms() const {
  return this->frame_received_ms_;
}

uint32_t GrayscaleDiagnostic::acquisition_ms() const {
  return this->acquisition_ms_;
}

uint32_t GrayscaleDiagnostic::diagnostic_processing_ms() const {
  return this->diagnostic_processing_ms_;
}

uint32_t GrayscaleDiagnostic::total_cycle_ms() const {
  return this->total_cycle_ms_;
}

uint8_t GrayscaleDiagnostic::raw_min() const {
  return this->raw_min_;
}

uint8_t GrayscaleDiagnostic::raw_max() const {
  return this->raw_max_;
}

float GrayscaleDiagnostic::raw_mean() const {
  return this->raw_mean_;
}

uint32_t GrayscaleDiagnostic::raw_zero_count() const {
  return this->raw_zero_count_;
}

uint32_t GrayscaleDiagnostic::raw_full_count() const {
  return this->raw_full_count_;
}

size_t GrayscaleDiagnostic::raw_pixel_count() const {
  return this->raw_pixel_count_;
}

bool GrayscaleDiagnostic::build_bmp_(const uint8_t *grayscale, size_t grayscale_size, uint16_t width,
                                     uint16_t height) {
  if (grayscale == nullptr || width == 0 || height == 0) {
    return false;
  }

  const size_t source_size = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (grayscale_size < source_size) {
    ESP_LOGE(TAG, "Framebuffer trop court: %u < %u", static_cast<unsigned>(grayscale_size),
             static_cast<unsigned>(source_size));
    return false;
  }

  const size_t row_stride = (static_cast<size_t>(width) + 3U) & ~static_cast<size_t>(3U);
  const size_t pixel_data_size = row_stride * static_cast<size_t>(height);
  const size_t file_size = BMP_PIXEL_OFFSET + pixel_data_size;

  if (!this->ensure_buffer_(file_size)) {
    return false;
  }

  std::memset(this->bmp_buffer_, 0, file_size);

  this->bmp_buffer_[0] = 'B';
  this->bmp_buffer_[1] = 'M';
  write_u32_le(this->bmp_buffer_, 2, static_cast<uint32_t>(file_size));
  write_u32_le(this->bmp_buffer_, 10, static_cast<uint32_t>(BMP_PIXEL_OFFSET));

  write_u32_le(this->bmp_buffer_, 14, static_cast<uint32_t>(BMP_INFO_HEADER_SIZE));
  write_u32_le(this->bmp_buffer_, 18, static_cast<uint32_t>(width));
  write_u32_le(this->bmp_buffer_, 22, static_cast<uint32_t>(height));
  write_u16_le(this->bmp_buffer_, 26, 1);
  write_u16_le(this->bmp_buffer_, 28, 8);
  write_u32_le(this->bmp_buffer_, 34, static_cast<uint32_t>(pixel_data_size));
  write_u32_le(this->bmp_buffer_, 46, 256);
  write_u32_le(this->bmp_buffer_, 50, 256);

  uint8_t *palette = this->bmp_buffer_ + BMP_FILE_HEADER_SIZE + BMP_INFO_HEADER_SIZE;
  for (size_t i = 0; i < 256; i++) {
    const size_t offset = i * 4;
    const uint8_t value = static_cast<uint8_t>(i);
    palette[offset] = value;
    palette[offset + 1] = value;
    palette[offset + 2] = value;
    palette[offset + 3] = 0;
  }

  uint8_t *pixels = this->bmp_buffer_ + BMP_PIXEL_OFFSET;
  for (size_t source_y = 0; source_y < height; source_y++) {
    const size_t destination_y = static_cast<size_t>(height) - 1U - source_y;
    uint8_t *destination = pixels + destination_y * row_stride;
    const uint8_t *source = grayscale + source_y * static_cast<size_t>(width);
    std::memcpy(destination, source, width);
  }

  this->bmp_size_ = file_size;
  return true;
}

void GrayscaleDiagnostic::calculate_statistics_(const uint8_t *grayscale, size_t pixel_count) {
  this->raw_pixel_count_ = pixel_count;
  this->raw_zero_count_ = 0;
  this->raw_full_count_ = 0;
  this->raw_mean_ = 0.0f;

  if (grayscale == nullptr || pixel_count == 0) {
    this->raw_min_ = 0;
    this->raw_max_ = 0;
    return;
  }

  uint8_t minimum = 255;
  uint8_t maximum = 0;
  uint64_t sum = 0;

  for (size_t i = 0; i < pixel_count; i++) {
    const uint8_t value = grayscale[i];
    if (value < minimum) {
      minimum = value;
    }
    if (value > maximum) {
      maximum = value;
    }
    if (value == 0) {
      this->raw_zero_count_++;
    }
    if (value == 255) {
      this->raw_full_count_++;
    }
    sum += value;
  }

  this->raw_min_ = minimum;
  this->raw_max_ = maximum;
  this->raw_mean_ = static_cast<float>(sum) / static_cast<float>(pixel_count);
}

bool GrayscaleDiagnostic::ensure_buffer_(size_t required_size) {
  if (this->bmp_buffer_ != nullptr && this->bmp_capacity_ >= required_size) {
    return true;
  }

  this->clear_buffer_();

  this->bmp_buffer_ = static_cast<uint8_t *>(heap_caps_malloc(required_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->bmp_buffer_ == nullptr) {
    ESP_LOGW(TAG, "Allocation PSRAM BMP impossible, tentative heap 8-bit");
    this->bmp_buffer_ = static_cast<uint8_t *>(heap_caps_malloc(required_size, MALLOC_CAP_8BIT));
  }

  if (this->bmp_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Allocation de %u octets impossible", static_cast<unsigned>(required_size));
    return false;
  }

  this->bmp_capacity_ = required_size;
  return true;
}

void GrayscaleDiagnostic::clear_buffer_() {
  if (this->bmp_buffer_ != nullptr) {
    heap_caps_free(this->bmp_buffer_);
  }

  this->bmp_buffer_ = nullptr;
  this->bmp_size_ = 0;
  this->bmp_capacity_ = 0;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
