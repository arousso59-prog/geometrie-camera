#include "rgb565_diagnostic.h"

#include <cstdlib>

#include "esp_camera.h"
#include "img_converters.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "rgb565_diag";
}

Rgb565Diagnostic::Rgb565Diagnostic()
    : camera_(nullptr),
      bmp_buffer_(nullptr),
      bmp_size_(0),
      source_size_(0),
      width_(0),
      height_(0),
      capture_count_(0),
      last_capture_ms_(0),
      capture_pending_(false),
      ready_(false) {}

Rgb565Diagnostic::~Rgb565Diagnostic() {
  this->clear_buffer_();
}

void Rgb565Diagnostic::set_camera(esp32_camera::ESP32Camera *camera) {
  if (camera == nullptr || this->camera_ == camera) {
    return;
  }

  this->camera_ = camera;
  this->camera_->add_listener(this);
  ESP_LOGI(TAG, "Diagnostic RGB565 relie a ESP32Camera");
}

bool Rgb565Diagnostic::request_capture() {
  if (this->camera_ == nullptr || this->capture_pending_) {
    return false;
  }

  this->capture_pending_ = true;
  this->camera_->request_image(camera::WEB_REQUESTER);
  ESP_LOGD(TAG, "Capture RGB565 brute demandee");
  return true;
}

void Rgb565Diagnostic::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->capture_pending_ || image == nullptr) {
    return;
  }

  auto esp_image = std::static_pointer_cast<esp32_camera::ESP32CameraImage>(image);
  camera_fb_t *frame = esp_image->get_raw_buffer();
  if (frame == nullptr || frame->buf == nullptr) {
    ESP_LOGE(TAG, "Framebuffer RGB565 indisponible");
    this->capture_pending_ = false;
    return;
  }

  if (frame->format != PIXFORMAT_RGB565) {
    ESP_LOGE(TAG, "Format inattendu pour le diagnostic RGB565: %d", static_cast<int>(frame->format));
    this->capture_pending_ = false;
    return;
  }

  const size_t expected_size = static_cast<size_t>(frame->width) * static_cast<size_t>(frame->height) * 2U;
  if (frame->len < expected_size) {
    ESP_LOGE(TAG, "Framebuffer RGB565 trop court: %u < %u", static_cast<unsigned>(frame->len),
             static_cast<unsigned>(expected_size));
    this->capture_pending_ = false;
    return;
  }

  uint8_t *new_bmp = nullptr;
  size_t new_bmp_size = 0;
  if (!frame2bmp(frame, &new_bmp, &new_bmp_size) || new_bmp == nullptr || new_bmp_size == 0) {
    ESP_LOGE(TAG, "Conversion RGB565 -> BMP impossible");
    this->capture_pending_ = false;
    return;
  }

  this->clear_buffer_();
  this->bmp_buffer_ = new_bmp;
  this->bmp_size_ = new_bmp_size;
  this->source_size_ = frame->len;
  this->width_ = frame->width;
  this->height_ = frame->height;
  this->capture_count_++;
  this->last_capture_ms_ = millis();
  this->ready_ = true;
  this->capture_pending_ = false;

  ESP_LOGI(TAG, "Capture RGB565 recue: %ux%u, source %u octets, BMP %u octets",
           static_cast<unsigned>(frame->width), static_cast<unsigned>(frame->height),
           static_cast<unsigned>(frame->len), static_cast<unsigned>(this->bmp_size_));
}

bool Rgb565Diagnostic::ready() const {
  return this->ready_;
}

bool Rgb565Diagnostic::capture_pending() const {
  return this->capture_pending_;
}

uint32_t Rgb565Diagnostic::capture_count() const {
  return this->capture_count_;
}

uint32_t Rgb565Diagnostic::last_capture_ms() const {
  return this->last_capture_ms_;
}

uint16_t Rgb565Diagnostic::width() const {
  return this->width_;
}

uint16_t Rgb565Diagnostic::height() const {
  return this->height_;
}

size_t Rgb565Diagnostic::source_size() const {
  return this->source_size_;
}

const uint8_t *Rgb565Diagnostic::bmp_data() const {
  return this->bmp_buffer_;
}

size_t Rgb565Diagnostic::bmp_size() const {
  return this->bmp_size_;
}

void Rgb565Diagnostic::clear_buffer_() {
  if (this->bmp_buffer_ != nullptr) {
    std::free(this->bmp_buffer_);
  }

  this->bmp_buffer_ = nullptr;
  this->bmp_size_ = 0;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
