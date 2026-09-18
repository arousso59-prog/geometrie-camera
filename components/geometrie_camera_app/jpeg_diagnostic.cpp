#include "jpeg_diagnostic.h"

#include <cstring>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "jpeg_diag";
}

JpegDiagnostic::JpegDiagnostic()
    : camera_(nullptr),
      jpeg_buffer_(nullptr),
      jpeg_size_(0),
      jpeg_capacity_(0),
      width_(0),
      height_(0),
      has_soi_(false),
      has_eoi_(false),
      capture_count_(0),
      discarded_frame_count_(0),
      last_capture_ms_(0),
      capture_pending_(false),
      discard_next_frame_(false),
      fresh_request_pending_(false),
      force_post_request_frame_(true),
      ready_(false),
      request_started_ms_(0),
      stale_frame_received_ms_(0),
      fresh_request_started_ms_(0),
      frame_received_ms_(0),
      acquisition_ms_(0),
      copy_ms_(0),
      total_cycle_ms_(0) {}

JpegDiagnostic::~JpegDiagnostic() {
  this->clear_buffer_();
}

void JpegDiagnostic::set_camera(esp32_camera::ESP32Camera *camera) {
  if (camera == nullptr || this->camera_ == camera) {
    return;
  }

  this->camera_ = camera;
  this->camera_->add_listener(this);
  ESP_LOGI(TAG, "Diagnostic JPEG relie a ESP32Camera");
}

void JpegDiagnostic::release_buffer() {
  if (this->capture_pending_) {
    return;
  }
  this->clear_buffer_();
  this->width_ = 0;
  this->height_ = 0;
  this->has_soi_ = false;
  this->has_eoi_ = false;
  this->ready_ = false;
}

void JpegDiagnostic::cancel_capture() {
  if (!this->capture_pending_) {
    return;
  }

  // Une requete deja remise au driver camera ne peut pas etre retiree, mais
  // on peut invalider immediatement notre sequence logique. Le callback
  // eventuel de cette image sera ignore par on_camera_image() et aucune frame
  // fraiche supplementaire ne sera demandee par loop().
  this->capture_pending_ = false;
  this->discard_next_frame_ = false;
  this->fresh_request_pending_ = false;
  this->force_post_request_frame_ = true;
  this->ready_ = false;
  this->request_started_ms_ = 0;
  this->stale_frame_received_ms_ = 0;
  this->fresh_request_started_ms_ = 0;
  this->frame_received_ms_ = 0;
  this->acquisition_ms_ = 0;
  this->copy_ms_ = 0;
  this->total_cycle_ms_ = 0;

  ESP_LOGI(TAG, "Capture JPEG en cours annulee");
}

bool JpegDiagnostic::request_capture(bool force_post_request_frame) {
  if (this->camera_ == nullptr || this->capture_pending_) {
    return false;
  }

  this->request_started_ms_ = millis();
  this->stale_frame_received_ms_ = 0;
  this->fresh_request_started_ms_ = 0;
  this->frame_received_ms_ = 0;
  this->acquisition_ms_ = 0;
  this->copy_ms_ = 0;
  this->total_cycle_ms_ = 0;

  // Une nouvelle acquisition invalide immediatement l'ancienne image pour les
  // traitements. Le buffer peut rester alloue, mais aucun filtre/detecteur ne
  // doit pouvoir le reutiliser tant qu'une nouvelle frame fraiche n'a pas ete
  // effectivement copiee et marquee ready_ dans on_camera_image().
  this->ready_ = false;
  this->capture_pending_ = true;
  this->force_post_request_frame_ = force_post_request_frame;
  this->discard_next_frame_ = force_post_request_frame;
  this->fresh_request_pending_ = false;

  // ESPHome pre-acquires en permanence la frame suivante dans sa tache camera.
  // Apres un changement de viewport/reglage, cette frame peut appartenir a
  // l'ancien etat : on la purge alors obligatoirement.
  //
  // En regime continu stable, elle est au contraire exactement la prochaine
  // frame sequentielle, capturee pendant le decodage/detection du cycle
  // precedent. L'accepter permet de chevaucher acquisition et traitement sans
  // modifier la camera ni la precision de l'image.
  this->camera_->request_image(camera::WEB_REQUESTER);
  ESP_LOGD(TAG,
           "Capture JPEG demandee a %u ms; strategie=%s",
           static_cast<unsigned>(this->request_started_ms_),
           force_post_request_frame ? "fresh_after_request" : "pipelined_next_frame");
  return true;
}

void JpegDiagnostic::loop() {
  if (!this->capture_pending_ || !this->fresh_request_pending_ || this->camera_ == nullptr) {
    return;
  }

  this->fresh_request_pending_ = false;
  this->fresh_request_started_ms_ = millis();
  this->camera_->request_image(camera::WEB_REQUESTER);
  ESP_LOGD(TAG, "Frame JPEG fraiche demandee a %u ms",
           static_cast<unsigned>(this->fresh_request_started_ms_));
}

void JpegDiagnostic::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->capture_pending_ || image == nullptr) {
    return;
  }

  auto esp_image = std::static_pointer_cast<esp32_camera::ESP32CameraImage>(image);
  camera_fb_t *frame = esp_image->get_raw_buffer();
  if (frame == nullptr || frame->buf == nullptr || frame->len == 0) {
    ESP_LOGE(TAG, "Framebuffer JPEG indisponible");
    this->capture_pending_ = false;
    this->discard_next_frame_ = false;
    this->fresh_request_pending_ = false;
    return;
  }

  if (frame->format != PIXFORMAT_JPEG) {
    ESP_LOGE(TAG, "Format inattendu pour le diagnostic JPEG: %d", static_cast<int>(frame->format));
    this->capture_pending_ = false;
    this->discard_next_frame_ = false;
    this->fresh_request_pending_ = false;
    return;
  }

  if (this->discard_next_frame_) {
    this->stale_frame_received_ms_ = millis();
    this->discard_next_frame_ = false;
    this->fresh_request_pending_ = true;
    this->discarded_frame_count_++;

    ESP_LOGD(TAG, "Frame JPEG en attente purgee: %ux%u, %u octets",
             static_cast<unsigned>(frame->width), static_cast<unsigned>(frame->height),
             static_cast<unsigned>(frame->len));
    return;
  }

  this->frame_received_ms_ = millis();
  if (this->fresh_request_started_ms_ != 0) {
    this->acquisition_ms_ = this->frame_received_ms_ - this->fresh_request_started_ms_;
  } else {
    this->acquisition_ms_ = this->frame_received_ms_ - this->request_started_ms_;
  }

  const uint32_t copy_started_ms = millis();
  if (!this->ensure_buffer_(frame->len)) {
    ESP_LOGE(TAG, "Allocation PSRAM impossible pour JPEG de %u octets", static_cast<unsigned>(frame->len));
    this->capture_pending_ = false;
    this->ready_ = false;
    return;
  }

  std::memcpy(this->jpeg_buffer_, frame->buf, frame->len);
  this->jpeg_size_ = frame->len;
  this->width_ = frame->width;
  this->height_ = frame->height;
  this->has_soi_ = this->jpeg_size_ >= 2 && this->jpeg_buffer_[0] == 0xFF && this->jpeg_buffer_[1] == 0xD8;
  this->has_eoi_ = this->jpeg_size_ >= 2 && this->jpeg_buffer_[this->jpeg_size_ - 2] == 0xFF &&
                   this->jpeg_buffer_[this->jpeg_size_ - 1] == 0xD9;
  this->copy_ms_ = millis() - copy_started_ms;
  this->capture_count_++;
  this->last_capture_ms_ = millis();
  this->total_cycle_ms_ = this->last_capture_ms_ - this->request_started_ms_;
  this->ready_ = true;
  this->capture_pending_ = false;
  this->fresh_request_pending_ = false;

  ESP_LOGI(TAG,
           "JPEG frais recu: %ux%u, %u octets, SOI=%s EOI=%s, acquisition=%u ms, copie=%u ms, total=%u ms",
           static_cast<unsigned>(this->width_), static_cast<unsigned>(this->height_),
           static_cast<unsigned>(this->jpeg_size_), this->has_soi_ ? "OK" : "NON",
           this->has_eoi_ ? "OK" : "NON", static_cast<unsigned>(this->acquisition_ms_),
           static_cast<unsigned>(this->copy_ms_), static_cast<unsigned>(this->total_cycle_ms_));
}

bool JpegDiagnostic::ready() const { return this->ready_; }
bool JpegDiagnostic::capture_pending() const { return this->capture_pending_; }
uint32_t JpegDiagnostic::capture_count() const { return this->capture_count_; }
uint32_t JpegDiagnostic::discarded_frame_count() const { return this->discarded_frame_count_; }
uint32_t JpegDiagnostic::last_capture_ms() const { return this->last_capture_ms_; }
uint16_t JpegDiagnostic::width() const { return this->width_; }
uint16_t JpegDiagnostic::height() const { return this->height_; }
const uint8_t *JpegDiagnostic::jpeg_data() const { return this->jpeg_buffer_; }
size_t JpegDiagnostic::jpeg_size() const { return this->jpeg_size_; }
bool JpegDiagnostic::has_soi() const { return this->has_soi_; }
bool JpegDiagnostic::has_eoi() const { return this->has_eoi_; }
uint32_t JpegDiagnostic::request_started_ms() const { return this->request_started_ms_; }
uint32_t JpegDiagnostic::stale_frame_received_ms() const { return this->stale_frame_received_ms_; }
uint32_t JpegDiagnostic::fresh_request_started_ms() const { return this->fresh_request_started_ms_; }
uint32_t JpegDiagnostic::frame_received_ms() const { return this->frame_received_ms_; }
uint32_t JpegDiagnostic::acquisition_ms() const { return this->acquisition_ms_; }
uint32_t JpegDiagnostic::copy_ms() const { return this->copy_ms_; }
uint32_t JpegDiagnostic::total_cycle_ms() const { return this->total_cycle_ms_; }

bool JpegDiagnostic::ensure_buffer_(size_t required_size) {
  if (this->jpeg_buffer_ != nullptr && this->jpeg_capacity_ >= required_size) {
    return true;
  }

  uint8_t *new_buffer = static_cast<uint8_t *>(
      heap_caps_malloc(required_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (new_buffer == nullptr) {
    new_buffer = static_cast<uint8_t *>(heap_caps_malloc(required_size, MALLOC_CAP_8BIT));
  }
  if (new_buffer == nullptr) {
    return false;
  }

  this->clear_buffer_();
  this->jpeg_buffer_ = new_buffer;
  this->jpeg_capacity_ = required_size;
  return true;
}

void JpegDiagnostic::clear_buffer_() {
  if (this->jpeg_buffer_ != nullptr) {
    heap_caps_free(this->jpeg_buffer_);
  }

  this->jpeg_buffer_ = nullptr;
  this->jpeg_size_ = 0;
  this->jpeg_capacity_ = 0;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
