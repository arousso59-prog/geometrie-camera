#include "target_search_diagnostic.h"

#include <algorithm>

#include "esp_camera.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "grayscale_diagnostic.h"
#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_search_diag";
constexpr uint8_t BOX_THICKNESS = 3;
}

TargetSearchDiagnostic::TargetSearchDiagnostic(TargetDetector *detector, GrayscaleDiagnostic *visualization)
    : camera_(nullptr),
      detector_(detector),
      visualization_(visualization),
      last_observation_(),
      ready_(false),
      search_pending_(false),
      search_count_(0),
      request_started_ms_(0),
      frame_received_ms_(0),
      acquisition_ms_(0),
      detection_ms_(0),
      visualization_ms_(0),
      total_cycle_ms_(0) {}

void TargetSearchDiagnostic::set_camera(esp32_camera::ESP32Camera *camera) {
  if (camera == nullptr || this->camera_ == camera) {
    return;
  }

  this->camera_ = camera;
  this->camera_->add_listener(this);
  ESP_LOGI(TAG, "Diagnostic recherche cible relie a ESP32Camera");
}

bool TargetSearchDiagnostic::request_search() {
  if (this->camera_ == nullptr || this->detector_ == nullptr || this->visualization_ == nullptr ||
      this->search_pending_ || this->visualization_->capture_pending()) {
    return false;
  }

  this->last_observation_ = TargetObservation();
  this->request_started_ms_ = millis();
  this->frame_received_ms_ = 0;
  this->acquisition_ms_ = 0;
  this->detection_ms_ = 0;
  this->visualization_ms_ = 0;
  this->total_cycle_ms_ = 0;
  this->search_pending_ = true;
  this->camera_->request_image(camera::WEB_REQUESTER);
  ESP_LOGD(TAG, "Recherche cible demandee a %u ms", static_cast<unsigned>(this->request_started_ms_));
  return true;
}

void TargetSearchDiagnostic::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  if (!this->search_pending_ || image == nullptr) {
    return;
  }

  this->frame_received_ms_ = millis();
  this->acquisition_ms_ = this->frame_received_ms_ - this->request_started_ms_;

  auto esp_image = std::static_pointer_cast<esp32_camera::ESP32CameraImage>(image);
  camera_fb_t *frame = esp_image->get_raw_buffer();
  if (frame == nullptr || frame->buf == nullptr || frame->format != PIXFORMAT_GRAYSCALE) {
    ESP_LOGE(TAG, "Framebuffer grayscale invalide pour recherche cible");
    this->search_pending_ = false;
    return;
  }

  const size_t required_size = static_cast<size_t>(frame->width) * static_cast<size_t>(frame->height);
  if (frame->len < required_size) {
    ESP_LOGE(TAG, "Framebuffer trop court pour recherche cible: %u < %u",
             static_cast<unsigned>(frame->len), static_cast<unsigned>(required_size));
    this->search_pending_ = false;
    return;
  }

  GrayFrameView view;
  view.data = frame->buf;
  view.width = frame->width;
  view.height = frame->height;
  view.stride = frame->width;

  const uint32_t detection_started_ms = millis();
  this->last_observation_ = this->detector_->detect(view);
  this->detection_ms_ = millis() - detection_started_ms;

  const uint32_t visualization_started_ms = millis();
  const bool visualization_ok = this->visualization_->update_visualization(
      frame->buf, frame->len, frame->width, frame->height, true);

  if (visualization_ok && this->last_observation_.valid) {
    const float left_f = this->last_observation_.center_x_px - this->last_observation_.width_px * 0.5f;
    const float top_f = this->last_observation_.center_y_px - this->last_observation_.height_px * 0.5f;
    const uint16_t left = static_cast<uint16_t>(std::max(0.0f, left_f));
    const uint16_t top = static_cast<uint16_t>(std::max(0.0f, top_f));
    const uint16_t box_width = static_cast<uint16_t>(std::max(1.0f, this->last_observation_.width_px));
    const uint16_t box_height = static_cast<uint16_t>(std::max(1.0f, this->last_observation_.height_px));
    this->visualization_->annotate_box_green(left, top, box_width, box_height, BOX_THICKNESS);
  }

  this->visualization_ms_ = millis() - visualization_started_ms;
  this->total_cycle_ms_ = millis() - this->request_started_ms_;
  this->search_count_++;
  this->ready_ = visualization_ok;
  this->search_pending_ = false;

  ESP_LOGI(TAG,
           "Recherche cible terminee: trouvee=%s acquisition=%u ms detection=%u ms visualisation=%u ms total=%u ms",
           this->last_observation_.valid ? "OUI" : "NON",
           static_cast<unsigned>(this->acquisition_ms_),
           static_cast<unsigned>(this->detection_ms_),
           static_cast<unsigned>(this->visualization_ms_),
           static_cast<unsigned>(this->total_cycle_ms_));
}

bool TargetSearchDiagnostic::ready() const { return this->ready_; }
bool TargetSearchDiagnostic::search_pending() const { return this->search_pending_; }
bool TargetSearchDiagnostic::target_found() const { return this->last_observation_.valid; }
uint32_t TargetSearchDiagnostic::search_count() const { return this->search_count_; }
uint32_t TargetSearchDiagnostic::request_started_ms() const { return this->request_started_ms_; }
uint32_t TargetSearchDiagnostic::frame_received_ms() const { return this->frame_received_ms_; }
uint32_t TargetSearchDiagnostic::acquisition_ms() const { return this->acquisition_ms_; }
uint32_t TargetSearchDiagnostic::detection_ms() const { return this->detection_ms_; }
uint32_t TargetSearchDiagnostic::visualization_ms() const { return this->visualization_ms_; }
uint32_t TargetSearchDiagnostic::total_cycle_ms() const { return this->total_cycle_ms_; }
const TargetObservation &TargetSearchDiagnostic::last_observation() const { return this->last_observation_; }

}  // namespace geometrie_camera_app
}  // namespace esphome
