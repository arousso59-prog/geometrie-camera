#include "target_detection_service.h"

#include "esphome/core/hal.h"
#include "jpeg_filtered_diagnostic.h"
#include "target_detector.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetectionService::TargetDetectionService(JpegFilteredDiagnostic *source, TargetDetector *detector)
    : source_(source),
      detector_(detector),
      last_observation_(),
      detection_count_(0),
      source_process_count_(0),
      detection_ms_(0),
      ready_(false) {}

bool TargetDetectionService::detect() {
  this->ready_ = false;
  this->last_observation_ = TargetObservation();
  this->detection_ms_ = 0;

  if (this->source_ == nullptr || this->detector_ == nullptr || !this->source_->ready() ||
      this->source_->grayscale_data() == nullptr || this->source_->width() == 0 || this->source_->height() == 0 ||
      this->source_->grayscale_stride() < this->source_->width()) {
    return false;
  }

  GrayFrameView frame;
  frame.data = this->source_->grayscale_data();
  frame.width = this->source_->width();
  frame.height = this->source_->height();
  frame.stride = this->source_->grayscale_stride();

  const uint32_t started_ms = millis();
  this->last_observation_ = this->detector_->detect(frame);
  this->detection_ms_ = millis() - started_ms;

  this->source_process_count_ = this->source_->process_count();
  this->detection_count_++;
  this->ready_ = true;
  return true;
}

void TargetDetectionService::reset_tracking() {
  if (this->detector_ != nullptr) {
    this->detector_->reset_tracking();
  }
}

bool TargetDetectionService::ready() const { return this->ready_; }
bool TargetDetectionService::target_found() const { return this->ready_ && this->last_observation_.valid; }
uint32_t TargetDetectionService::detection_count() const { return this->detection_count_; }
uint32_t TargetDetectionService::source_process_count() const { return this->source_process_count_; }
uint32_t TargetDetectionService::detection_ms() const { return this->detection_ms_; }
const TargetObservation &TargetDetectionService::last_observation() const { return this->last_observation_; }

}  // namespace geometrie_camera_app
}  // namespace esphome
