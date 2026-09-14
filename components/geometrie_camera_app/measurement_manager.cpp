#include "measurement_manager.h"

namespace esphome {
namespace geometrie_camera_app {

MeasurementManager::MeasurementManager()
    : target_detector_(), measurement_engine_(), last_measurement_(), valid_measurement_count_(0) {}

void MeasurementManager::setup() {
  this->reset();
}

void MeasurementManager::reset() {
  this->last_measurement_ = GeometryMeasurement();
  this->valid_measurement_count_ = 0;
}

bool MeasurementManager::process(const GrayFrameView &frame, uint32_t timestamp_ms) {
  const TargetObservation observation = this->target_detector_.detect(frame);
  this->last_measurement_ = this->measurement_engine_.compute(observation, timestamp_ms);

  if (this->last_measurement_.valid) {
    this->valid_measurement_count_++;
    return true;
  }

  return false;
}

uint32_t MeasurementManager::valid_measurement_count() const {
  return this->valid_measurement_count_;
}

const GeometryMeasurement &MeasurementManager::last_measurement() const {
  return this->last_measurement_;
}

TargetDetector &MeasurementManager::target_detector() {
  return this->target_detector_;
}

GeometryMeasurementEngine &MeasurementManager::measurement_engine() {
  return this->measurement_engine_;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
