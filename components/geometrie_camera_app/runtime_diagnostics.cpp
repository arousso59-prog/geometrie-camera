#include "runtime_diagnostics.h"

#include "esp_heap_caps.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace geometrie_camera_app {

RuntimeDiagnostics::RuntimeDiagnostics()
    : last_loop_timestamp_us_(0),
      last_loop_gap_us_(0),
      max_loop_gap_us_(0),
      loop_gap_sum_us_(0),
      loop_sample_count_(0),
      initialized_(false) {}

void RuntimeDiagnostics::record_loop() {
  const uint32_t now_us = micros();

  if (!this->initialized_) {
    this->last_loop_timestamp_us_ = now_us;
    this->initialized_ = true;
    return;
  }

  const uint32_t gap_us = now_us - this->last_loop_timestamp_us_;
  this->last_loop_timestamp_us_ = now_us;
  this->last_loop_gap_us_ = gap_us;
  this->loop_gap_sum_us_ += gap_us;
  this->loop_sample_count_++;

  if (gap_us > this->max_loop_gap_us_) {
    this->max_loop_gap_us_ = gap_us;
  }
}

uint64_t RuntimeDiagnostics::loop_sample_count() const { return this->loop_sample_count_; }
uint32_t RuntimeDiagnostics::last_loop_gap_us() const { return this->last_loop_gap_us_; }
uint32_t RuntimeDiagnostics::max_loop_gap_us() const { return this->max_loop_gap_us_; }

uint32_t RuntimeDiagnostics::average_loop_gap_us() const {
  if (this->loop_sample_count_ == 0) {
    return 0;
  }

  return static_cast<uint32_t>(this->loop_gap_sum_us_ / this->loop_sample_count_);
}

size_t RuntimeDiagnostics::internal_free_bytes() const {
  return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

size_t RuntimeDiagnostics::internal_largest_block_bytes() const {
  return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

size_t RuntimeDiagnostics::psram_free_bytes() const {
  return heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

size_t RuntimeDiagnostics::psram_largest_block_bytes() const {
  return heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
