#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

class RuntimeDiagnostics {
 public:
  RuntimeDiagnostics();

  void record_loop();

  uint64_t loop_sample_count() const;
  uint32_t last_loop_gap_us() const;
  uint32_t average_loop_gap_us() const;
  uint32_t max_loop_gap_us() const;

  size_t internal_free_bytes() const;
  size_t internal_largest_block_bytes() const;
  size_t psram_free_bytes() const;
  size_t psram_largest_block_bytes() const;

 private:
  uint32_t last_loop_timestamp_us_;
  uint32_t last_loop_gap_us_;
  uint32_t max_loop_gap_us_;
  uint64_t loop_gap_sum_us_;
  uint64_t loop_sample_count_;
  bool initialized_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
