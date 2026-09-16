#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct JpegArtifactCorrectionStats {
  uint32_t green_seed_pixels;
  uint32_t thin_green_pixels;
  uint32_t corrected_green_pixels;
  uint32_t corrected_dark_pixels;
  uint32_t corrected_total_pixels;
  uint16_t affected_rows;
};

class JpegArtifactCorrector {
 public:
  JpegArtifactCorrector();
  ~JpegArtifactCorrector();

  bool correct(uint8_t *grayscale, size_t row_stride, const uint8_t *green_mask,
               uint16_t width, uint16_t height, uint32_t green_seed_count);

  const JpegArtifactCorrectionStats &stats() const;

 private:
  bool ensure_workspace_(size_t mask_size);
  bool build_fast_masks_(const uint8_t *green_mask, uint16_t width, uint16_t height);
  void clear_workspace_();
  void reset_stats_();

  uint8_t *near_green_mask_;
  uint8_t *thin_green_mask_;
  size_t mask_capacity_;
  JpegArtifactCorrectionStats stats_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
