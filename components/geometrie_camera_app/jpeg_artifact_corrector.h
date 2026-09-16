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

  bool is_green_seed(uint8_t red, uint8_t green, uint8_t blue) const;
  bool correct(uint8_t *grayscale, size_t row_stride, const uint8_t *green_mask,
               uint16_t width, uint16_t height, uint32_t green_seed_count);

  const JpegArtifactCorrectionStats &stats() const;

 private:
  void reset_stats_();

  JpegArtifactCorrectionStats stats_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
