#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;

class ImageSharpnessEvaluator {
 public:
  explicit ImageSharpnessEvaluator(JpegDiagnostic *source);
  ~ImageSharpnessEvaluator();

  bool evaluate_region(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

  bool ready() const;
  uint32_t score_x100() const;
  uint32_t evaluation_ms() const;
  uint32_t mean_luma_x100() const;
  uint32_t dark_percent_x100() const;
  uint32_t bright_percent_x100() const;
  uint8_t p10_luma() const;
  uint8_t p90_luma() const;
  uint8_t contrast_luma() const;
  uint16_t preview_width() const;
  uint16_t preview_height() const;

 private:
  bool ensure_buffers_(uint16_t width, uint16_t height);
  void clear_buffers_();
  uint32_t compute_score_x100_(uint16_t x, uint16_t y, uint16_t width, uint16_t height) const;

  JpegDiagnostic *source_;
  uint8_t *grayscale_buffer_;
  size_t grayscale_capacity_;
  uint8_t *jpeg_work_buffer_;
  uint16_t width_;
  uint16_t height_;
  uint32_t score_x100_;
  uint32_t evaluation_ms_;
  uint32_t mean_luma_x100_;
  uint32_t dark_percent_x100_;
  uint32_t bright_percent_x100_;
  uint8_t p10_luma_;
  uint8_t p90_luma_;
  uint8_t contrast_luma_;
  bool ready_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
