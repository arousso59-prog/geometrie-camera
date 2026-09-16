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

  bool evaluate();

  bool ready() const;
  uint32_t score_x100() const;
  uint32_t evaluation_ms() const;
  uint16_t preview_width() const;
  uint16_t preview_height() const;

 private:
  bool ensure_buffers_(uint16_t width, uint16_t height);
  void clear_buffers_();
  uint32_t compute_score_x100_() const;

  JpegDiagnostic *source_;
  uint8_t *grayscale_buffer_;
  size_t grayscale_capacity_;
  uint8_t *jpeg_work_buffer_;
  uint16_t width_;
  uint16_t height_;
  uint32_t score_x100_;
  uint32_t evaluation_ms_;
  bool ready_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
