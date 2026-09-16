#pragma once

#include <cstddef>
#include <cstdint>

#include "jpeg_artifact_corrector.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;

class JpegFilteredDiagnostic {
 public:
  explicit JpegFilteredDiagnostic(JpegDiagnostic *source);
  ~JpegFilteredDiagnostic();

  bool process();

  bool ready() const;
  uint32_t process_count() const;
  uint32_t source_capture_count() const;
  uint16_t width() const;
  uint16_t height() const;
  const uint8_t *bmp_data() const;
  size_t bmp_size() const;
  int decode_result() const;
  uint32_t decode_ms() const;
  uint32_t correction_ms() const;
  uint32_t total_ms() const;
  const JpegArtifactCorrectionStats &correction_stats() const;

 private:
  bool ensure_buffers_(uint16_t width, uint16_t height);
  void clear_buffers_();
  void build_bmp_header_(uint16_t width, uint16_t height, size_t row_stride);

  JpegDiagnostic *source_;
  JpegArtifactCorrector corrector_;
  uint8_t *bmp_buffer_;
  size_t bmp_size_;
  size_t bmp_capacity_;
  uint8_t *green_mask_;
  size_t green_mask_capacity_;
  uint16_t width_;
  uint16_t height_;
  uint32_t process_count_;
  uint32_t source_capture_count_;
  bool ready_;
  int decode_result_;
  uint32_t decode_ms_;
  uint32_t correction_ms_;
  uint32_t total_ms_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
