#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

class JpegFilteredDiagnostic;
struct TargetObservation;

class TargetDetectionPreview {
 public:
  TargetDetectionPreview();
  ~TargetDetectionPreview();

  bool render(const JpegFilteredDiagnostic *source, const TargetObservation &observation);

  const uint8_t *bmp_data() const;
  size_t bmp_size() const;
  uint16_t width() const;
  uint16_t height() const;

 private:
  bool ensure_buffer_(size_t required_size);
  void clear_buffer_();
  void build_bmp_header_(uint16_t width, uint16_t height, size_t row_stride);
  void draw_rectangle_(uint8_t *pixels, size_t row_stride, uint16_t width, uint16_t height,
                       int x0, int y0, int x1, int y1) const;

  uint8_t *bmp_buffer_;
  size_t bmp_size_;
  size_t bmp_capacity_;
  uint16_t width_;
  uint16_t height_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
