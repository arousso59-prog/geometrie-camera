#pragma once

#include <cstdint>

#include "target_candidate_finder.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

// Raffinement de precision applique apres la detection/validation classique.
// Le detecteur reste responsable de trouver la cible. Cette classe ne cherche
// que la position subpixel des quatre bords externes afin d'ameliorer les
// calculs de taille, distance et pose.
struct TargetSubpixelMetrics {
  bool valid;
  float mean_rms_px;
  float max_rms_px;
  float mean_gradient;
  uint8_t min_edge_samples;
  float width_px;
  float height_px;
  float width_sigma_px;
  float height_sigma_px;
  float width_gradient;
  float height_gradient;

  float v5_width_px;
  float v5_height_px;
  float v5_width_sigma_px;
  float v5_height_sigma_px;
  float v6_width_px;
  float v6_height_px;
  float v6_width_sigma_px;
  float v6_height_sigma_px;

  float top_rms_px;
  float right_rms_px;
  float bottom_rms_px;
  float left_rms_px;
  float top_gradient;
  float right_gradient;
  float bottom_gradient;
  float left_gradient;

  TargetPoint top_line_point;
  TargetPoint right_line_point;
  TargetPoint bottom_line_point;
  TargetPoint left_line_point;
  float top_line_dx;
  float top_line_dy;
  float right_line_dx;
  float right_line_dy;
  float bottom_line_dx;
  float bottom_line_dy;
  float left_line_dx;
  float left_line_dy;
};

class TargetSubpixelRefiner {
 public:
  TargetSubpixelRefiner();

  bool refine(const GrayFrameView &frame,
              const TargetCandidate &input,
              TargetCandidate &output,
              TargetSubpixelMetrics *metrics = nullptr) const;

 private:
  static constexpr uint8_t EDGE_SAMPLE_CAPACITY = 31;

  struct EdgeLine {
    TargetPoint point;
    float dx;
    float dy;
    float rms;
    float position_sigma;
    float mean_gradient;
    uint8_t samples;
    uint8_t local_sample_count;
    float local_fraction[EDGE_SAMPLE_CAPACITY];
    float local_x[EDGE_SAMPLE_CAPACITY];
    float local_y[EDGE_SAMPLE_CAPACITY];
  };

  bool refine_edge_(const GrayFrameView &frame,
                    const TargetPoint &start,
                    const TargetPoint &end,
                    const TargetPoint &target_center,
                    EdgeLine &line) const;

  bool find_edge_offset_(const GrayFrameView &frame,
                         float anchor_x,
                         float anchor_y,
                         float tangent_x,
                         float tangent_y,
                         float normal_x,
                         float normal_y,
                         float expected_gradient_sign,
                         float &offset,
                         float &gradient) const;

  bool fit_edge_line_(const float *s,
                      const float *offsets,
                      const float *weights,
                      uint8_t count,
                      const TargetPoint &start,
                      float tangent_x,
                      float tangent_y,
                      float normal_x,
                      float normal_y,
                      EdgeLine &line) const;

  bool intersect_(const EdgeLine &a,
                  const EdgeLine &b,
                  TargetPoint &point) const;
  float opposite_edge_separation_(const EdgeLine &a,
                                  const EdgeLine &b) const;
  float robust_local_separation_(const EdgeLine &a,
                                 const EdgeLine &b,
                                 float &sigma_px) const;

  bool bilinear_sample_(const GrayFrameView &frame,
                        float x,
                        float y,
                        float &value) const;

  bool geometry_valid_(const TargetCandidate &input,
                       const TargetCandidate &candidate) const;
  void update_geometry_(TargetCandidate &candidate) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
