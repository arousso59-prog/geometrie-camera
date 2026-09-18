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
};

class TargetSubpixelRefiner {
 public:
  TargetSubpixelRefiner();

  bool refine(const GrayFrameView &frame,
              const TargetCandidate &input,
              TargetCandidate &output,
              TargetSubpixelMetrics *metrics = nullptr) const;

 private:
  struct EdgeLine {
    TargetPoint point;
    float dx;
    float dy;
    float rms;
    float mean_gradient;
    uint8_t samples;
  };

  bool refine_edge_(const GrayFrameView &frame,
                    const TargetPoint &start,
                    const TargetPoint &end,
                    EdgeLine &line) const;

  bool find_edge_offset_(const GrayFrameView &frame,
                         float anchor_x,
                         float anchor_y,
                         float tangent_x,
                         float tangent_y,
                         float normal_x,
                         float normal_y,
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
