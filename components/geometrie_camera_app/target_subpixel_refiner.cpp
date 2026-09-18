#include "target_subpixel_refiner.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_subpixel_refiner";

constexpr uint8_t EDGE_SAMPLE_COUNT = 31;
constexpr uint8_t MIN_EDGE_SAMPLES = 15;
constexpr float EDGE_MARGIN_RATIO = 0.18f;
constexpr int TANGENT_AVERAGE_RADIUS_PX = 3;
constexpr float NORMAL_SEARCH_RADIUS_PX = 3.0f;
constexpr float NORMAL_PROFILE_STEP_PX = 0.25f;
constexpr uint8_t NORMAL_PROFILE_COUNT =
    static_cast<uint8_t>((2.0f * NORMAL_SEARCH_RADIUS_PX) /
                         NORMAL_PROFILE_STEP_PX) + 1U;
constexpr uint8_t NORMAL_CENTROID_RADIUS = 4;
constexpr float GRADIENT_HALF_SPAN_PX = 0.75f;
constexpr float MIN_EDGE_GRADIENT = 8.0f;
constexpr float INITIAL_OFFSET_GATE_PX = 2.0f;
constexpr float RESIDUAL_GATE_PX = 0.85f;
constexpr float MIN_ADAPTIVE_GATE_PX = 0.30f;
constexpr float MAX_LINE_RMS_PX = 0.60f;
constexpr float MAX_SLOPE_CORRECTION = 0.08f;
constexpr float MAX_CORNER_SHIFT_PX = 4.0f;
constexpr float MIN_EDGE_LENGTH_PX = 8.0f;
constexpr float MIN_EDGE_RATIO = 0.30f;
constexpr float MIN_AREA_RATIO = 0.20f;
constexpr float EPSILON = 1.0e-6f;

float point_distance(const TargetPoint &a, const TargetPoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

float signed_area_twice(const TargetCandidate &candidate) {
  return
      candidate.top_left.x * candidate.top_right.y -
      candidate.top_left.y * candidate.top_right.x +
      candidate.top_right.x * candidate.bottom_right.y -
      candidate.top_right.y * candidate.bottom_right.x +
      candidate.bottom_right.x * candidate.bottom_left.y -
      candidate.bottom_right.y * candidate.bottom_left.x +
      candidate.bottom_left.x * candidate.top_left.y -
      candidate.bottom_left.y * candidate.top_left.x;
}

float median_copy(const float *values, uint8_t count) {
  float sorted[EDGE_SAMPLE_COUNT];
  const uint8_t bounded_count = std::min<uint8_t>(count, EDGE_SAMPLE_COUNT);
  for (uint8_t i = 0; i < bounded_count; ++i) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < bounded_count; ++i) {
    const float value = sorted[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }

  if (bounded_count == 0) {
    return 0.0f;
  }
  if ((bounded_count & 1U) != 0U) {
    return sorted[bounded_count / 2U];
  }
  return 0.5f * (sorted[bounded_count / 2U - 1U] +
                 sorted[bounded_count / 2U]);
}

bool finite_point(const TargetPoint &point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}
}  // namespace

TargetSubpixelRefiner::TargetSubpixelRefiner() {}

bool TargetSubpixelRefiner::refine(const GrayFrameView &frame,
                                   const TargetCandidate &input,
                                   TargetCandidate &output,
                                   TargetSubpixelMetrics *metrics) const {
  output = input;
  if (metrics != nullptr) {
    metrics->valid = false;
    metrics->mean_rms_px = 0.0f;
    metrics->max_rms_px = 0.0f;
    metrics->mean_gradient = 0.0f;
    metrics->min_edge_samples = 0;
    metrics->width_px = 0.0f;
    metrics->height_px = 0.0f;
    metrics->width_sigma_px = 0.0f;
    metrics->height_sigma_px = 0.0f;
    metrics->width_gradient = 0.0f;
    metrics->height_gradient = 0.0f;
  }

  if (frame.data == nullptr || frame.width < 4 || frame.height < 4 ||
      frame.stride < frame.width ||
      input.width < MIN_EDGE_LENGTH_PX ||
      input.height < MIN_EDGE_LENGTH_PX) {
    return false;
  }

  EdgeLine top;
  EdgeLine right;
  EdgeLine bottom;
  EdgeLine left;

  TargetPoint target_center;
  target_center.x = input.center_x;
  target_center.y = input.center_y;

  if (!this->refine_edge_(frame, input.top_left, input.top_right, target_center, top) ||
      !this->refine_edge_(frame, input.top_right, input.bottom_right, target_center, right) ||
      !this->refine_edge_(frame, input.bottom_left, input.bottom_right, target_center, bottom) ||
      !this->refine_edge_(frame, input.top_left, input.bottom_left, target_center, left)) {
    return false;
  }

  float direct_width_sigma_px = 0.0f;
  float direct_height_sigma_px = 0.0f;
  float direct_width_px =
      this->robust_local_separation_(left, right, direct_width_sigma_px);
  float direct_height_px =
      this->robust_local_separation_(top, bottom, direct_height_sigma_px);

  if (!std::isfinite(direct_width_px) || direct_width_px <= 0.0f) {
    direct_width_px = this->opposite_edge_separation_(left, right);
    direct_width_sigma_px =
        std::sqrt(left.position_sigma * left.position_sigma +
                  right.position_sigma * right.position_sigma);
  }
  if (!std::isfinite(direct_height_px) || direct_height_px <= 0.0f) {
    direct_height_px = this->opposite_edge_separation_(top, bottom);
    direct_height_sigma_px =
        std::sqrt(top.position_sigma * top.position_sigma +
                  bottom.position_sigma * bottom.position_sigma);
  }
  if (!std::isfinite(direct_width_px) || !std::isfinite(direct_height_px) ||
      direct_width_px < MIN_EDGE_LENGTH_PX ||
      direct_height_px < MIN_EDGE_LENGTH_PX) {
    return false;
  }

  TargetCandidate refined = input;
  if (!this->intersect_(top, left, refined.top_left) ||
      !this->intersect_(top, right, refined.top_right) ||
      !this->intersect_(bottom, right, refined.bottom_right) ||
      !this->intersect_(bottom, left, refined.bottom_left)) {
    return false;
  }

  this->update_geometry_(refined);
  if (!this->geometry_valid_(input, refined)) {
    return false;
  }

  output = refined;

  if (metrics != nullptr) {
    metrics->valid = true;
    metrics->mean_rms_px =
        0.25f * (top.rms + right.rms + bottom.rms + left.rms);
    metrics->max_rms_px =
        std::max(std::max(top.rms, right.rms),
                 std::max(bottom.rms, left.rms));
    metrics->mean_gradient =
        0.25f * (top.mean_gradient + right.mean_gradient +
                 bottom.mean_gradient + left.mean_gradient);
    metrics->min_edge_samples =
        std::min(std::min(top.samples, right.samples),
                 std::min(bottom.samples, left.samples));
    metrics->width_px = direct_width_px;
    metrics->height_px = direct_height_px;
    metrics->width_sigma_px = direct_width_sigma_px;
    metrics->height_sigma_px = direct_height_sigma_px;
    metrics->width_gradient =
        0.5f * (left.mean_gradient + right.mean_gradient);
    metrics->height_gradient =
        0.5f * (top.mean_gradient + bottom.mean_gradient);
  }

  ESP_LOGD(
      TAG,
      "Subpixel V6 OK center=(%.3f,%.3f) corners=%.3fx%.3f edges=%.3fx%.3f "
      "rms=(%.3f,%.3f,%.3f,%.3f) grad=(%.1f,%.1f,%.1f,%.1f)",
      output.center_x, output.center_y, output.width, output.height,
      direct_width_px, direct_height_px,
      top.rms, right.rms, bottom.rms, left.rms,
      top.mean_gradient, right.mean_gradient,
      bottom.mean_gradient, left.mean_gradient);
  return true;
}

bool TargetSubpixelRefiner::refine_edge_(const GrayFrameView &frame,
                                         const TargetPoint &start,
                                         const TargetPoint &end,
                                         const TargetPoint &target_center,
                                         EdgeLine &line) const {
  const float vx = end.x - start.x;
  const float vy = end.y - start.y;
  const float length = std::sqrt(vx * vx + vy * vy);
  if (!std::isfinite(length) || length < MIN_EDGE_LENGTH_PX) {
    return false;
  }

  const float tangent_x = vx / length;
  const float tangent_y = vy / length;
  const float normal_x = -tangent_y;
  const float normal_y = tangent_x;

  // Le cadre exterieur du marqueur est noir. Determiner de quel cote de cette
  // normale se trouve l'interieur permet de chercher uniquement la transition
  // fond clair -> bord noir dans le bon sens et d'ignorer les transitions
  // internes du motif qui ont la polarite opposee.
  const float mid_x = 0.5f * (start.x + end.x);
  const float mid_y = 0.5f * (start.y + end.y);
  const float inward_projection =
      (target_center.x - mid_x) * normal_x +
      (target_center.y - mid_y) * normal_y;
  if (!std::isfinite(inward_projection) ||
      std::fabs(inward_projection) < EPSILON) {
    return false;
  }
  const float expected_gradient_sign =
      inward_projection > 0.0f ? -1.0f : 1.0f;

  float s_values[EDGE_SAMPLE_COUNT];
  float offsets[EDGE_SAMPLE_COUNT];
  float weights[EDGE_SAMPLE_COUNT];
  uint8_t count = 0;

  for (uint8_t i = 0; i < EDGE_SAMPLE_COUNT; ++i) {
    const float fraction =
        EDGE_MARGIN_RATIO +
        (1.0f - 2.0f * EDGE_MARGIN_RATIO) *
            (static_cast<float>(i) / static_cast<float>(EDGE_SAMPLE_COUNT - 1U));
    const float s = fraction * length;
    const float anchor_x = start.x + tangent_x * s;
    const float anchor_y = start.y + tangent_y * s;

    float offset = 0.0f;
    float gradient = 0.0f;
    if (!this->find_edge_offset_(frame, anchor_x, anchor_y,
                                 tangent_x, tangent_y,
                                 normal_x, normal_y,
                                 expected_gradient_sign,
                                 offset, gradient)) {
      continue;
    }

    s_values[count] = s;
    offsets[count] = offset;
    weights[count] = gradient;
    ++count;
  }

  if (count < MIN_EDGE_SAMPLES) {
    return false;
  }

  // Premier rejet tres simple mais robuste : l'edge externe doit rester
  // proche de la droite fournie par le raffinement pixel. Cela evite qu'une
  // ligne du motif 7x7, un reflet ou une poussiere prenne le dessus.
  const float median_offset = median_copy(offsets, count);
  float gated_s[EDGE_SAMPLE_COUNT];
  float gated_offsets[EDGE_SAMPLE_COUNT];
  float gated_weights[EDGE_SAMPLE_COUNT];
  uint8_t gated_count = 0;

  for (uint8_t i = 0; i < count; ++i) {
    if (std::fabs(offsets[i] - median_offset) <= INITIAL_OFFSET_GATE_PX) {
      gated_s[gated_count] = s_values[i];
      gated_offsets[gated_count] = offsets[i];
      gated_weights[gated_count] = weights[i];
      ++gated_count;
    }
  }

  if (gated_count < MIN_EDGE_SAMPLES) {
    return false;
  }

  EdgeLine initial_line;
  if (!this->fit_edge_line_(gated_s, gated_offsets, gated_weights, gated_count,
                            start, tangent_x, tangent_y, normal_x, normal_y,
                            initial_line)) {
    return false;
  }

  // Deuxieme passe : retirer les points qui ne suivent pas la droite estimee.
  // On recalcule d(s) a partir de la ligne initiale dans le repere local.
  const float initial_dir_cross_t =
      tangent_x * initial_line.dy - tangent_y * initial_line.dx;
  const float initial_dir_dot_t =
      tangent_x * initial_line.dx + tangent_y * initial_line.dy;
  if (std::fabs(initial_dir_dot_t) < EPSILON) {
    return false;
  }
  const float initial_slope = initial_dir_cross_t / initial_dir_dot_t;

  const float point_dx = initial_line.point.x - start.x;
  const float point_dy = initial_line.point.y - start.y;
  const float initial_point_s =
      point_dx * tangent_x + point_dy * tangent_y;
  const float initial_point_d =
      point_dx * normal_x + point_dy * normal_y;
  const float initial_intercept =
      initial_point_d - initial_slope * initial_point_s;

  float residual_abs[EDGE_SAMPLE_COUNT];
  for (uint8_t i = 0; i < gated_count; ++i) {
    const float predicted = initial_slope * gated_s[i] + initial_intercept;
    residual_abs[i] = std::fabs(gated_offsets[i] - predicted);
  }
  const float median_residual = median_copy(residual_abs, gated_count);
  const float adaptive_gate = std::max(
      MIN_ADAPTIVE_GATE_PX,
      std::min(RESIDUAL_GATE_PX, 3.5f * median_residual + 0.05f));

  float final_s[EDGE_SAMPLE_COUNT];
  float final_offsets[EDGE_SAMPLE_COUNT];
  float final_weights[EDGE_SAMPLE_COUNT];
  uint8_t final_count = 0;
  for (uint8_t i = 0; i < gated_count; ++i) {
    const float residual = residual_abs[i];
    if (residual <= adaptive_gate) {
      final_s[final_count] = gated_s[i];
      final_offsets[final_count] = gated_offsets[i];

      // Poids robuste de type Cauchy : un point proche de la droite et avec un
      // gradient fort contribue davantage, sans qu'un seul pixel tres contraste
      // puisse dominer tout l'ajustement.
      const float normalized = residual / std::max(0.05f, adaptive_gate);
      const float robust_weight = 1.0f / (1.0f + normalized * normalized);
      final_weights[final_count] =
          std::max(1.0f, gated_weights[i] * robust_weight);
      ++final_count;
    }
  }

  if (final_count < MIN_EDGE_SAMPLES) {
    return false;
  }

  if (!this->fit_edge_line_(final_s, final_offsets, final_weights, final_count,
                            start, tangent_x, tangent_y, normal_x, normal_y,
                            line)) {
    return false;
  }

  line.local_sample_count = 0;
  for (uint8_t i = 0;
       i < final_count && line.local_sample_count < EDGE_SAMPLE_CAPACITY;
       ++i) {
    const uint8_t out = line.local_sample_count++;
    line.local_fraction[out] = final_s[i] / length;
    line.local_x[out] =
        start.x + tangent_x * final_s[i] + normal_x * final_offsets[i];
    line.local_y[out] =
        start.y + tangent_y * final_s[i] + normal_y * final_offsets[i];
  }

  return line.rms <= MAX_LINE_RMS_PX &&
         std::isfinite(line.mean_gradient) &&
         line.mean_gradient >= MIN_EDGE_GRADIENT;
}

bool TargetSubpixelRefiner::find_edge_offset_(const GrayFrameView &frame,
                                              float anchor_x,
                                              float anchor_y,
                                              float tangent_x,
                                              float tangent_y,
                                              float normal_x,
                                              float normal_y,
                                              float expected_gradient_sign,
                                              float &offset,
                                              float &gradient) const {
  float strengths[NORMAL_PROFILE_COUNT];

  int best_index = -1;
  float best_strength = -1.0f;

  // V5 : echantillonner le profil a 0,25 px. L'interpolation bilineaire donne
  // une vraie courbe de gradient subpixel, beaucoup moins sensible au passage
  // d'un maximum d'un pixel entier au suivant.
  for (uint8_t index = 0; index < NORMAL_PROFILE_COUNT; ++index) {
    const float delta =
        -NORMAL_SEARCH_RADIUS_PX +
        static_cast<float>(index) * NORMAL_PROFILE_STEP_PX;
    const float before_offset = delta - GRADIENT_HALF_SPAN_PX;
    const float after_offset = delta + GRADIENT_HALF_SPAN_PX;

    float before_sum = 0.0f;
    float after_sum = 0.0f;
    uint8_t valid_pairs = 0;

    for (int tangent_offset = -TANGENT_AVERAGE_RADIUS_PX;
         tangent_offset <= TANGENT_AVERAGE_RADIUS_PX; ++tangent_offset) {
      const float along = static_cast<float>(tangent_offset);
      const float base_x = anchor_x + tangent_x * along;
      const float base_y = anchor_y + tangent_y * along;

      float before = 0.0f;
      float after = 0.0f;
      if (!this->bilinear_sample_(
              frame,
              base_x + normal_x * before_offset,
              base_y + normal_y * before_offset,
              before) ||
          !this->bilinear_sample_(
              frame,
              base_x + normal_x * after_offset,
              base_y + normal_y * after_offset,
              after)) {
        continue;
      }

      before_sum += before;
      after_sum += after;
      ++valid_pairs;
    }

    if (valid_pairs < 5) {
      strengths[index] = 0.0f;
      continue;
    }

    const float signed_gradient =
        after_sum / valid_pairs - before_sum / valid_pairs;
    const float oriented_strength =
        signed_gradient * expected_gradient_sign;
    const float strength = std::max(0.0f, oriented_strength);
    strengths[index] = strength;
    if (strength > best_strength) {
      best_strength = strength;
      best_index = index;
    }
  }

  if (best_index < static_cast<int>(NORMAL_CENTROID_RADIUS) ||
      best_index >=
          static_cast<int>(NORMAL_PROFILE_COUNT - NORMAL_CENTROID_RADIUS) ||
      best_strength < MIN_EDGE_GRADIENT) {
    return false;
  }

  // Au lieu d'une parabole definie par seulement trois points, calculer le
  // barycentre du lobe de gradient sur +/-1 px. Soustraire le plancher local
  // rend la position beaucoup moins sensible au bruit JPEG.
  float local_floor = strengths[best_index - NORMAL_CENTROID_RADIUS];
  for (int i = best_index - NORMAL_CENTROID_RADIUS;
       i <= best_index + NORMAL_CENTROID_RADIUS; ++i) {
    local_floor = std::min(local_floor, strengths[i]);
  }

  double weighted_position = 0.0;
  double weight_sum = 0.0;
  for (int i = best_index - NORMAL_CENTROID_RADIUS;
       i <= best_index + NORMAL_CENTROID_RADIUS; ++i) {
    const float delta =
        -NORMAL_SEARCH_RADIUS_PX +
        static_cast<float>(i) * NORMAL_PROFILE_STEP_PX;
    const float excess = std::max(0.0f, strengths[i] - local_floor);
    const float weight = excess * excess;
    weighted_position += static_cast<double>(delta) * weight;
    weight_sum += weight;
  }

  if (weight_sum > EPSILON) {
    offset = static_cast<float>(weighted_position / weight_sum);
  } else {
    offset =
        -NORMAL_SEARCH_RADIUS_PX +
        static_cast<float>(best_index) * NORMAL_PROFILE_STEP_PX;
  }
  gradient = best_strength;

  return std::isfinite(offset) && std::isfinite(gradient) &&
         std::fabs(offset) <= NORMAL_SEARCH_RADIUS_PX;
}

bool TargetSubpixelRefiner::fit_edge_line_(const float *s,
                                           const float *offsets,
                                           const float *weights,
                                           uint8_t count,
                                           const TargetPoint &start,
                                           float tangent_x,
                                           float tangent_y,
                                           float normal_x,
                                           float normal_y,
                                           EdgeLine &line) const {
  if (s == nullptr || offsets == nullptr || weights == nullptr ||
      count < MIN_EDGE_SAMPLES) {
    return false;
  }

  double sum_w = 0.0;
  double sum_s = 0.0;
  double sum_d = 0.0;
  double sum_ss = 0.0;
  double sum_sd = 0.0;
  double gradient_sum = 0.0;

  for (uint8_t i = 0; i < count; ++i) {
    const double weight = std::max(1.0f, weights[i]);
    sum_w += weight;
    sum_s += weight * s[i];
    sum_d += weight * offsets[i];
    sum_ss += weight * s[i] * s[i];
    sum_sd += weight * s[i] * offsets[i];
    gradient_sum += weights[i];
  }

  const double denominator = sum_w * sum_ss - sum_s * sum_s;
  if (!std::isfinite(denominator) || std::fabs(denominator) < 1.0e-9) {
    return false;
  }

  const float slope = static_cast<float>(
      (sum_w * sum_sd - sum_s * sum_d) / denominator);
  const float intercept = static_cast<float>(
      (sum_d - static_cast<double>(slope) * sum_s) / sum_w);

  if (!std::isfinite(slope) || !std::isfinite(intercept) ||
      std::fabs(slope) > MAX_SLOPE_CORRECTION) {
    return false;
  }

  double weighted_error = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    const double weight = std::max(1.0f, weights[i]);
    const double residual =
        offsets[i] - (static_cast<double>(slope) * s[i] + intercept);
    weighted_error += weight * residual * residual;
  }

  const float rms = static_cast<float>(std::sqrt(weighted_error / sum_w));
  if (!std::isfinite(rms)) {
    return false;
  }

  float dir_x = tangent_x + normal_x * slope;
  float dir_y = tangent_y + normal_y * slope;
  const float dir_norm = std::sqrt(dir_x * dir_x + dir_y * dir_y);
  if (!std::isfinite(dir_norm) || dir_norm < EPSILON) {
    return false;
  }
  dir_x /= dir_norm;
  dir_y /= dir_norm;

  const float mean_s = static_cast<float>(sum_s / sum_w);
  const float mean_offset = slope * mean_s + intercept;
  line.point.x =
      start.x + tangent_x * mean_s + normal_x * mean_offset;
  line.point.y =
      start.y + tangent_y * mean_s + normal_y * mean_offset;
  line.dx = dir_x;
  line.dy = dir_y;
  line.rms = rms;
  line.position_sigma =
      std::max(0.005f, rms / std::sqrt(static_cast<float>(count)));
  line.mean_gradient = static_cast<float>(gradient_sum / count);
  line.samples = count;
  return finite_point(line.point);
}

bool TargetSubpixelRefiner::intersect_(const EdgeLine &a,
                                       const EdgeLine &b,
                                       TargetPoint &point) const {
  const float denominator = a.dx * b.dy - a.dy * b.dx;
  if (!std::isfinite(denominator) || std::fabs(denominator) < 1.0e-4f) {
    return false;
  }

  const float qx = b.point.x - a.point.x;
  const float qy = b.point.y - a.point.y;
  const float t = (qx * b.dy - qy * b.dx) / denominator;

  point.x = a.point.x + t * a.dx;
  point.y = a.point.y + t * a.dy;
  return finite_point(point);
}

float TargetSubpixelRefiner::opposite_edge_separation_(
    const EdgeLine &a, const EdgeLine &b) const {
  // Les line.point sont places au milieu pondere de chaque bord. La distance
  // symetrique entre les deux droites donne une dimension au centre de la
  // cible et evite l'amplification des petites erreurs par intersection des
  // coins lorsque les droites convergent legerement en perspective.
  const float delta_x = b.point.x - a.point.x;
  const float delta_y = b.point.y - a.point.y;

  const float normal_a_x = -a.dy;
  const float normal_a_y = a.dx;
  const float normal_b_x = -b.dy;
  const float normal_b_y = b.dx;

  const float distance_to_a =
      std::fabs(delta_x * normal_a_x + delta_y * normal_a_y);
  const float distance_to_b =
      std::fabs(delta_x * normal_b_x + delta_y * normal_b_y);
  return 0.5f * (distance_to_a + distance_to_b);
}

float TargetSubpixelRefiner::robust_local_separation_(
    const EdgeLine &a, const EdgeLine &b, float &sigma_px) const {
  sigma_px = 0.0f;
  if (a.local_sample_count < 7 || b.local_sample_count < 7) {
    return NAN;
  }

  float b_dx = b.dx;
  float b_dy = b.dy;
  if (a.dx * b_dx + a.dy * b_dy < 0.0f) {
    b_dx = -b_dx;
    b_dy = -b_dy;
  }

  float tangent_x = a.dx + b_dx;
  float tangent_y = a.dy + b_dy;
  const float tangent_norm =
      std::sqrt(tangent_x * tangent_x + tangent_y * tangent_y);
  if (!std::isfinite(tangent_norm) || tangent_norm < EPSILON) {
    return NAN;
  }
  tangent_x /= tangent_norm;
  tangent_y /= tangent_norm;
  const float normal_x = -tangent_y;
  const float normal_y = tangent_x;

  float separations[EDGE_SAMPLE_CAPACITY];
  uint8_t count = 0;

  for (uint8_t i = 0;
       i < a.local_sample_count && count < EDGE_SAMPLE_CAPACITY;
       ++i) {
    int best_j = -1;
    float best_fraction_delta = 1.0f;
    for (uint8_t j = 0; j < b.local_sample_count; ++j) {
      const float delta =
          std::fabs(a.local_fraction[i] - b.local_fraction[j]);
      if (delta < best_fraction_delta) {
        best_fraction_delta = delta;
        best_j = j;
      }
    }

    if (best_j < 0 || best_fraction_delta > 0.04f) {
      continue;
    }

    const float dx = b.local_x[best_j] - a.local_x[i];
    const float dy = b.local_y[best_j] - a.local_y[i];
    const float separation = std::fabs(dx * normal_x + dy * normal_y);
    if (std::isfinite(separation) && separation >= MIN_EDGE_LENGTH_PX) {
      separations[count++] = separation;
    }
  }

  if (count < 7) {
    return NAN;
  }

  const float median = median_copy(separations, count);
  float deviations[EDGE_SAMPLE_CAPACITY];
  for (uint8_t i = 0; i < count; ++i) {
    deviations[i] = std::fabs(separations[i] - median);
  }
  const float mad = median_copy(deviations, count);
  const float robust_sigma = 1.4826f * mad;
  const float gate = std::max(0.30f, 3.5f * robust_sigma);

  double sum = 0.0;
  double sum_sq = 0.0;
  uint8_t inliers = 0;
  for (uint8_t i = 0; i < count; ++i) {
    if (std::fabs(separations[i] - median) > gate) {
      continue;
    }
    sum += separations[i];
    sum_sq += static_cast<double>(separations[i]) * separations[i];
    ++inliers;
  }

  if (inliers < 5) {
    return NAN;
  }

  const float mean = static_cast<float>(sum / inliers);
  const double variance =
      std::max(0.0, sum_sq / inliers -
                        static_cast<double>(mean) * mean);
  const float standard_error =
      static_cast<float>(std::sqrt(variance / inliers));
  sigma_px = std::max(0.010f, standard_error);
  return mean;
}

bool TargetSubpixelRefiner::bilinear_sample_(const GrayFrameView &frame,
                                             float x,
                                             float y,
                                             float &value) const {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      x < 0.0f || y < 0.0f ||
      x >= static_cast<float>(frame.width - 1U) ||
      y >= static_cast<float>(frame.height - 1U)) {
    return false;
  }

  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float fx = x - static_cast<float>(x0);
  const float fy = y - static_cast<float>(y0);

  const uint8_t *row0 =
      frame.data + static_cast<size_t>(y0) * frame.stride;
  const uint8_t *row1 =
      frame.data + static_cast<size_t>(y1) * frame.stride;

  const float top =
      static_cast<float>(row0[x0]) +
      (static_cast<float>(row0[x1]) - static_cast<float>(row0[x0])) * fx;
  const float bottom =
      static_cast<float>(row1[x0]) +
      (static_cast<float>(row1[x1]) - static_cast<float>(row1[x0])) * fx;
  value = top + (bottom - top) * fy;
  return std::isfinite(value);
}

bool TargetSubpixelRefiner::geometry_valid_(const TargetCandidate &input,
                                            const TargetCandidate &candidate) const {
  if (!finite_point(candidate.top_left) ||
      !finite_point(candidate.top_right) ||
      !finite_point(candidate.bottom_right) ||
      !finite_point(candidate.bottom_left)) {
    return false;
  }

  if (point_distance(input.top_left, candidate.top_left) > MAX_CORNER_SHIFT_PX ||
      point_distance(input.top_right, candidate.top_right) > MAX_CORNER_SHIFT_PX ||
      point_distance(input.bottom_right, candidate.bottom_right) > MAX_CORNER_SHIFT_PX ||
      point_distance(input.bottom_left, candidate.bottom_left) > MAX_CORNER_SHIFT_PX) {
    return false;
  }

  const float top = point_distance(candidate.top_left, candidate.top_right);
  const float right = point_distance(candidate.top_right, candidate.bottom_right);
  const float bottom = point_distance(candidate.bottom_left, candidate.bottom_right);
  const float left = point_distance(candidate.top_left, candidate.bottom_left);

  const float minimum_edge =
      std::min(std::min(top, bottom), std::min(left, right));
  const float maximum_edge =
      std::max(std::max(top, bottom), std::max(left, right));
  if (minimum_edge < MIN_EDGE_LENGTH_PX ||
      maximum_edge <= 0.0f ||
      minimum_edge / maximum_edge < MIN_EDGE_RATIO) {
    return false;
  }

  const float input_area = signed_area_twice(input);
  const float refined_area = signed_area_twice(candidate);
  if (!std::isfinite(input_area) || !std::isfinite(refined_area) ||
      input_area * refined_area <= 0.0f) {
    return false;
  }

  const float reference_area =
      std::max(1.0f, candidate.width * candidate.height);
  return (0.5f * std::fabs(refined_area)) / reference_area >= MIN_AREA_RATIO;
}

void TargetSubpixelRefiner::update_geometry_(TargetCandidate &candidate) const {
  candidate.center_x =
      0.25f * (candidate.top_left.x + candidate.top_right.x +
               candidate.bottom_right.x + candidate.bottom_left.x);
  candidate.center_y =
      0.25f * (candidate.top_left.y + candidate.top_right.y +
               candidate.bottom_right.y + candidate.bottom_left.y);
  candidate.width =
      0.5f * (point_distance(candidate.top_left, candidate.top_right) +
              point_distance(candidate.bottom_left, candidate.bottom_right));
  candidate.height =
      0.5f * (point_distance(candidate.top_left, candidate.bottom_left) +
              point_distance(candidate.top_right, candidate.bottom_right));
}

}  // namespace geometrie_camera_app
}  // namespace esphome
