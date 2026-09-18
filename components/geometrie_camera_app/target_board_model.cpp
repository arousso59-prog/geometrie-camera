#include "target_board_model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace geometrie_camera_app {

namespace {

constexpr uint16_t MAX_COMPOSITE_FEATURES = 384;
constexpr uint16_t MIN_PATTERN_FEATURES = 12;

constexpr TargetMarkerSpec MARKERS[3] = {
    {TargetMarkerId::A, 12.0f, 48.0f, 40.0f},
    {TargetMarkerId::B, 100.0f, 25.0f, 50.0f},
    {TargetMarkerId::C, 198.0f, 12.0f, 40.0f},
};

constexpr uint8_t GRID_A[7][7] = {
    {1,1,1,1,1,1,1},
    {1,0,0,1,0,0,1},
    {1,1,1,1,1,1,1},
    {1,0,0,0,0,1,1},
    {1,0,0,1,1,0,1},
    {1,0,0,0,1,1,1},
    {1,1,1,1,1,1,1},
};

constexpr uint8_t GRID_B[7][7] = {
    {1,1,1,1,1,1,1},
    {1,1,0,1,1,0,1},
    {1,0,1,0,0,1,1},
    {1,1,1,1,0,0,1},
    {1,0,0,1,1,1,1},
    {1,1,0,0,1,0,1},
    {1,1,1,1,1,1,1},
};

constexpr uint8_t GRID_C[7][7] = {
    {1,1,1,1,1,1,1},
    {1,1,1,0,1,0,1},
    {1,1,0,0,1,0,1},
    {1,0,1,0,0,1,1},
    {1,1,0,1,0,1,1},
    {1,0,0,0,0,1,1},
    {1,1,1,1,1,1,1},
};

float point_distance(const ImagePoint &a, const ImagePoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

void canonical_corners(const TargetObservation &observation,
                       ImagePoint (&points)[4]) {
  const ImagePoint observed[4] = {
      observation.top_left_px,
      observation.top_right_px,
      observation.bottom_right_px,
      observation.bottom_left_px,
  };
  int rotation =
      static_cast<int>(std::lround(observation.rotation_deg / 90.0f));
  rotation %= 4;
  if (rotation < 0) rotation += 4;
  static constexpr uint8_t MAP[4][4] = {
      {0,1,2,3},
      {1,2,3,0},
      {2,3,0,1},
      {3,0,1,2},
  };
  for (uint8_t i = 0; i < 4; ++i) {
    points[i] = observed[MAP[rotation][i]];
  }
}

bool solve_8x8(float matrix[8][9], float solution[8]) {
  for (uint8_t column = 0; column < 8; ++column) {
    uint8_t pivot = column;
    float pivot_abs = std::fabs(matrix[pivot][column]);
    for (uint8_t row = static_cast<uint8_t>(column + 1U); row < 8; ++row) {
      const float value_abs = std::fabs(matrix[row][column]);
      if (value_abs > pivot_abs) {
        pivot = row;
        pivot_abs = value_abs;
      }
    }
    if (!std::isfinite(pivot_abs) || pivot_abs < 1.0e-8f) return false;
    if (pivot != column) {
      for (uint8_t k = column; k < 9; ++k) {
        std::swap(matrix[column][k], matrix[pivot][k]);
      }
    }
    const float divisor = matrix[column][column];
    for (uint8_t k = column; k < 9; ++k) matrix[column][k] /= divisor;
    for (uint8_t row = 0; row < 8; ++row) {
      if (row == column) continue;
      const float factor = matrix[row][column];
      if (std::fabs(factor) < 1.0e-12f) continue;
      for (uint8_t k = column; k < 9; ++k) {
        matrix[row][k] -= factor * matrix[column][k];
      }
    }
  }
  for (uint8_t row = 0; row < 8; ++row) {
    solution[row] = matrix[row][8];
    if (!std::isfinite(solution[row])) return false;
  }
  return true;
}

bool project_h(const float h[9], float u, float v, ImagePoint &point) {
  const float w = h[6] * u + h[7] * v + h[8];
  if (!std::isfinite(w) || std::fabs(w) < 1.0e-8f) return false;
  point.x = (h[0] * u + h[1] * v + h[2]) / w;
  point.y = (h[3] * u + h[4] * v + h[5]) / w;
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool fit_homography(PatternFeature *features, uint16_t count,
                    float h[9], uint16_t &inliers,
                    float &rms, float &max_residual) {
  if (features == nullptr || count < 8) return false;

  // Buffers persistants : eviter ~5 Ko de tableaux temporaires sur la pile
  // FreeRTOS pendant la detection multi-marqueurs.
  static float weights[MAX_COMPOSITE_FEATURES];
  static float residuals[MAX_COMPOSITE_FEATURES];
  if (count > MAX_COMPOSITE_FEATURES) return false;
  for (uint16_t i = 0; i < count; ++i) {
    weights[i] = features[i].inlier ? 1.0f : 0.0f;
    residuals[i] = 0.0f;
  }

  for (uint8_t iteration = 0; iteration < 3; ++iteration) {
    static float normal[8][9];
    std::memset(normal, 0, sizeof(normal));
    uint16_t active = 0;

    for (uint16_t i = 0; i < count; ++i) {
      if (weights[i] <= 0.0f) continue;
      const auto &f = features[i];
      const float strength_weight =
          std::max(0.65f, std::min(1.50f, f.strength / 18.0f));
      const float weight = weights[i] * strength_weight;
      const float rows[2][9] = {
          {f.u, f.v, 1.0f, 0,0,0, -f.x*f.u, -f.x*f.v, f.x},
          {0,0,0, f.u, f.v, 1.0f, -f.y*f.u, -f.y*f.v, f.y},
      };
      for (uint8_t eq = 0; eq < 2; ++eq) {
        for (uint8_t row = 0; row < 8; ++row) {
          for (uint8_t col = 0; col < 8; ++col) {
            normal[row][col] +=
                weight * rows[eq][row] * rows[eq][col];
          }
          normal[row][8] += weight * rows[eq][row] * rows[eq][8];
        }
      }
      active++;
    }
    if (active < 4) return false;

    float solution[8];
    if (!solve_8x8(normal, solution)) return false;
    for (uint8_t i = 0; i < 8; ++i) h[i] = solution[i];
    h[8] = 1.0f;

    uint16_t residual_count = 0;
    for (uint16_t i = 0; i < count; ++i) {
      if (weights[i] <= 0.0f) continue;
      ImagePoint projected;
      if (!project_h(h, features[i].u, features[i].v, projected)) return false;
      const float dx = projected.x - features[i].x;
      const float dy = projected.y - features[i].y;
      residuals[residual_count++] = std::sqrt(dx*dx + dy*dy);
      features[i].residual = residuals[residual_count - 1U];
    }
    if (residual_count < 4) return false;
    std::sort(residuals, residuals + residual_count);
    const float median = residuals[residual_count / 2U];
    const float huber = std::max(0.20f, std::min(1.20f, 2.5f * median));

    for (uint16_t i = 0; i < count; ++i) {
      if (weights[i] <= 0.0f) continue;
      const float r = std::max(1.0e-5f, features[i].residual);
      weights[i] = r <= huber ? 1.0f : huber / r;
    }
  }

  static float sorted[MAX_COMPOSITE_FEATURES];
  uint16_t sorted_count = 0;
  for (uint16_t i = 0; i < count; ++i) {
    ImagePoint projected;
    if (!project_h(h, features[i].u, features[i].v, projected)) return false;
    const float dx = projected.x - features[i].x;
    const float dy = projected.y - features[i].y;
    features[i].residual = std::sqrt(dx*dx + dy*dy);
    sorted[sorted_count++] = features[i].residual;
  }
  std::sort(sorted, sorted + sorted_count);
  const float median = sorted[sorted_count / 2U];
  const float limit = std::max(0.35f, std::min(1.50f, 3.0f * median));

  double sum_sq = 0.0;
  inliers = 0;
  max_residual = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    features[i].inlier = features[i].residual <= limit;
    if (!features[i].inlier) continue;
    sum_sq += static_cast<double>(features[i].residual) * features[i].residual;
    max_residual = std::max(max_residual, features[i].residual);
    inliers++;
  }
  if (inliers < 4) return false;
  rms = static_cast<float>(std::sqrt(sum_sq / inliers));
  return std::isfinite(rms);
}

void set_line(ImageLine &line, const ImagePoint &a, const ImagePoint &b) {
  line.point = a;
  line.dx = b.x - a.x;
  line.dy = b.y - a.y;
  const float norm = std::sqrt(line.dx * line.dx + line.dy * line.dy);
  line.valid = std::isfinite(norm) && norm > 1.0e-5f;
  if (line.valid) {
    line.dx /= norm;
    line.dy /= norm;
  }
}

}  // namespace

const TargetMarkerSpec &target_r1_marker_spec(TargetMarkerId id) {
  const int index = target_r1_marker_index(id);
  return MARKERS[index >= 0 ? index : 1];
}

int target_r1_marker_index(TargetMarkerId id) {
  switch (id) {
    case TargetMarkerId::A: return 0;
    case TargetMarkerId::B: return 1;
    case TargetMarkerId::C: return 2;
    default: return -1;
  }
}

uint8_t target_r1_marker_cell(TargetMarkerId id, uint8_t row, uint8_t column) {
  if (row >= 7 || column >= 7) return 0;
  switch (id) {
    case TargetMarkerId::A: return GRID_A[row][column];
    case TargetMarkerId::C: return GRID_C[row][column];
    case TargetMarkerId::B:
    default:
      return GRID_B[row][column];
  }
}

bool build_target_r1_observation(
    const TargetObservation (&markers)[3],
    PatternFeature *composite_features,
    uint16_t composite_capacity,
    TargetObservation &result) {
  result = TargetObservation();
  if (composite_features == nullptr || composite_capacity < 8) return false;

  uint8_t marker_count = 0;
  uint8_t marker_mask = 0;
  bool all_subpixel = true;
  float quality_sum = 0.0f;
  float width_gradient_sum = 0.0f;
  float height_gradient_sum = 0.0f;
  uint8_t gradient_count = 0;
  uint16_t feature_count = 0;

  for (uint8_t m = 0; m < 3; ++m) {
    const TargetObservation &marker = markers[m];
    if (!marker.valid) continue;
    const TargetMarkerSpec &spec = MARKERS[m];
    marker_count++;
    marker_mask |= static_cast<uint8_t>(1U << m);
    quality_sum += marker.quality;
    all_subpixel = all_subpixel && marker.subpixel_refined;

    if (marker.subpixel_width_gradient > 0.0f &&
        marker.subpixel_height_gradient > 0.0f) {
      width_gradient_sum += marker.subpixel_width_gradient;
      height_gradient_sum += marker.subpixel_height_gradient;
      gradient_count++;
    }

    if (marker.pattern_refined &&
        marker.pattern_features != nullptr) {
      for (uint16_t i = 0;
           i < marker.pattern_features_count &&
           feature_count < composite_capacity;
           ++i) {
        const PatternFeature &source = marker.pattern_features[i];
        if (!source.inlier) continue;
        PatternFeature &dest = composite_features[feature_count++];
        const float x_mm = spec.x_mm + source.u * spec.size_mm;
        const float y_mm = spec.y_top_mm + source.v * spec.size_mm;
        dest.u = (x_mm - TARGET_R1_REFERENCE_X_MM) /
                 TARGET_R1_REFERENCE_WIDTH_MM;
        dest.v = (y_mm - TARGET_R1_REFERENCE_Y_MM) /
                 TARGET_R1_REFERENCE_HEIGHT_MM;
        dest.x = source.x;
        dest.y = source.y;
        dest.strength = source.strength;
        dest.residual = source.residual;
        dest.inlier = true;
      }
    }
  }

  if (marker_count < 2) return false;

  // Si le raffinement interne n'a pas fourni assez de points, utiliser les
  // coins canoniques des marqueurs pour le tracking. La mesure haute precision
  // restera desactivee tant que les trois marqueurs subpixel ne sont pas la.
  if (feature_count < MIN_PATTERN_FEATURES) {
    feature_count = 0;
    for (uint8_t m = 0; m < 3; ++m) {
      const TargetObservation &marker = markers[m];
      if (!marker.valid) continue;
      const TargetMarkerSpec &spec = MARKERS[m];
      ImagePoint corners[4];
      canonical_corners(marker, corners);
      const float x0 = spec.x_mm;
      const float y0 = spec.y_top_mm;
      const float x1 = spec.x_mm + spec.size_mm;
      const float y1 = spec.y_top_mm + spec.size_mm;
      const float board_xy[4][2] = {
          {x0,y0},{x1,y0},{x1,y1},{x0,y1},
      };
      for (uint8_t k = 0; k < 4 && feature_count < composite_capacity; ++k) {
        auto &f = composite_features[feature_count++];
        f.u = (board_xy[k][0] - TARGET_R1_REFERENCE_X_MM) /
              TARGET_R1_REFERENCE_WIDTH_MM;
        f.v = (board_xy[k][1] - TARGET_R1_REFERENCE_Y_MM) /
              TARGET_R1_REFERENCE_HEIGHT_MM;
        f.x = corners[k].x;
        f.y = corners[k].y;
        f.strength = 20.0f;
        f.residual = 0.0f;
        f.inlier = true;
      }
    }
  }

  float h[9] = {0,0,0,0,0,0,0,0,1};
  uint16_t inliers = 0;
  float rms = 0.0f;
  float max_residual = 0.0f;
  if (!fit_homography(
          composite_features, feature_count,
          h, inliers, rms, max_residual)) {
    return false;
  }

  ImagePoint corners[4];
  if (!project_h(h, 0.0f, 0.0f, corners[0]) ||
      !project_h(h, 1.0f, 0.0f, corners[1]) ||
      !project_h(h, 1.0f, 1.0f, corners[2]) ||
      !project_h(h, 0.0f, 1.0f, corners[3])) {
    return false;
  }
  ImagePoint center;
  if (!project_h(h, 0.5f, 0.5f, center)) return false;

  const float width_px =
      0.5f * (point_distance(corners[0], corners[1]) +
              point_distance(corners[3], corners[2]));
  const float height_px =
      0.5f * (point_distance(corners[0], corners[3]) +
              point_distance(corners[1], corners[2]));
  if (!std::isfinite(width_px) || !std::isfinite(height_px) ||
      width_px < 20.0f || height_px < 10.0f) {
    return false;
  }

  result.valid = true;
  result.marker_id = TargetMarkerId::BOARD_R1;
  result.board_marker_mask = marker_mask;
  result.board_marker_count = marker_count;
  result.board_complete = marker_count == 3;
  result.center_x_px = center.x;
  result.center_y_px = center.y;
  result.width_px = width_px;
  result.height_px = height_px;
  result.rotation_deg = 0.0f;
  result.quality =
      (quality_sum / marker_count) *
      (marker_count == 3 ? 1.0f : 0.88f);

  result.top_left_px = corners[0];
  result.top_right_px = corners[1];
  result.bottom_right_px = corners[2];
  result.bottom_left_px = corners[3];

  const bool precision_ready =
      result.board_complete && all_subpixel &&
      feature_count >= MIN_PATTERN_FEATURES &&
      inliers >= MIN_PATTERN_FEATURES;

  result.subpixel_refined = precision_ready;
  result.subpixel_rms_px = rms;
  result.subpixel_max_rms_px = max_residual;
  result.subpixel_gradient =
      gradient_count > 0
          ? 0.5f * (width_gradient_sum + height_gradient_sum) / gradient_count
          : 0.0f;
  result.subpixel_width_px = width_px;
  result.subpixel_height_px = height_px;
  const float sigma =
      std::max(0.010f,
               rms / std::sqrt(static_cast<float>(std::max<uint16_t>(1, inliers))));
  result.subpixel_width_sigma_px = sigma;
  result.subpixel_height_sigma_px = sigma;
  result.subpixel_width_gradient =
      gradient_count > 0 ? width_gradient_sum / gradient_count : 0.0f;
  result.subpixel_height_gradient =
      gradient_count > 0 ? height_gradient_sum / gradient_count : 0.0f;

  result.subpixel_v5_width_px = width_px;
  result.subpixel_v5_height_px = height_px;
  result.subpixel_v5_width_sigma_px = sigma;
  result.subpixel_v5_height_sigma_px = sigma;
  result.subpixel_v6_width_px = width_px;
  result.subpixel_v6_height_px = height_px;
  result.subpixel_v6_width_sigma_px = sigma;
  result.subpixel_v6_height_sigma_px = sigma;

  result.subpixel_top_rms_px = rms;
  result.subpixel_right_rms_px = rms;
  result.subpixel_bottom_rms_px = rms;
  result.subpixel_left_rms_px = rms;
  result.subpixel_top_gradient = result.subpixel_height_gradient;
  result.subpixel_bottom_gradient = result.subpixel_height_gradient;
  result.subpixel_left_gradient = result.subpixel_width_gradient;
  result.subpixel_right_gradient = result.subpixel_width_gradient;

  set_line(result.subpixel_top_line, corners[0], corners[1]);
  set_line(result.subpixel_right_line, corners[1], corners[2]);
  set_line(result.subpixel_bottom_line, corners[3], corners[2]);
  set_line(result.subpixel_left_line, corners[0], corners[3]);

  result.pattern_refined = precision_ready;
  result.pattern_feature_count = feature_count;
  result.pattern_inlier_count = inliers;
  result.pattern_rms_px = rms;
  result.pattern_max_residual_px = max_residual;
  for (uint8_t i = 0; i < 9; ++i) result.pattern_homography[i] = h[i];
  result.pattern_features = composite_features;
  result.pattern_features_count = feature_count;
  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
