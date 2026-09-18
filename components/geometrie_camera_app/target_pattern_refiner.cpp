#include "target_pattern_refiner.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_pattern_refiner";

constexpr uint8_t TARGET_GRID[7][7] = {
    {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 1, 1},
    {1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 1, 1, 1, 1},
    {1, 1, 0, 0, 1, 0, 1},
    {1, 1, 1, 1, 1, 1, 1},
};

constexpr float SEARCH_RADIUS_PX = 2.50f;
constexpr float SEARCH_STEP_PX = 0.25f;
constexpr int SEARCH_SAMPLE_COUNT = 21;
constexpr float MIN_ORIENTED_GRADIENT = 4.0f;
constexpr uint16_t MIN_FEATURES = 16;
constexpr uint16_t MIN_INLIERS = 12;
constexpr float MAX_ACCEPTED_RMS_PX = 0.85f;
constexpr float MAX_ACCEPTED_RESIDUAL_PX = 2.00f;
constexpr float DERIVATIVE_EPS_UV = 0.0025f;

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

    if (!std::isfinite(pivot_abs) || pivot_abs < 1.0e-7f) {
      return false;
    }

    if (pivot != column) {
      for (uint8_t k = column; k < 9; ++k) {
        std::swap(matrix[column][k], matrix[pivot][k]);
      }
    }

    const float divisor = matrix[column][column];
    for (uint8_t k = column; k < 9; ++k) {
      matrix[column][k] /= divisor;
    }

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

bool project_homography(const float h[9], float u, float v,
                        float &x, float &y) {
  const float w = h[6] * u + h[7] * v + h[8];
  if (!std::isfinite(w) || std::fabs(w) < 1.0e-7f) return false;
  x = (h[0] * u + h[1] * v + h[2]) / w;
  y = (h[3] * u + h[4] * v + h[5]) / w;
  return std::isfinite(x) && std::isfinite(y);
}

float clampf(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}
}  // namespace

TargetPatternMetrics::TargetPatternMetrics()
    : valid(false),
      feature_count(0),
      inlier_count(0),
      rms_px(0.0f),
      max_residual_px(0.0f),
      homography{0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 1.0f},
      features(nullptr),
      features_count(0) {}

TargetPatternRefiner::TargetPatternRefiner() {}

bool TargetPatternRefiner::refine(
    const GrayFrameView &frame,
    const TargetCandidate &candidate,
    uint8_t rotation_quarters,
    TargetPatternMetrics &metrics) const {
  metrics = TargetPatternMetrics();

  if (frame.data == nullptr || frame.width < 8 || frame.height < 8 ||
      frame.stride < frame.width ||
      candidate.width < 20.0f || candidate.height < 20.0f) {
    return false;
  }

  PatternFeature *features = this->features_;
  uint16_t feature_count = 0;
  const float segment_positions[2] = {0.33f, 0.67f};

  for (uint8_t row = 0; row < 7 && feature_count < MAX_FEATURES; ++row) {
    for (uint8_t column = 1; column < 7 && feature_count < MAX_FEATURES; ++column) {
      const uint8_t before = this->expected_cell_(
          row, static_cast<uint8_t>(column - 1U), rotation_quarters);
      const uint8_t after = this->expected_cell_(
          row, column, rotation_quarters);
      if (before == after) continue;

      const int expected_sign =
          (before != 0 && after == 0) ? 1 : -1;
      for (float segment : segment_positions) {
        if (feature_count >= MAX_FEATURES) break;
        const float u = static_cast<float>(column) / 7.0f;
        const float v =
            (static_cast<float>(row) + segment) / 7.0f;
        PatternFeature feature{};
        if (this->find_transition_(
                frame, candidate, u, v, true,
                expected_sign, rotation_quarters, feature)) {
          features[feature_count++] = feature;
        }
      }
    }
  }

  for (uint8_t row = 1; row < 7 && feature_count < MAX_FEATURES; ++row) {
    for (uint8_t column = 0; column < 7 && feature_count < MAX_FEATURES; ++column) {
      const uint8_t before = this->expected_cell_(
          static_cast<uint8_t>(row - 1U), column, rotation_quarters);
      const uint8_t after = this->expected_cell_(
          row, column, rotation_quarters);
      if (before == after) continue;

      const int expected_sign =
          (before != 0 && after == 0) ? 1 : -1;
      for (float segment : segment_positions) {
        if (feature_count >= MAX_FEATURES) break;
        const float u =
            (static_cast<float>(column) + segment) / 7.0f;
        const float v = static_cast<float>(row) / 7.0f;
        PatternFeature feature{};
        if (this->find_transition_(
                frame, candidate, u, v, false,
                expected_sign, rotation_quarters, feature)) {
          features[feature_count++] = feature;
        }
      }
    }
  }

  metrics.feature_count = feature_count;
  metrics.features = this->features_;
  metrics.features_count = feature_count;
  if (feature_count < MIN_FEATURES) {
    ESP_LOGD(TAG, "Pose V3 motif: seulement %u transitions valides",
             static_cast<unsigned>(feature_count));
    return false;
  }

  uint16_t inlier_count = 0;
  float rms_px = 0.0f;
  float max_residual_px = 0.0f;
  if (!this->fit_homography_(
          features, feature_count, metrics.homography,
          inlier_count, rms_px, max_residual_px)) {
    return false;
  }

  metrics.inlier_count = inlier_count;
  metrics.rms_px = rms_px;
  metrics.max_residual_px = max_residual_px;
  metrics.valid =
      inlier_count >= MIN_INLIERS &&
      rms_px <= MAX_ACCEPTED_RMS_PX &&
      max_residual_px <= MAX_ACCEPTED_RESIDUAL_PX;

  ESP_LOGD(TAG,
           "Pose V3 motif: features=%u inliers=%u rms=%.3f max=%.3f valid=%s",
           static_cast<unsigned>(metrics.feature_count),
           static_cast<unsigned>(metrics.inlier_count),
           metrics.rms_px, metrics.max_residual_px,
           metrics.valid ? "YES" : "NO");
  return metrics.valid;
}

TargetPoint TargetPatternRefiner::project_candidate_(
    const TargetCandidate &candidate, float u, float v) const {
  const float x0 = candidate.top_left.x;
  const float y0 = candidate.top_left.y;
  const float x1 = candidate.top_right.x;
  const float y1 = candidate.top_right.y;
  const float x2 = candidate.bottom_right.x;
  const float y2 = candidate.bottom_right.y;
  const float x3 = candidate.bottom_left.x;
  const float y3 = candidate.bottom_left.y;

  const float sx = x0 - x1 + x2 - x3;
  const float sy = y0 - y1 + y2 - y3;
  const float dx1 = x1 - x2;
  const float dx2 = x3 - x2;
  const float dy1 = y1 - y2;
  const float dy2 = y3 - y2;

  float g = 0.0f;
  float h = 0.0f;
  const float denominator = dx1 * dy2 - dx2 * dy1;
  if ((std::fabs(sx) > 0.0001f || std::fabs(sy) > 0.0001f) &&
      std::fabs(denominator) > 0.0001f) {
    g = (sx * dy2 - dx2 * sy) / denominator;
    h = (dx1 * sy - sx * dy1) / denominator;
  }

  const float a = x1 - x0 + g * x1;
  const float b = x3 - x0 + h * x3;
  const float c = x0;
  const float d = y1 - y0 + g * y1;
  const float e = y3 - y0 + h * y3;
  const float f = y0;
  const float w = g * u + h * v + 1.0f;

  TargetPoint point;
  if (std::fabs(w) > 0.0001f) {
    point.x = (a * u + b * v + c) / w;
    point.y = (d * u + e * v + f) / w;
  } else {
    const float top_x = x0 + (x1 - x0) * u;
    const float top_y = y0 + (y1 - y0) * u;
    const float bottom_x = x3 + (x2 - x3) * u;
    const float bottom_y = y3 + (y2 - y3) * u;
    point.x = top_x + (bottom_x - top_x) * v;
    point.y = top_y + (bottom_y - top_y) * v;
  }
  return point;
}

float TargetPatternRefiner::sample_bilinear_(
    const GrayFrameView &frame, float x, float y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return NAN;
  if (x < 0.0f || y < 0.0f ||
      x > static_cast<float>(frame.width - 1U) ||
      y > static_cast<float>(frame.height - 1U)) {
    return NAN;
  }

  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min<int>(frame.width - 1, x0 + 1);
  const int y1 = std::min<int>(frame.height - 1, y0 + 1);
  const float fx = x - static_cast<float>(x0);
  const float fy = y - static_cast<float>(y0);

  const float p00 = frame.data[static_cast<size_t>(y0) * frame.stride + x0];
  const float p10 = frame.data[static_cast<size_t>(y0) * frame.stride + x1];
  const float p01 = frame.data[static_cast<size_t>(y1) * frame.stride + x0];
  const float p11 = frame.data[static_cast<size_t>(y1) * frame.stride + x1];

  const float top = p00 + (p10 - p00) * fx;
  const float bottom = p01 + (p11 - p01) * fx;
  return top + (bottom - top) * fy;
}

uint8_t TargetPatternRefiner::expected_cell_(
    uint8_t row, uint8_t column, uint8_t rotation_quarters) const {
  rotation_quarters &= 0x03U;
  switch (rotation_quarters) {
    case 1:
      return TARGET_GRID[6U - column][row];
    case 2:
      return TARGET_GRID[6U - row][6U - column];
    case 3:
      return TARGET_GRID[column][6U - row];
    default:
      return TARGET_GRID[row][column];
  }
}

void TargetPatternRefiner::canonical_uv_(
    float observed_u, float observed_v,
    uint8_t rotation_quarters,
    float &canonical_u, float &canonical_v) const {
  rotation_quarters &= 0x03U;
  switch (rotation_quarters) {
    case 1:
      canonical_u = observed_v;
      canonical_v = 1.0f - observed_u;
      break;
    case 2:
      canonical_u = 1.0f - observed_u;
      canonical_v = 1.0f - observed_v;
      break;
    case 3:
      canonical_u = 1.0f - observed_v;
      canonical_v = observed_u;
      break;
    default:
      canonical_u = observed_u;
      canonical_v = observed_v;
      break;
  }
}

bool TargetPatternRefiner::find_transition_(
    const GrayFrameView &frame,
    const TargetCandidate &candidate,
    float observed_u, float observed_v,
    bool along_u,
    int expected_sign,
    uint8_t rotation_quarters,
    PatternFeature &feature) const {
  const TargetPoint base =
      this->project_candidate_(candidate, observed_u, observed_v);

  TargetPoint minus;
  TargetPoint plus;
  if (along_u) {
    minus = this->project_candidate_(
        candidate, observed_u - DERIVATIVE_EPS_UV, observed_v);
    plus = this->project_candidate_(
        candidate, observed_u + DERIVATIVE_EPS_UV, observed_v);
  } else {
    minus = this->project_candidate_(
        candidate, observed_u, observed_v - DERIVATIVE_EPS_UV);
    plus = this->project_candidate_(
        candidate, observed_u, observed_v + DERIVATIVE_EPS_UV);
  }

  float nx = plus.x - minus.x;
  float ny = plus.y - minus.y;
  const float norm = std::sqrt(nx * nx + ny * ny);
  if (!std::isfinite(norm) || norm < 1.0e-5f) return false;
  nx /= norm;
  ny /= norm;

  float gradients[SEARCH_SAMPLE_COUNT];
  float best_gradient = -1.0e30f;
  int best_index = -1;

  for (int index = 0; index < SEARCH_SAMPLE_COUNT; ++index) {
    const float t =
        -SEARCH_RADIUS_PX +
        static_cast<float>(index) * SEARCH_STEP_PX;
    const float before = this->sample_bilinear_(
        frame,
        base.x + nx * (t - SEARCH_STEP_PX),
        base.y + ny * (t - SEARCH_STEP_PX));
    const float after = this->sample_bilinear_(
        frame,
        base.x + nx * (t + SEARCH_STEP_PX),
        base.y + ny * (t + SEARCH_STEP_PX));

    if (!std::isfinite(before) || !std::isfinite(after)) {
      gradients[index] = -1.0e30f;
      continue;
    }

    gradients[index] =
        static_cast<float>(expected_sign) * (after - before);
    if (gradients[index] > best_gradient) {
      best_gradient = gradients[index];
      best_index = index;
    }
  }

  if (best_index <= 0 ||
      best_index >= SEARCH_SAMPLE_COUNT - 1 ||
      best_gradient < MIN_ORIENTED_GRADIENT) {
    return false;
  }

  const float gm = gradients[best_index - 1];
  const float g0 = gradients[best_index];
  const float gp = gradients[best_index + 1];
  const float denominator = gm - 2.0f * g0 + gp;
  float fractional = 0.0f;
  if (std::isfinite(denominator) &&
      std::fabs(denominator) > 1.0e-5f) {
    fractional = clampf(
        0.5f * (gm - gp) / denominator,
        -1.0f, 1.0f);
  }

  const float refined_t =
      -SEARCH_RADIUS_PX +
      (static_cast<float>(best_index) + fractional) *
          SEARCH_STEP_PX;

  this->canonical_uv_(
      observed_u, observed_v, rotation_quarters,
      feature.u, feature.v);
  feature.x = base.x + nx * refined_t;
  feature.y = base.y + ny * refined_t;
  feature.strength = best_gradient;
  feature.residual = 0.0f;

  return std::isfinite(feature.x) && std::isfinite(feature.y);
}

bool TargetPatternRefiner::fit_homography_(
    PatternFeature *features, uint16_t count,
    float homography[9],
    uint16_t &inlier_count,
    float &rms_px,
    float &max_residual_px) const {
  if (features == nullptr || count < MIN_FEATURES) return false;

  float *robust_weights = this->robust_weights_;
  float *residuals = this->residuals_;
  for (uint16_t i = 0; i < count; ++i) {
    robust_weights[i] = 1.0f;
    residuals[i] = 0.0f;
  }

  float h[9] = {0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};

  for (uint8_t iteration = 0; iteration < 3; ++iteration) {
    float (*normal)[9] = this->normal_matrix_;
    std::memset(this->normal_matrix_, 0, sizeof(this->normal_matrix_));

    for (uint16_t i = 0; i < count; ++i) {
      const PatternFeature &f = features[i];
      const float strength_weight =
          clampf(f.strength / 18.0f, 0.60f, 1.60f);
      const float weight = strength_weight * robust_weights[i];

      const float rows[2][9] = {
          {f.u, f.v, 1.0f, 0.0f, 0.0f, 0.0f,
           -f.x * f.u, -f.x * f.v, f.x},
          {0.0f, 0.0f, 0.0f, f.u, f.v, 1.0f,
           -f.y * f.u, -f.y * f.v, f.y},
      };

      for (uint8_t equation = 0; equation < 2; ++equation) {
        for (uint8_t row = 0; row < 8; ++row) {
          for (uint8_t col = 0; col < 8; ++col) {
            normal[row][col] +=
                weight * rows[equation][row] *
                rows[equation][col];
          }
          normal[row][8] +=
              weight * rows[equation][row] *
              rows[equation][8];
        }
      }
    }

    float solution[8];
    if (!solve_8x8(normal, solution)) return false;
    for (uint8_t i = 0; i < 8; ++i) h[i] = solution[i];
    h[8] = 1.0f;

    for (uint16_t i = 0; i < count; ++i) {
      float px = 0.0f;
      float py = 0.0f;
      if (!project_homography(
              h, features[i].u, features[i].v, px, py)) {
        return false;
      }
      const float dx = px - features[i].x;
      const float dy = py - features[i].y;
      features[i].residual = std::sqrt(dx * dx + dy * dy);
      residuals[i] = features[i].residual;
    }

    std::sort(residuals, residuals + count);
    const float median = residuals[count / 2U];
    const float huber =
        clampf(2.5f * median, 0.30f, 1.25f);

    for (uint16_t i = 0; i < count; ++i) {
      const float residual =
          std::max(1.0e-4f, features[i].residual);
      robust_weights[i] =
          residual <= huber ? 1.0f : huber / residual;
    }
  }

  for (uint16_t i = 0; i < count; ++i) {
    float px = 0.0f;
    float py = 0.0f;
    if (!project_homography(
            h, features[i].u, features[i].v, px, py)) {
      return false;
    }
    const float dx = px - features[i].x;
    const float dy = py - features[i].y;
    features[i].residual = std::sqrt(dx * dx + dy * dy);
    residuals[i] = features[i].residual;
  }

  std::sort(residuals, residuals + count);
  const float median = residuals[count / 2U];
  const float inlier_limit =
      clampf(3.0f * median, 0.40f, 1.50f);

  double sum_sq = 0.0;
  inlier_count = 0;
  max_residual_px = 0.0f;
  for (uint16_t i = 0; i < count; ++i) {
    features[i].inlier = features[i].residual <= inlier_limit;
    if (features[i].inlier) {
      sum_sq +=
          static_cast<double>(features[i].residual) *
          features[i].residual;
      max_residual_px =
          std::max(max_residual_px, features[i].residual);
      inlier_count++;
    }
  }

  if (inlier_count < MIN_INLIERS) return false;
  rms_px = static_cast<float>(
      std::sqrt(sum_sq / static_cast<double>(inlier_count)));

  for (uint8_t i = 0; i < 9; ++i) {
    homography[i] = h[i];
  }
  return true;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
