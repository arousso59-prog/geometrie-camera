#include "geometry_measurement.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr float RAD_TO_DEG_F = 57.29577951308232f;
constexpr float MIN_VECTOR_NORM = 1.0e-6f;
constexpr float MAX_POSE_SCALE_ERROR_PCT = 25.0f;
constexpr float DISTANCE_BLEND_START_RATIO = 0.03f;
constexpr float DISTANCE_BLEND_FULL_RATIO = 0.15f;
constexpr float POSE_V2_MAX_LINE_RMS_PX = 1.20f;
constexpr float POSE_V2_MAX_CORNER_RMS_PX = 2.50f;
constexpr float POSE_V2_MIN_NORMAL_Z = 0.05f;
constexpr float POSE_V2_CORNER_COST_WEIGHT = 0.10f;

struct Vec3 {
  float x;
  float y;
  float z;
};

float norm(const Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 scale(const Vec3 &value, float factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

float dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

bool normalize(Vec3 &value) {
  const float length = norm(value);
  if (!std::isfinite(length) || length < MIN_VECTOR_NORM) {
    return false;
  }
  value = scale(value, 1.0f / length);
  return true;
}

float point_distance(const ImagePoint &a, const ImagePoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool distortion_is_zero(const CameraCalibration &calibration) {
  constexpr float EPS = 1.0e-12f;
  return std::fabs(calibration.k1) < EPS &&
         std::fabs(calibration.k2) < EPS &&
         std::fabs(calibration.p1) < EPS &&
         std::fabs(calibration.p2) < EPS &&
         std::fabs(calibration.k3) < EPS;
}

ImagePoint undistort_point(const ImagePoint &point, const CameraCalibration &calibration) {
  if (distortion_is_zero(calibration) ||
      calibration.fx_px <= 0.0f || calibration.fy_px <= 0.0f) {
    return point;
  }

  const float xd = (point.x - calibration.cx_px) / calibration.fx_px;
  const float yd = (point.y - calibration.cy_px) / calibration.fy_px;

  float xu = xd;
  float yu = yd;
  for (uint8_t iteration = 0; iteration < 6; ++iteration) {
    const float x2 = xu * xu;
    const float y2 = yu * yu;
    const float r2 = x2 + y2;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float radial = 1.0f + calibration.k1 * r2 +
                         calibration.k2 * r4 + calibration.k3 * r6;
    if (!std::isfinite(radial) || std::fabs(radial) < 1.0e-6f) {
      return point;
    }

    const float delta_x = 2.0f * calibration.p1 * xu * yu +
                          calibration.p2 * (r2 + 2.0f * x2);
    const float delta_y = calibration.p1 * (r2 + 2.0f * y2) +
                          2.0f * calibration.p2 * xu * yu;

    xu = (xd - delta_x) / radial;
    yu = (yd - delta_y) / radial;
    if (!std::isfinite(xu) || !std::isfinite(yu)) {
      return point;
    }
  }

  ImagePoint result;
  result.x = calibration.fx_px * xu + calibration.cx_px;
  result.y = calibration.fy_px * yu + calibration.cy_px;
  return result;
}

void undistort_points(ImagePoint (&points)[4], const CameraCalibration &calibration) {
  if (distortion_is_zero(calibration)) {
    return;
  }
  for (auto &point : points) {
    point = undistort_point(point, calibration);
  }
}

ImagePoint quadrilateral_center(const ImagePoint (&points)[4]) {
  // L'intersection des diagonales est l'image projective du centre du carre.
  // Elle exploite directement les quatre coins raffines et evite de reutiliser
  // le centre grossier du candidat de localisation.
  const float rx = points[2].x - points[0].x;
  const float ry = points[2].y - points[0].y;
  const float sx = points[3].x - points[1].x;
  const float sy = points[3].y - points[1].y;
  const float denominator = rx * sy - ry * sx;

  ImagePoint center;
  if (std::fabs(denominator) > 1.0e-6f) {
    const float qpx = points[1].x - points[0].x;
    const float qpy = points[1].y - points[0].y;
    const float t = (qpx * sy - qpy * sx) / denominator;
    center.x = points[0].x + t * rx;
    center.y = points[0].y + t * ry;
    if (std::isfinite(center.x) && std::isfinite(center.y)) {
      return center;
    }
  }

  center.x = 0.25f * (points[0].x + points[1].x + points[2].x + points[3].x);
  center.y = 0.25f * (points[0].y + points[1].y + points[2].y + points[3].y);
  return center;
}

float fuse_size_distance(float z_from_width, float z_from_height,
                         float width_weight, float height_weight) {
  const float lower = std::min(z_from_width, z_from_height);
  const float upper = std::max(z_from_width, z_from_height);
  if (!std::isfinite(z_from_width) || !std::isfinite(z_from_height) ||
      z_from_width <= 0.0f || z_from_height <= 0.0f || lower <= 0.0f) {
    return lower;
  }

  float w_width = std::isfinite(width_weight) && width_weight > 0.0f
                      ? width_weight
                      : 1.0f;
  float w_height = std::isfinite(height_weight) && height_weight > 0.0f
                       ? height_weight
                       : 1.0f;

  // Ne jamais laisser un axe ecraser totalement l'autre : la seconde
  // dimension reste un controle geometrique utile. Le rapport max 16:1 donne
  // toutefois un avantage net a l'axe dont l'incertitude subpixel est faible.
  if (w_width > w_height * 16.0f) w_width = w_height * 16.0f;
  if (w_height > w_width * 16.0f) w_height = w_width * 16.0f;

  const float weight_sum = w_width + w_height;
  const float weighted =
      (w_width * z_from_width + w_height * z_from_height) / weight_sum;
  const float disagreement = (upper - lower) / lower;

  if (disagreement <= DISTANCE_BLEND_START_RATIO) {
    return weighted;
  }
  if (disagreement >= DISTANCE_BLEND_FULL_RATIO) {
    return lower;
  }

  // En cas d'inclinaison importante, l'estimation la plus basse reste la moins
  // affectee par le raccourcissement projectif. La transition reste continue.
  const float blend = (disagreement - DISTANCE_BLEND_START_RATIO) /
                      (DISTANCE_BLEND_FULL_RATIO - DISTANCE_BLEND_START_RATIO);
  return weighted + (lower - weighted) * blend;
}

float normalize_half_turn(float angle_deg) {
  while (angle_deg >= 90.0f) {
    angle_deg -= 180.0f;
  }
  while (angle_deg < -90.0f) {
    angle_deg += 180.0f;
  }
  return angle_deg;
}

void canonical_corners(const TargetObservation &observation, ImagePoint (&points)[4]) {
  const ImagePoint observed[4] = {
      observation.top_left_px,
      observation.top_right_px,
      observation.bottom_right_px,
      observation.bottom_left_px,
  };

  int rotation = static_cast<int>(std::lround(observation.rotation_deg / 90.0f));
  rotation %= 4;
  if (rotation < 0) {
    rotation += 4;
  }

  static const uint8_t MAP[4][4] = {
      {0, 1, 2, 3},
      {1, 2, 3, 0},
      {2, 3, 0, 1},
      {3, 0, 1, 2},
  };

  for (uint8_t index = 0; index < 4; ++index) {
    points[index] = observed[MAP[rotation][index]];
  }
}

bool build_unit_square_homography(const ImagePoint (&points)[4], float (&h)[9]) {
  const float x0 = points[0].x;
  const float y0 = points[0].y;
  const float x1 = points[1].x;
  const float y1 = points[1].y;
  const float x2 = points[2].x;
  const float y2 = points[2].y;
  const float x3 = points[3].x;
  const float y3 = points[3].y;

  const float sx = x0 - x1 + x2 - x3;
  const float sy = y0 - y1 + y2 - y3;
  const float dx1 = x1 - x2;
  const float dx2 = x3 - x2;
  const float dy1 = y1 - y2;
  const float dy2 = y3 - y2;
  const float denominator = dx1 * dy2 - dx2 * dy1;

  float g = 0.0f;
  float k = 0.0f;
  if (std::fabs(sx) > 0.0001f || std::fabs(sy) > 0.0001f) {
    if (std::fabs(denominator) < 0.0001f) {
      return false;
    }
    g = (sx * dy2 - dx2 * sy) / denominator;
    k = (dx1 * sy - sx * dy1) / denominator;
  }

  h[0] = x1 - x0 + g * x1;
  h[1] = x3 - x0 + k * x3;
  h[2] = x0;
  h[3] = y1 - y0 + g * y1;
  h[4] = y3 - y0 + k * y3;
  h[5] = y0;
  h[6] = g;
  h[7] = k;
  h[8] = 1.0f;
  return true;
}

Vec3 inverse_intrinsics_column(float hx, float hy, float hz,
                               const CameraCalibration &calibration) {
  return {
      (hx - calibration.cx_px * hz) / calibration.fx_px,
      (hy - calibration.cy_px * hz) / calibration.fy_px,
      hz,
  };
}

struct PoseBasis {
  Vec3 right;
  Vec3 down;
  Vec3 normal;
};

struct PoseV2Score {
  bool valid;
  float objective;
  float line_rms_px;
  float corner_rms_px;
};

bool normalize_basis(PoseBasis &basis) {
  if (!normalize(basis.right)) return false;

  basis.down = subtract(
      basis.down, scale(basis.right, dot(basis.down, basis.right)));
  if (!normalize(basis.down)) return false;

  basis.normal = cross(basis.right, basis.down);
  if (!normalize(basis.normal)) return false;

  // Conserver un repere direct et une normale qui regarde dans le meme
  // demi-espace que la decomposition homographique d'origine.
  basis.down = cross(basis.normal, basis.right);
  return normalize(basis.down);
}

Vec3 rotate_camera_axis(const Vec3 &value, uint8_t axis, float angle_rad) {
  const float cs = std::cos(angle_rad);
  const float sn = std::sin(angle_rad);
  if (axis == 0) {
    return {value.x,
            cs * value.y - sn * value.z,
            sn * value.y + cs * value.z};
  }
  if (axis == 1) {
    return {cs * value.x + sn * value.z,
            value.y,
            -sn * value.x + cs * value.z};
  }
  return {cs * value.x - sn * value.y,
          sn * value.x + cs * value.y,
          value.z};
}

PoseBasis rotate_basis_camera_axis(
    const PoseBasis &basis, uint8_t axis, float angle_rad) {
  PoseBasis rotated;
  rotated.right = rotate_camera_axis(basis.right, axis, angle_rad);
  rotated.down = rotate_camera_axis(basis.down, axis, angle_rad);
  rotated.normal = rotate_camera_axis(basis.normal, axis, angle_rad);
  normalize_basis(rotated);
  return rotated;
}

bool project_target_point(const Vec3 &local,
                          const PoseBasis &basis,
                          const Vec3 &translation,
                          const CameraCalibration &calibration,
                          ImagePoint &image) {
  const Vec3 camera = {
      translation.x +
          basis.right.x * local.x +
          basis.down.x * local.y +
          basis.normal.x * local.z,
      translation.y +
          basis.right.y * local.x +
          basis.down.y * local.y +
          basis.normal.y * local.z,
      translation.z +
          basis.right.z * local.x +
          basis.down.z * local.y +
          basis.normal.z * local.z,
  };

  if (!std::isfinite(camera.x) || !std::isfinite(camera.y) ||
      !std::isfinite(camera.z) || camera.z <= 1.0f) {
    return false;
  }

  image.x = calibration.fx_px * camera.x / camera.z +
            calibration.cx_px;
  image.y = calibration.fy_px * camera.y / camera.z +
            calibration.cy_px;
  return std::isfinite(image.x) && std::isfinite(image.y);
}

float point_line_distance(const ImagePoint &point, const ImageLine &line) {
  const float norm_dir =
      std::sqrt(line.dx * line.dx + line.dy * line.dy);
  if (!line.valid || !std::isfinite(norm_dir) || norm_dir < 1.0e-6f) {
    return NAN;
  }

  const float dx = line.dx / norm_dir;
  const float dy = line.dy / norm_dir;
  const float nx = -dy;
  const float ny = dx;
  return std::fabs(
      (point.x - line.point.x) * nx +
      (point.y - line.point.y) * ny);
}

void canonical_lines(const TargetObservation &observation,
                     ImageLine (&lines)[4],
                     float (&rms)[4],
                     float (&gradient)[4]) {
  const ImageLine observed_lines[4] = {
      observation.subpixel_top_line,
      observation.subpixel_right_line,
      observation.subpixel_bottom_line,
      observation.subpixel_left_line,
  };
  const float observed_rms[4] = {
      observation.subpixel_top_rms_px,
      observation.subpixel_right_rms_px,
      observation.subpixel_bottom_rms_px,
      observation.subpixel_left_rms_px,
  };
  const float observed_gradient[4] = {
      observation.subpixel_top_gradient,
      observation.subpixel_right_gradient,
      observation.subpixel_bottom_gradient,
      observation.subpixel_left_gradient,
  };

  int rotation =
      static_cast<int>(std::lround(observation.rotation_deg / 90.0f));
  rotation %= 4;
  if (rotation < 0) rotation += 4;

  static const uint8_t MAP[4][4] = {
      {0, 1, 2, 3},
      {1, 2, 3, 0},
      {2, 3, 0, 1},
      {3, 0, 1, 2},
  };

  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t source = MAP[rotation][i];
    lines[i] = observed_lines[source];
    rms[i] = observed_rms[source];
    gradient[i] = observed_gradient[source];
  }
}

float pose_edge_weight(float rms_px, float gradient) {
  const float bounded_rms =
      std::max(0.0f, std::min(0.60f, rms_px));
  // Poids volontairement doux : aucun bord ne doit pouvoir ecraser les trois
  // autres, meme si son RMS instantane est exceptionnellement faible.
  const float rms_weight = 1.0f / (1.0f + 3.0f * bounded_rms);
  const float gradient_weight =
      std::max(0.75f, std::min(1.35f, gradient / 30.0f));
  return rms_weight * gradient_weight;
}

PoseV2Score evaluate_pose_v2(
    const PoseBasis &basis,
    const Vec3 &translation,
    float target_size_mm,
    const CameraCalibration &calibration,
    const ImagePoint (&observed_corners)[4],
    const ImageLine (&observed_lines)[4],
    const float (&edge_rms)[4],
    const float (&edge_gradient)[4]) {
  PoseV2Score score{};
  score.valid = false;
  score.objective = 1.0e30f;
  score.line_rms_px = 0.0f;
  score.corner_rms_px = 0.0f;

  if (basis.normal.z <= POSE_V2_MIN_NORMAL_Z) {
    return score;
  }

  const float half = 0.5f * target_size_mm;
  const Vec3 model[4] = {
      {-half, -half, 0.0f},
      { half, -half, 0.0f},
      { half,  half, 0.0f},
      {-half,  half, 0.0f},
  };

  ImagePoint projected[4];
  for (uint8_t i = 0; i < 4; ++i) {
    if (!project_target_point(
            model[i], basis, translation, calibration, projected[i])) {
      return score;
    }
  }

  double line_sq = 0.0;
  double line_weight_sum = 0.0;
  for (uint8_t edge = 0; edge < 4; ++edge) {
    if (!observed_lines[edge].valid) return score;

    const uint8_t next = static_cast<uint8_t>((edge + 1U) & 0x03U);
    const float d0 =
        point_line_distance(projected[edge], observed_lines[edge]);
    const float d1 =
        point_line_distance(projected[next], observed_lines[edge]);
    if (!std::isfinite(d0) || !std::isfinite(d1)) return score;

    const float weight =
        pose_edge_weight(edge_rms[edge], edge_gradient[edge]);
    line_sq += static_cast<double>(weight) *
               (static_cast<double>(d0) * d0 +
                static_cast<double>(d1) * d1);
    line_weight_sum += 2.0 * weight;
  }

  if (line_weight_sum <= 0.0) return score;
  score.line_rms_px =
      static_cast<float>(std::sqrt(line_sq / line_weight_sum));

  double corner_sq = 0.0;
  for (uint8_t i = 0; i < 4; ++i) {
    const float dx = projected[i].x - observed_corners[i].x;
    const float dy = projected[i].y - observed_corners[i].y;
    corner_sq += static_cast<double>(dx) * dx +
                 static_cast<double>(dy) * dy;
  }
  score.corner_rms_px =
      static_cast<float>(std::sqrt(corner_sq / 4.0));

  score.objective =
      score.line_rms_px * score.line_rms_px +
      POSE_V2_CORNER_COST_WEIGHT *
          score.corner_rms_px * score.corner_rms_px;
  score.valid = std::isfinite(score.objective);
  return score;
}

bool refine_pose_v2(
    const PoseBasis &initial_basis,
    const Vec3 &translation,
    float target_size_mm,
    const CameraCalibration &calibration,
    const ImagePoint (&observed_corners)[4],
    const TargetObservation &observation,
    PoseBasis &refined_basis,
    PoseV2Score &refined_score) {
  ImageLine lines[4];
  float edge_rms[4];
  float edge_gradient[4];
  canonical_lines(observation, lines, edge_rms, edge_gradient);
  for (const auto &line : lines) {
    if (!line.valid) return false;
  }

  PoseBasis current = initial_basis;
  if (!normalize_basis(current)) return false;
  PoseV2Score current_score = evaluate_pose_v2(
      current, translation, target_size_mm, calibration,
      observed_corners, lines, edge_rms, edge_gradient);
  if (!current_score.valid) return false;

  // Recherche multi-echelle sur SO(3). Les pas finaux descendent sous la
  // minute d'arc ; le calcul reste tres leger face aux ~300 ms de detection.
  constexpr float STEPS_DEG[] = {
      3.0f, 0.75f, 0.20f, 0.05f, 0.01f, 0.003f,
  };

  for (float step_deg : STEPS_DEG) {
    const float step_rad = step_deg / RAD_TO_DEG_F;
    for (uint8_t pass = 0; pass < 6; ++pass) {
      bool improved = false;
      for (uint8_t axis = 0; axis < 3; ++axis) {
        PoseBasis best_basis = current;
        PoseV2Score best_score = current_score;

        static const int SIGNS[2] = {-1, 1};
        for (int sign : SIGNS) {
          PoseBasis candidate = rotate_basis_camera_axis(
              current, axis, static_cast<float>(sign) * step_rad);
          if (candidate.normal.z <= POSE_V2_MIN_NORMAL_Z) continue;

          const PoseV2Score candidate_score = evaluate_pose_v2(
              candidate, translation, target_size_mm, calibration,
              observed_corners, lines, edge_rms, edge_gradient);
          if (candidate_score.valid &&
              candidate_score.objective + 1.0e-8f <
                  best_score.objective) {
            best_basis = candidate;
            best_score = candidate_score;
          }
        }

        if (best_score.objective + 1.0e-8f <
            current_score.objective) {
          current = best_basis;
          current_score = best_score;
          improved = true;
        }
      }

      if (!improved) break;
    }
  }

  if (!current_score.valid ||
      current_score.line_rms_px > POSE_V2_MAX_LINE_RMS_PX ||
      current_score.corner_rms_px > POSE_V2_MAX_CORNER_RMS_PX) {
    return false;
  }

  refined_basis = current;
  refined_score = current_score;
  return true;
}

void pose_angles_from_basis(
    const PoseBasis &basis,
    float &yaw_deg,
    float &pitch_deg,
    float &roll_deg) {
  yaw_deg =
      std::atan2(basis.normal.x, basis.normal.z) * RAD_TO_DEG_F;
  pitch_deg =
      std::atan2(
          -basis.normal.y,
          std::sqrt(
              basis.normal.x * basis.normal.x +
              basis.normal.z * basis.normal.z)) *
      RAD_TO_DEG_F;

  Vec3 reference_right = {
      basis.normal.z, 0.0f, -basis.normal.x};
  if (!normalize(reference_right)) {
    reference_right = {1.0f, 0.0f, 0.0f};
  }

  Vec3 reference_down =
      cross(basis.normal, reference_right);
  if (!normalize(reference_down)) {
    reference_down = {0.0f, 1.0f, 0.0f};
  }

  roll_deg = normalize_half_turn(
      std::atan2(
          dot(basis.right, reference_down),
          dot(basis.right, reference_right)) *
      RAD_TO_DEG_F);
}
}

GeometryMeasurementEngine::GeometryMeasurementEngine()
    : calibration_(), target_size_mm_(50.0f) {}

bool GeometryMeasurementEngine::set_target_size_mm(float target_size_mm) {
  if (!std::isfinite(target_size_mm) || target_size_mm <= 0.0f) {
    return false;
  }
  this->target_size_mm_ = target_size_mm;
  return true;
}

float GeometryMeasurementEngine::target_size_mm() const {
  return this->target_size_mm_;
}

void GeometryMeasurementEngine::set_calibration(const CameraCalibration &calibration) {
  this->calibration_ = calibration;
}

void GeometryMeasurementEngine::clear_calibration() {
  this->calibration_ = CameraCalibration();
}

bool GeometryMeasurementEngine::has_calibration() const {
  return this->calibration_.fx_px > 0.0f && this->calibration_.fy_px > 0.0f &&
         this->calibration_.reference_width_px > 0 && this->calibration_.reference_height_px > 0;
}

const CameraCalibration &GeometryMeasurementEngine::calibration() const {
  return this->calibration_;
}

CameraCalibration GeometryMeasurementEngine::effective_calibration(uint16_t frame_width,
                                                                    uint16_t frame_height) const {
  CameraCalibration result;
  if (!this->has_calibration() || frame_width == 0 || frame_height == 0) {
    return result;
  }

  const float scale_x = static_cast<float>(frame_width) /
                        static_cast<float>(this->calibration_.reference_width_px);
  const float scale_y = static_cast<float>(frame_height) /
                        static_cast<float>(this->calibration_.reference_height_px);

  result.fx_px = this->calibration_.fx_px * scale_x;
  result.fy_px = this->calibration_.fy_px * scale_y;
  result.cx_px = (this->calibration_.cx_px + 0.5f) * scale_x - 0.5f;
  result.cy_px = (this->calibration_.cy_px + 0.5f) * scale_y - 0.5f;
  // Les coefficients Brown-Conrady sont exprimes en coordonnees normalisees :
  // ils ne changent pas avec la resolution.
  result.k1 = this->calibration_.k1;
  result.k2 = this->calibration_.k2;
  result.p1 = this->calibration_.p1;
  result.p2 = this->calibration_.p2;
  result.k3 = this->calibration_.k3;
  result.reference_width_px = frame_width;
  result.reference_height_px = frame_height;
  return result;
}

bool GeometryMeasurementEngine::derive_calibration_from_known_distance(
    const TargetObservation &observation,
    uint16_t frame_width,
    uint16_t frame_height,
    float known_distance_mm,
    CameraCalibration &result) const {
  if (!observation.valid || frame_width == 0 || frame_height == 0 ||
      !std::isfinite(known_distance_mm) || known_distance_mm <= 0.0f ||
      this->target_size_mm_ <= 0.0f) {
    return false;
  }

  ImagePoint points[4];
  canonical_corners(observation, points);

  bool use_v4_edges =
      observation.subpixel_refined &&
      observation.subpixel_width_px >= 4.0f &&
      observation.subpixel_height_px >= 4.0f;

  // Si une calibration avec distorsion existe deja, utiliser ses coefficients
  // comme correction provisoire avant de reestimer fx/fy. Les dimensions V4
  // sont mesurees dans l'image brute : tant que la distorsion n'est pas nulle,
  // conserver le chemin par coins corriges.
  if (this->has_calibration()) {
    const CameraCalibration current =
        this->effective_calibration(frame_width, frame_height);
    undistort_points(points, current);
    use_v4_edges = use_v4_edges && distortion_is_zero(current);
  }

  float width_px = 0.5f * (point_distance(points[0], points[1]) +
                           point_distance(points[3], points[2]));
  float height_px = 0.5f * (point_distance(points[0], points[3]) +
                            point_distance(points[1], points[2]));

  if (use_v4_edges) {
    width_px = observation.subpixel_width_px;
    height_px = observation.subpixel_height_px;
  }

  if (!std::isfinite(width_px) || !std::isfinite(height_px) ||
      width_px < 4.0f || height_px < 4.0f) {
    return false;
  }

  CameraCalibration calibration;
  calibration.cx_px = (static_cast<float>(frame_width) - 1.0f) * 0.5f;
  calibration.cy_px = (static_cast<float>(frame_height) - 1.0f) * 0.5f;
  calibration.reference_width_px = frame_width;
  calibration.reference_height_px = frame_height;

  // La distance saisie par l'utilisateur est la distance physique entre le
  // centre optique de la camera et le centre de la cible (range 3D), et non
  // la profondeur Z. Hors axe, utiliser directement range comme Z surestime
  // fx/fy et introduit un biais systematique. Resoudre iterativement :
  //
  //   range = Z * sqrt(1 + x_n^2 + y_n^2)
  //   fx = largeur_px * Z / taille_cible
  //   fy = hauteur_px * Z / taille_cible
  //
  // avec x_n/y_n calcules au centre projectif de la cible.
  const ImagePoint center = quadrilateral_center(points);
  float fx = width_px * known_distance_mm / this->target_size_mm_;
  float fy = height_px * known_distance_mm / this->target_size_mm_;

  for (uint8_t iteration = 0; iteration < 8; ++iteration) {
    if (!std::isfinite(fx) || !std::isfinite(fy) ||
        fx <= 0.0f || fy <= 0.0f) {
      return false;
    }

    const float normalized_x = (center.x - calibration.cx_px) / fx;
    const float normalized_y = (center.y - calibration.cy_px) / fy;
    const float range_factor =
        std::sqrt(1.0f + normalized_x * normalized_x +
                  normalized_y * normalized_y);
    if (!std::isfinite(range_factor) || range_factor < 1.0f) {
      return false;
    }

    const float z_mm = known_distance_mm / range_factor;
    const float next_fx = width_px * z_mm / this->target_size_mm_;
    const float next_fy = height_px * z_mm / this->target_size_mm_;

    if (std::fabs(next_fx - fx) < 0.0001f &&
        std::fabs(next_fy - fy) < 0.0001f) {
      fx = next_fx;
      fy = next_fy;
      break;
    }
    fx = next_fx;
    fy = next_fy;
  }

  calibration.fx_px = fx;
  calibration.fy_px = fy;

  // La calibration distance ne sait pas estimer la distorsion a elle seule.
  // Conserver les coefficients deja connus ; sinon ils restent nuls.
  calibration.k1 = this->calibration_.k1;
  calibration.k2 = this->calibration_.k2;
  calibration.p1 = this->calibration_.p1;
  calibration.p2 = this->calibration_.p2;
  calibration.k3 = this->calibration_.k3;

  if (!std::isfinite(calibration.fx_px) || !std::isfinite(calibration.fy_px) ||
      calibration.fx_px <= 0.0f || calibration.fy_px <= 0.0f) {
    return false;
  }

  result = calibration;
  return true;
}

bool GeometryMeasurementEngine::calibrate_from_known_distance(
    const TargetObservation &observation,
    uint16_t frame_width,
    uint16_t frame_height,
    float known_distance_mm) {
  CameraCalibration calibration;
  if (!this->derive_calibration_from_known_distance(
          observation, frame_width, frame_height, known_distance_mm, calibration)) {
    return false;
  }
  this->calibration_ = calibration;
  return true;
}

bool GeometryMeasurementEngine::set_distortion_coefficients(
    float k1, float k2, float p1, float p2, float k3) {
  if (!std::isfinite(k1) || !std::isfinite(k2) ||
      !std::isfinite(p1) || !std::isfinite(p2) || !std::isfinite(k3)) {
    return false;
  }

  // Bornes volontairement larges mais finies pour eviter une configuration
  // manifestement corrompue.
  if (std::fabs(k1) > 5.0f || std::fabs(k2) > 5.0f ||
      std::fabs(k3) > 5.0f || std::fabs(p1) > 1.0f ||
      std::fabs(p2) > 1.0f) {
    return false;
  }

  this->calibration_.k1 = k1;
  this->calibration_.k2 = k2;
  this->calibration_.p1 = p1;
  this->calibration_.p2 = p2;
  this->calibration_.k3 = k3;
  return true;
}

void GeometryMeasurementEngine::clear_distortion() {
  this->calibration_.k1 = 0.0f;
  this->calibration_.k2 = 0.0f;
  this->calibration_.p1 = 0.0f;
  this->calibration_.p2 = 0.0f;
  this->calibration_.k3 = 0.0f;
}

GeometryMeasurement GeometryMeasurementEngine::compute(const TargetObservation &observation,
                                                       uint16_t frame_width,
                                                       uint16_t frame_height,
                                                       uint32_t timestamp_ms) const {
  GeometryMeasurement result;
  result.timestamp_ms = timestamp_ms;
  result.quality = observation.quality;
  result.calibrated = this->has_calibration();

  if (!observation.valid || !result.calibrated || this->target_size_mm_ <= 0.0f) {
    return result;
  }

  const CameraCalibration calibration = this->effective_calibration(frame_width, frame_height);
  if (calibration.fx_px <= 0.0f || calibration.fy_px <= 0.0f) {
    return result;
  }

  ImagePoint points[4];
  canonical_corners(observation, points);
  undistort_points(points, calibration);

  float width_px = 0.5f * (point_distance(points[0], points[1]) +
                           point_distance(points[3], points[2]));
  float height_px = 0.5f * (point_distance(points[0], points[3]) +
                            point_distance(points[1], points[2]));

  result.corner_width_px = width_px;
  result.corner_height_px = height_px;

  // Distance V5 : quand le raffinement des bords a reussi et qu'aucune
  // correction de distorsion n'est necessaire, utiliser directement la
  // separation des paires de droites opposees. On evite ainsi de convertir
  // quatre droites stables en quatre intersections plus bruitees, puis de
  // recalculer les dimensions a partir de ces coins.
  const bool use_v4_edges =
      observation.subpixel_refined &&
      observation.subpixel_width_px >= 4.0f &&
      observation.subpixel_height_px >= 4.0f &&
      distortion_is_zero(calibration);
  if (use_v4_edges) {
    width_px = observation.subpixel_width_px;
    height_px = observation.subpixel_height_px;
  }

  if (!std::isfinite(width_px) || !std::isfinite(height_px) ||
      width_px < 4.0f || height_px < 4.0f) {
    return result;
  }

  result.edge_v4_used = use_v4_edges;
  result.edge_v5_used =
      observation.subpixel_refined &&
      observation.subpixel_v5_width_px >= 4.0f &&
      observation.subpixel_v5_height_px >= 4.0f;
  result.edge_v6_used =
      observation.subpixel_refined &&
      observation.subpixel_v6_width_px >= 4.0f &&
      observation.subpixel_v6_height_px >= 4.0f;

  // Valeur officielle V6.1 : dimensions V5 directes + incertitude enrichie
  // par le desaccord avec V6 local. Les diagnostics ci-dessous sont purement
  // paralleles et ne pilotent pas la sortie officielle.
  result.apparent_width_px = width_px;
  result.apparent_height_px = height_px;
  result.apparent_width_sigma_px =
      use_v4_edges ? observation.subpixel_width_sigma_px : 0.0f;
  result.apparent_height_sigma_px =
      use_v4_edges ? observation.subpixel_height_sigma_px : 0.0f;

  result.v5_width_px =
      result.edge_v5_used ? observation.subpixel_v5_width_px : 0.0f;
  result.v5_height_px =
      result.edge_v5_used ? observation.subpixel_v5_height_px : 0.0f;
  result.v6_width_px =
      result.edge_v6_used ? observation.subpixel_v6_width_px : 0.0f;
  result.v6_height_px =
      result.edge_v6_used ? observation.subpixel_v6_height_px : 0.0f;
  result.v61_width_px = use_v4_edges ? observation.subpixel_width_px : width_px;
  result.v61_height_px = use_v4_edges ? observation.subpixel_height_px : height_px;

  if (result.edge_v5_used && result.edge_v6_used) {
    result.v5_v6_width_delta_px =
        result.v6_width_px - result.v5_width_px;
    result.v5_v6_height_delta_px =
        result.v6_height_px - result.v5_height_px;
  }

  result.edge_top_rms_px = observation.subpixel_top_rms_px;
  result.edge_right_rms_px = observation.subpixel_right_rms_px;
  result.edge_bottom_rms_px = observation.subpixel_bottom_rms_px;
  result.edge_left_rms_px = observation.subpixel_left_rms_px;
  result.edge_top_gradient = observation.subpixel_top_gradient;
  result.edge_right_gradient = observation.subpixel_right_gradient;
  result.edge_bottom_gradient = observation.subpixel_bottom_gradient;
  result.edge_left_gradient = observation.subpixel_left_gradient;

  // Distance V5 : convertir l'incertitude en pixels de chaque paire de droites
  // en incertitude attendue sur Z. Cela donne un poids physique directement
  // comparable entre largeur et hauteur.
  result.z_from_width_mm = calibration.fx_px * this->target_size_mm_ / width_px;
  result.z_from_height_mm = calibration.fy_px * this->target_size_mm_ / height_px;
  if (!std::isfinite(result.z_from_width_mm) || !std::isfinite(result.z_from_height_mm) ||
      result.z_from_width_mm <= 0.0f || result.z_from_height_mm <= 0.0f) {
    return result;
  }

  float width_weight = 1.0f;
  float height_weight = 1.0f;
  if (use_v4_edges &&
      result.apparent_width_sigma_px > 0.0f &&
      result.apparent_height_sigma_px > 0.0f) {
    const float width_sigma_px =
        std::max(0.015f, result.apparent_width_sigma_px);
    const float height_sigma_px =
        std::max(0.015f, result.apparent_height_sigma_px);

    const float width_sigma_z =
        std::max(0.05f,
                 result.z_from_width_mm * width_sigma_px / width_px);
    const float height_sigma_z =
        std::max(0.05f,
                 result.z_from_height_mm * height_sigma_px / height_px);

    width_weight = 1.0f / (width_sigma_z * width_sigma_z);
    height_weight = 1.0f / (height_sigma_z * height_sigma_z);

    // A incertitude equivalente, un bord avec davantage de contraste est un
    // peu plus fiable. Le facteur est volontairement borne pour ne jamais
    // remplacer l'information geometrique par le seul contraste.
    if (observation.subpixel_width_gradient > 0.0f &&
        observation.subpixel_height_gradient > 0.0f) {
      const float gradient_sum =
          observation.subpixel_width_gradient +
          observation.subpixel_height_gradient;
      const float width_gradient_factor =
          std::max(0.75f, std::min(1.25f,
              2.0f * observation.subpixel_width_gradient / gradient_sum));
      const float height_gradient_factor =
          std::max(0.75f, std::min(1.25f,
              2.0f * observation.subpixel_height_gradient / gradient_sum));
      width_weight *= width_gradient_factor;
      height_weight *= height_gradient_factor;
    }
  }

  const float weight_sum = width_weight + height_weight;
  result.width_distance_weight = width_weight / weight_sum;
  result.height_distance_weight = height_weight / weight_sum;

  result.z_mm = fuse_size_distance(
      result.z_from_width_mm, result.z_from_height_mm,
      width_weight, height_weight);
  if (!std::isfinite(result.z_mm) || result.z_mm <= 0.0f) {
    return result;
  }

  auto diagnostic_z = [&](float diagnostic_width_px,
                          float diagnostic_height_px,
                          float width_sigma_px,
                          float height_sigma_px) -> float {
    if (!std::isfinite(diagnostic_width_px) ||
        !std::isfinite(diagnostic_height_px) ||
        diagnostic_width_px < 4.0f || diagnostic_height_px < 4.0f) {
      return 0.0f;
    }

    const float z_width =
        calibration.fx_px * this->target_size_mm_ / diagnostic_width_px;
    const float z_height =
        calibration.fy_px * this->target_size_mm_ / diagnostic_height_px;
    if (!std::isfinite(z_width) || !std::isfinite(z_height) ||
        z_width <= 0.0f || z_height <= 0.0f) {
      return 0.0f;
    }

    float diagnostic_width_weight = 1.0f;
    float diagnostic_height_weight = 1.0f;
    if (width_sigma_px > 0.0f && height_sigma_px > 0.0f) {
      const float sigma_w = std::max(0.015f, width_sigma_px);
      const float sigma_h = std::max(0.015f, height_sigma_px);
      const float sigma_z_w =
          std::max(0.05f, z_width * sigma_w / diagnostic_width_px);
      const float sigma_z_h =
          std::max(0.05f, z_height * sigma_h / diagnostic_height_px);
      diagnostic_width_weight = 1.0f / (sigma_z_w * sigma_z_w);
      diagnostic_height_weight = 1.0f / (sigma_z_h * sigma_z_h);

      if (observation.subpixel_width_gradient > 0.0f &&
          observation.subpixel_height_gradient > 0.0f) {
        const float gradient_sum =
            observation.subpixel_width_gradient +
            observation.subpixel_height_gradient;
        const float width_gradient_factor =
            std::max(0.75f, std::min(
                1.25f,
                2.0f * observation.subpixel_width_gradient / gradient_sum));
        const float height_gradient_factor =
            std::max(0.75f, std::min(
                1.25f,
                2.0f * observation.subpixel_height_gradient / gradient_sum));
        diagnostic_width_weight *= width_gradient_factor;
        diagnostic_height_weight *= height_gradient_factor;
      }
    }

    return fuse_size_distance(
        z_width, z_height,
        diagnostic_width_weight, diagnostic_height_weight);
  };

  if (result.edge_v5_used) {
    result.v5_z_mm = diagnostic_z(
        result.v5_width_px, result.v5_height_px,
        observation.subpixel_v5_width_sigma_px,
        observation.subpixel_v5_height_sigma_px);
  }
  if (result.edge_v6_used) {
    result.v6_z_mm = diagnostic_z(
        result.v6_width_px, result.v6_height_px,
        observation.subpixel_v6_width_sigma_px,
        observation.subpixel_v6_height_sigma_px);
  }
  result.v61_z_mm = result.z_mm;

  const ImagePoint center = quadrilateral_center(points);
  const float normalized_x = (center.x - calibration.cx_px) / calibration.fx_px;
  const float normalized_y = (center.y - calibration.cy_px) / calibration.fy_px;
  result.x_mm = normalized_x * result.z_mm;
  result.y_mm = normalized_y * result.z_mm;
  result.distance_mm = std::sqrt(result.x_mm * result.x_mm +
                                 result.y_mm * result.y_mm +
                                 result.z_mm * result.z_mm);
  if (!std::isfinite(result.distance_mm) || result.distance_mm <= 0.0f) {
    return result;
  }

  result.bearing_yaw_deg = std::atan2(normalized_x, 1.0f) * RAD_TO_DEG_F;
  result.bearing_pitch_deg = std::atan2(normalized_y, 1.0f) * RAD_TO_DEG_F;
  result.valid = true;

  // La decomposition projective n'est pas autorisee a piloter directement la
  // distance principale. Elle sert a l'orientation du plan et fournit pose_z
  // comme controle independant de coherence avec la distance V5.
  float unit_h[9];
  if (!build_unit_square_homography(points, unit_h)) {
    return result;
  }

  const float inverse_size = 1.0f / this->target_size_mm_;
  const Vec3 h1 = {
      unit_h[0] * inverse_size,
      unit_h[3] * inverse_size,
      unit_h[6] * inverse_size,
  };
  const Vec3 h2 = {
      unit_h[1] * inverse_size,
      unit_h[4] * inverse_size,
      unit_h[7] * inverse_size,
  };
  const Vec3 h3 = {
      0.5f * unit_h[0] + 0.5f * unit_h[1] + unit_h[2],
      0.5f * unit_h[3] + 0.5f * unit_h[4] + unit_h[5],
      0.5f * unit_h[6] + 0.5f * unit_h[7] + unit_h[8],
  };

  Vec3 v1 = inverse_intrinsics_column(h1.x, h1.y, h1.z, calibration);
  Vec3 v2 = inverse_intrinsics_column(h2.x, h2.y, h2.z, calibration);
  Vec3 v3 = inverse_intrinsics_column(h3.x, h3.y, h3.z, calibration);

  const float norm1 = norm(v1);
  const float norm2 = norm(v2);
  const float scale_denominator = norm1 + norm2;
  if (!std::isfinite(scale_denominator) || scale_denominator < MIN_VECTOR_NORM) {
    return result;
  }

  const float pose_scale = 2.0f / scale_denominator;
  Vec3 translation = scale(v3, pose_scale);
  if (translation.z < 0.0f) {
    v1 = scale(v1, -1.0f);
    v2 = scale(v2, -1.0f);
    translation = scale(translation, -1.0f);
  }

  if (!std::isfinite(translation.z) || translation.z <= 0.0f) {
    return result;
  }

  result.pose_z_mm = translation.z;
  result.pose_scale_error_pct =
      std::fabs(result.pose_z_mm - result.z_mm) * 100.0f /
      std::max(result.z_mm, 1.0f);

  Vec3 r1 = v1;
  if (!normalize(r1)) {
    return result;
  }

  Vec3 r2 = subtract(v2, scale(r1, dot(v2, r1)));
  if (!normalize(r2)) {
    return result;
  }

  Vec3 normal = cross(r1, r2);
  if (!normalize(normal)) {
    return result;
  }

  PoseBasis pose_v1_basis{r1, r2, normal};
  if (!normalize_basis(pose_v1_basis)) {
    return result;
  }

  pose_angles_from_basis(
      pose_v1_basis,
      result.pose_v1_yaw_deg,
      result.pose_v1_pitch_deg,
      result.pose_v1_roll_deg);

  result.pose_v1_valid =
      std::isfinite(result.pose_scale_error_pct) &&
      result.pose_scale_error_pct <= MAX_POSE_SCALE_ERROR_PCT;

  // Pose V2 : seule l'orientation est optimisee. La translation provient
  // EXCLUSIVEMENT de V6.1-robust5 deja calculee ci-dessus et n'est jamais
  // reinjectee dans la distance. Le cout s'appuie principalement sur les
  // quatre droites subpixel, les coins ne servant que de faible regularisation.
  const Vec3 fixed_translation = {
      result.x_mm, result.y_mm, result.z_mm};

  PoseBasis pose_v2_basis;
  PoseV2Score pose_v2_score{};
  if (observation.subpixel_refined &&
      distortion_is_zero(calibration) &&
      refine_pose_v2(
          pose_v1_basis, fixed_translation,
          this->target_size_mm_, calibration,
          points, observation,
          pose_v2_basis, pose_v2_score)) {
    result.pose_v2_valid = true;
    result.pose_v2_used = true;
    result.pose_v2_line_rms_px = pose_v2_score.line_rms_px;
    result.pose_v2_corner_rms_px = pose_v2_score.corner_rms_px;

    pose_angles_from_basis(
        pose_v2_basis,
        result.pose_v2_yaw_deg,
        result.pose_v2_pitch_deg,
        result.pose_v2_roll_deg);

    result.yaw_deg = result.pose_v2_yaw_deg;
    result.pitch_deg = result.pose_v2_pitch_deg;
    result.roll_deg = result.pose_v2_roll_deg;
    result.pose_normal_x = pose_v2_basis.normal.x;
    result.pose_normal_y = pose_v2_basis.normal.y;
    result.pose_normal_z = pose_v2_basis.normal.z;
    result.pose_valid = true;
    return result;
  }

  // Repli conservateur : l'ancienne pose reste disponible uniquement si sa
  // coherence d'echelle historique est acceptable. La distance principale
  // reste dans tous les cas celle de V6.1-robust5.
  if (result.pose_v1_valid) {
    result.yaw_deg = result.pose_v1_yaw_deg;
    result.pitch_deg = result.pose_v1_pitch_deg;
    result.roll_deg = result.pose_v1_roll_deg;
    result.pose_normal_x = pose_v1_basis.normal.x;
    result.pose_normal_y = pose_v1_basis.normal.y;
    result.pose_normal_z = pose_v1_basis.normal.z;
    result.pose_valid = true;
  }

  return result;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
