#include "geometry_measurement.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr float RAD_TO_DEG_F = 57.29577951308232f;
constexpr float MIN_VECTOR_NORM = 1.0e-6f;

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
  result.reference_width_px = frame_width;
  result.reference_height_px = frame_height;
  return result;
}

bool GeometryMeasurementEngine::calibrate_from_known_distance(const TargetObservation &observation,
                                                               uint16_t frame_width,
                                                               uint16_t frame_height,
                                                               float known_distance_mm) {
  if (!observation.valid || frame_width == 0 || frame_height == 0 ||
      !std::isfinite(known_distance_mm) || known_distance_mm <= 0.0f ||
      this->target_size_mm_ <= 0.0f) {
    return false;
  }

  ImagePoint points[4];
  canonical_corners(observation, points);
  const float width_px = 0.5f * (point_distance(points[0], points[1]) +
                                 point_distance(points[3], points[2]));
  const float height_px = 0.5f * (point_distance(points[0], points[3]) +
                                  point_distance(points[1], points[2]));
  if (!std::isfinite(width_px) || !std::isfinite(height_px) || width_px < 4.0f || height_px < 4.0f) {
    return false;
  }

  CameraCalibration calibration;
  calibration.fx_px = width_px * known_distance_mm / this->target_size_mm_;
  calibration.fy_px = height_px * known_distance_mm / this->target_size_mm_;
  calibration.cx_px = (static_cast<float>(frame_width) - 1.0f) * 0.5f;
  calibration.cy_px = (static_cast<float>(frame_height) - 1.0f) * 0.5f;
  calibration.reference_width_px = frame_width;
  calibration.reference_height_px = frame_height;

  if (!std::isfinite(calibration.fx_px) || !std::isfinite(calibration.fy_px) ||
      calibration.fx_px <= 0.0f || calibration.fy_px <= 0.0f) {
    return false;
  }

  this->calibration_ = calibration;
  return true;
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

  const float distance_mm = norm(translation);
  if (!std::isfinite(distance_mm) || distance_mm <= 0.0f || translation.z <= 0.0f) {
    return result;
  }

  result.x_mm = translation.x;
  result.y_mm = translation.y;
  result.z_mm = translation.z;
  result.distance_mm = distance_mm;
  result.bearing_yaw_deg = std::atan2(translation.x, translation.z) * RAD_TO_DEG_F;
  result.bearing_pitch_deg = std::atan2(translation.y, translation.z) * RAD_TO_DEG_F;

  result.yaw_deg = std::atan2(normal.x, normal.z) * RAD_TO_DEG_F;
  result.pitch_deg = std::atan2(-normal.y,
                                std::sqrt(normal.x * normal.x + normal.z * normal.z)) * RAD_TO_DEG_F;

  Vec3 reference_right = {normal.z, 0.0f, -normal.x};
  if (!normalize(reference_right)) {
    reference_right = {1.0f, 0.0f, 0.0f};
  }
  Vec3 reference_down = cross(normal, reference_right);
  if (!normalize(reference_down)) {
    return result;
  }

  result.roll_deg = std::atan2(dot(r1, reference_down), dot(r1, reference_right)) * RAD_TO_DEG_F;
  result.valid = true;
  return result;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
