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

float fuse_size_distance(float z_from_width, float z_from_height) {
  const float lower = std::min(z_from_width, z_from_height);
  const float upper = std::max(z_from_width, z_from_height);
  const float sum = z_from_width + z_from_height;
  if (!std::isfinite(sum) || sum <= 0.0f || lower <= 0.0f) {
    return lower;
  }

  // Moyenne harmonique = moyenne des deux tailles apparentes normalisees.
  // Elle est continue et beaucoup moins sensible au basculement largeur/hauteur
  // que le min() historique lorsque les deux estimations sont proches.
  const float harmonic = 2.0f * z_from_width * z_from_height / sum;
  const float disagreement = (upper - lower) / lower;

  if (disagreement <= DISTANCE_BLEND_START_RATIO) {
    return harmonic;
  }
  if (disagreement >= DISTANCE_BLEND_FULL_RATIO) {
    return lower;
  }

  // Quand les deux axes divergent progressivement (inclinaison de la cible),
  // revenir sans discontinuite vers la dimension la moins raccourcie.
  const float blend = (disagreement - DISTANCE_BLEND_START_RATIO) /
                      (DISTANCE_BLEND_FULL_RATIO - DISTANCE_BLEND_START_RATIO);
  return harmonic + (lower - harmonic) * blend;
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

  // Si une calibration avec distorsion existe deja, utiliser ses coefficients
  // comme correction provisoire avant de reestimer fx/fy. Sans coefficients,
  // cette etape est strictement neutre.
  if (this->has_calibration()) {
    const CameraCalibration current = this->effective_calibration(frame_width, frame_height);
    undistort_points(points, current);
  }

  const float width_px = 0.5f * (point_distance(points[0], points[1]) +
                                 point_distance(points[3], points[2]));
  const float height_px = 0.5f * (point_distance(points[0], points[3]) +
                                  point_distance(points[1], points[2]));
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
  const float width_px = 0.5f * (point_distance(points[0], points[1]) +
                                 point_distance(points[3], points[2]));
  const float height_px = 0.5f * (point_distance(points[0], points[3]) +
                                  point_distance(points[1], points[2]));
  if (!std::isfinite(width_px) || !std::isfinite(height_px) || width_px < 4.0f || height_px < 4.0f) {
    return result;
  }

  // Distance V3 : exploiter les quatre cotes raffines sans basculement brutal
  // entre largeur et hauteur. Quand les deux axes sont coherents, leurs tailles
  // apparentes sont fusionnees. En cas d'inclinaison marquee, la fusion revient
  // progressivement vers l'estimation la moins affectee par le raccourcissement.
  result.z_from_width_mm = calibration.fx_px * this->target_size_mm_ / width_px;
  result.z_from_height_mm = calibration.fy_px * this->target_size_mm_ / height_px;
  if (!std::isfinite(result.z_from_width_mm) || !std::isfinite(result.z_from_height_mm) ||
      result.z_from_width_mm <= 0.0f || result.z_from_height_mm <= 0.0f) {
    return result;
  }

  result.z_mm = fuse_size_distance(result.z_from_width_mm, result.z_from_height_mm);
  if (!std::isfinite(result.z_mm) || result.z_mm <= 0.0f) {
    return result;
  }

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
  // comme controle independant de coherence avec la distance V3.
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
      std::fabs(result.pose_z_mm - result.z_mm) * 100.0f / std::max(result.z_mm, 1.0f);
  if (!std::isfinite(result.pose_scale_error_pct) ||
      result.pose_scale_error_pct > MAX_POSE_SCALE_ERROR_PCT) {
    return result;
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

  result.roll_deg = normalize_half_turn(
      std::atan2(dot(r1, reference_down), dot(r1, reference_right)) * RAD_TO_DEG_F);
  result.pose_valid = true;
  return result;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
