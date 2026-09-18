#include "measurement_manager.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace geometrie_camera_app {

namespace {
constexpr float RAD_TO_DEG_F = 57.29577951308232f;
constexpr uint8_t LOCAL_STABILIZATION_WINDOW = 5;
constexpr float DISTANCE_BLEND_START_RATIO = 0.03f;
constexpr float DISTANCE_BLEND_FULL_RATIO = 0.15f;

float robust_center(const float *values, uint8_t count) {
  if (values == nullptr || count == 0) {
    return 0.0f;
  }

  float sorted[LOCAL_STABILIZATION_WINDOW];
  for (uint8_t i = 0; i < count; ++i) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < count; ++i) {
    const float value = sorted[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }

  if (count == 1) {
    return sorted[0];
  }
  if (count == 2) {
    return 0.5f * (sorted[0] + sorted[1]);
  }
  if (count == 3) {
    return sorted[1];
  }

  // A partir de 4 valeurs, retirer systematiquement les extremes.
  // Avec la fenetre nominale de 5, la valeur publiee est donc la moyenne
  // robuste des trois mesures centrales.
  float sum = 0.0f;
  for (uint8_t i = 1; i + 1 < count; ++i) {
    sum += sorted[i];
  }
  return sum / static_cast<float>(count - 2U);
}

float normalize_half_turn_local(float angle_deg) {
  while (angle_deg >= 90.0f) angle_deg -= 180.0f;
  while (angle_deg < -90.0f) angle_deg += 180.0f;
  return angle_deg;
}

float periodic_delta_180(float a_deg, float b_deg) {
  return normalize_half_turn_local(a_deg - b_deg);
}

float robust_periodic_center_180(const float *values, uint8_t count) {
  if (values == nullptr || count == 0) return 0.0f;

  // Choisir comme ancre l'echantillon qui minimise la somme des distances
  // periodiques. Cela evite qu'un premier point aberrant impose la branche.
  uint8_t best_anchor = 0;
  float best_cost = 1.0e30f;
  for (uint8_t candidate = 0; candidate < count; ++candidate) {
    float cost = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
      cost += std::fabs(
          periodic_delta_180(values[i], values[candidate]));
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_anchor = candidate;
    }
  }

  const float anchor = values[best_anchor];
  float unwrapped[LOCAL_STABILIZATION_WINDOW];
  for (uint8_t i = 0; i < count; ++i) {
    unwrapped[i] =
        anchor + periodic_delta_180(values[i], anchor);
  }

  return normalize_half_turn_local(
      robust_center(unwrapped, count));
}

float standard_deviation(const float *values, uint8_t count) {
  if (values == nullptr || count < 2) {
    return 0.0f;
  }
  double mean = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    mean += values[i];
  }
  mean /= count;

  double variance = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    const double delta = values[i] - mean;
    variance += delta * delta;
  }
  variance /= count;
  return static_cast<float>(std::sqrt(variance));
}

float fuse_stabilized_axes(float z_width, float z_height,
                           float width_weight, float height_weight) {
  const float lower = std::min(z_width, z_height);
  const float upper = std::max(z_width, z_height);
  if (!std::isfinite(z_width) || !std::isfinite(z_height) ||
      z_width <= 0.0f || z_height <= 0.0f || lower <= 0.0f) {
    return lower;
  }

  float w_width =
      std::isfinite(width_weight) && width_weight > 0.0f ? width_weight : 0.5f;
  float w_height =
      std::isfinite(height_weight) && height_weight > 0.0f ? height_weight : 0.5f;
  if (w_width > w_height * 16.0f) w_width = w_height * 16.0f;
  if (w_height > w_width * 16.0f) w_height = w_width * 16.0f;

  const float weighted =
      (w_width * z_width + w_height * z_height) / (w_width + w_height);
  const float disagreement = (upper - lower) / lower;
  if (disagreement <= DISTANCE_BLEND_START_RATIO) return weighted;
  if (disagreement >= DISTANCE_BLEND_FULL_RATIO) return lower;

  const float blend =
      (disagreement - DISTANCE_BLEND_START_RATIO) /
      (DISTANCE_BLEND_FULL_RATIO - DISTANCE_BLEND_START_RATIO);
  return weighted + (lower - weighted) * blend;
}

float span(const float *values, uint8_t count) {
  if (values == nullptr || count == 0) {
    return 0.0f;
  }
  float minimum = values[0];
  float maximum = values[0];
  for (uint8_t i = 1; i < count; ++i) {
    minimum = std::min(minimum, values[i]);
    maximum = std::max(maximum, values[i]);
  }
  return maximum - minimum;
}
}  // namespace

MeasurementManager::MeasurementManager()
    : measurement_engine_(),
      raw_measurement_(),
      last_measurement_(),
      stabilization_samples_{},
      stabilization_count_(0),
      stabilization_next_index_(0),
      last_measurement_stabilized_(false),
      distance_stddev_mm_(0.0f),
      distance_span_mm_(0.0f),
      valid_measurement_count_(0) {}

void MeasurementManager::setup() {
  this->reset();
}

void MeasurementManager::reset() {
  this->raw_measurement_ = GeometryMeasurement();
  this->last_measurement_ = GeometryMeasurement();
  this->valid_measurement_count_ = 0;
  this->reset_stabilization();
}

void MeasurementManager::reset_stabilization() {
  for (auto &sample : this->stabilization_samples_) {
    sample = GeometryMeasurement();
  }
  this->stabilization_count_ = 0;
  this->stabilization_next_index_ = 0;
  this->last_measurement_stabilized_ = false;
  this->distance_stddev_mm_ = 0.0f;
  this->distance_span_mm_ = 0.0f;
}

bool MeasurementManager::process(const TargetObservation &observation,
                                 uint16_t frame_width, uint16_t frame_height,
                                 uint32_t timestamp_ms,
                                 bool stabilize) {
  this->raw_measurement_ = this->measurement_engine_.compute(
      observation, frame_width, frame_height, timestamp_ms);

  if (!this->raw_measurement_.valid) {
    this->last_measurement_ = this->raw_measurement_;
    this->last_measurement_stabilized_ = false;
    return false;
  }

  this->valid_measurement_count_++;

  if (!stabilize) {
    this->reset_stabilization();
    this->last_measurement_ = this->raw_measurement_;
    return true;
  }

  this->append_stabilization_sample_(this->raw_measurement_);
  this->compute_stabilized_measurement_();
  return true;
}

void MeasurementManager::append_stabilization_sample_(
    const GeometryMeasurement &measurement) {
  this->stabilization_samples_[this->stabilization_next_index_] = measurement;
  this->stabilization_next_index_ =
      static_cast<uint8_t>((this->stabilization_next_index_ + 1U) %
                           STABILIZATION_WINDOW);
  if (this->stabilization_count_ < STABILIZATION_WINDOW) {
    this->stabilization_count_++;
  }
}

void MeasurementManager::compute_stabilized_measurement_() {
  if (this->stabilization_count_ == 0) {
    this->last_measurement_ = this->raw_measurement_;
    this->last_measurement_stabilized_ = false;
    return;
  }

  float distance_values[STABILIZATION_WINDOW];
  float x_values[STABILIZATION_WINDOW];
  float y_values[STABILIZATION_WINDOW];
  float z_values[STABILIZATION_WINDOW];
  float z_width_values[STABILIZATION_WINDOW];
  float z_height_values[STABILIZATION_WINDOW];
  float width_weight_values[STABILIZATION_WINDOW];
  float height_weight_values[STABILIZATION_WINDOW];
  float quality_values[STABILIZATION_WINDOW];

  float yaw_values[STABILIZATION_WINDOW];
  float pitch_values[STABILIZATION_WINDOW];
  float roll_values[STABILIZATION_WINDOW];
  float normal_x_values[STABILIZATION_WINDOW];
  float normal_y_values[STABILIZATION_WINDOW];
  float normal_z_values[STABILIZATION_WINDOW];
  float pose_z_values[STABILIZATION_WINDOW];
  float pose_error_values[STABILIZATION_WINDOW];
  uint8_t pose_count = 0;
  uint8_t pose_method_values[STABILIZATION_WINDOW] = {0};
  uint8_t pose_v3_count = 0;
  uint8_t pose_v2_count = 0;
  uint8_t pose_v1_count = 0;

  for (uint8_t i = 0; i < this->stabilization_count_; ++i) {
    const GeometryMeasurement &sample = this->stabilization_samples_[i];
    distance_values[i] = sample.distance_mm;
    x_values[i] = sample.x_mm;
    y_values[i] = sample.y_mm;
    z_values[i] = sample.z_mm;
    z_width_values[i] = sample.z_from_width_mm;
    z_height_values[i] = sample.z_from_height_mm;
    width_weight_values[i] = sample.width_distance_weight;
    height_weight_values[i] = sample.height_distance_weight;
    quality_values[i] = sample.quality;

    if (sample.pose_valid) {
      yaw_values[pose_count] = sample.yaw_deg;
      pitch_values[pose_count] = sample.pitch_deg;
      roll_values[pose_count] = sample.roll_deg;
      normal_x_values[pose_count] = sample.pose_normal_x;
      normal_y_values[pose_count] = sample.pose_normal_y;
      normal_z_values[pose_count] = sample.pose_normal_z;
      pose_z_values[pose_count] = sample.pose_z_mm;
      pose_error_values[pose_count] = sample.pose_scale_error_pct;
      if (sample.pose_v3_used) {
        pose_method_values[pose_count] = 3;
        pose_v3_count++;
      } else if (sample.pose_v2_used) {
        pose_method_values[pose_count] = 2;
        pose_v2_count++;
      } else if (sample.pose_v1_valid) {
        pose_method_values[pose_count] = 1;
        pose_v1_count++;
      }
      pose_count++;
    }
  }

  this->distance_stddev_mm_ =
      standard_deviation(distance_values, this->stabilization_count_);
  this->distance_span_mm_ =
      span(distance_values, this->stabilization_count_);

  // Les deux premiers points servent uniquement a amorcer la fenetre :
  // publier la mesure brute evite de faire croire a une stabilisation qui
  // n'existe pas encore. A partir de 3 points, la mediane/moyenne tronquee
  // devient active.
  if (this->stabilization_count_ < 3) {
    this->last_measurement_ = this->raw_measurement_;
    this->last_measurement_stabilized_ = false;
    return;
  }

  GeometryMeasurement result = this->raw_measurement_;
  result.x_mm = robust_center(x_values, this->stabilization_count_);
  result.y_mm = robust_center(y_values, this->stabilization_count_);
  result.z_from_width_mm =
      robust_center(z_width_values, this->stabilization_count_);
  result.z_from_height_mm =
      robust_center(z_height_values, this->stabilization_count_);

  float width_weight =
      robust_center(width_weight_values, this->stabilization_count_);
  float height_weight =
      robust_center(height_weight_values, this->stabilization_count_);
  const float weight_sum = width_weight + height_weight;
  if (std::isfinite(weight_sum) && weight_sum > 0.0f) {
    result.width_distance_weight = width_weight / weight_sum;
    result.height_distance_weight = height_weight / weight_sum;
  } else {
    result.width_distance_weight = 0.5f;
    result.height_distance_weight = 0.5f;
  }

  result.z_mm = fuse_stabilized_axes(
      result.z_from_width_mm, result.z_from_height_mm,
      result.width_distance_weight, result.height_distance_weight);
  result.distance_mm = std::sqrt(result.x_mm * result.x_mm +
                                 result.y_mm * result.y_mm +
                                 result.z_mm * result.z_mm);
  result.quality = robust_center(quality_values, this->stabilization_count_);

  if (result.z_mm > 0.0f) {
    result.bearing_yaw_deg =
        std::atan2(result.x_mm, result.z_mm) * RAD_TO_DEG_F;
    result.bearing_pitch_deg =
        std::atan2(result.y_mm, result.z_mm) * RAD_TO_DEG_F;
  }

  const uint8_t required_pose =
      static_cast<uint8_t>((this->stabilization_count_ + 1U) / 2U);
  result.pose_valid = pose_count >= required_pose;
  if (result.pose_valid) {
    uint8_t selected_method = 0;
    if (pose_v3_count >= required_pose) {
      selected_method = 3;
    } else if (pose_v2_count >= required_pose) {
      selected_method = 2;
    } else if (pose_v1_count >= required_pose) {
      selected_method = 1;
    }

    result.pose_v3_used = selected_method == 3;
    result.pose_v3_valid = pose_v3_count > 0;
    result.pose_v2_used = selected_method == 2;
    result.pose_v2_valid = pose_v2_count > 0 || result.pose_v2_valid;
    result.pose_v1_valid = pose_v1_count > 0 || result.pose_v1_valid;

    // Ne jamais melanger les normales de methodes differentes. Un repli V2
    // ponctuel ne doit pas degrader la moyenne V3, et inversement.
    if (selected_method != 0) {
      uint8_t filtered_count = 0;
      for (uint8_t i = 0; i < pose_count; ++i) {
        if (pose_method_values[i] != selected_method) continue;
        yaw_values[filtered_count] = yaw_values[i];
        pitch_values[filtered_count] = pitch_values[i];
        roll_values[filtered_count] = roll_values[i];
        normal_x_values[filtered_count] = normal_x_values[i];
        normal_y_values[filtered_count] = normal_y_values[i];
        normal_z_values[filtered_count] = normal_z_values[i];
        pose_z_values[filtered_count] = pose_z_values[i];
        pose_error_values[filtered_count] = pose_error_values[i];
        filtered_count++;
      }
      if (filtered_count >= required_pose) {
        pose_count = filtered_count;
      }
    }

    float normal_x = robust_center(normal_x_values, pose_count);
    float normal_y = robust_center(normal_y_values, pose_count);
    float normal_z = robust_center(normal_z_values, pose_count);
    const float normal_norm = std::sqrt(
        normal_x * normal_x +
        normal_y * normal_y +
        normal_z * normal_z);

    if (std::isfinite(normal_norm) && normal_norm > 1.0e-6f) {
      normal_x /= normal_norm;
      normal_y /= normal_norm;
      normal_z /= normal_norm;
      result.pose_normal_x = normal_x;
      result.pose_normal_y = normal_y;
      result.pose_normal_z = normal_z;
      result.yaw_deg =
          std::atan2(normal_x, normal_z) * RAD_TO_DEG_F;
      result.pitch_deg =
          std::atan2(
              -normal_y,
              std::sqrt(normal_x * normal_x +
                        normal_z * normal_z)) *
          RAD_TO_DEG_F;
    } else {
      result.yaw_deg = robust_center(yaw_values, pose_count);
      result.pitch_deg = robust_center(pitch_values, pose_count);
    }

    // Le roll est defini modulo 180 degres. Une moyenne lineaire ferait de
    // +89 et -89 deux valeurs opposees alors qu'elles sont voisines.
    result.roll_deg =
        robust_periodic_center_180(roll_values, pose_count);
    result.pose_z_mm = robust_center(pose_z_values, pose_count);
    result.pose_scale_error_pct =
        robust_center(pose_error_values, pose_count);
  }

  result.timestamp_ms = this->raw_measurement_.timestamp_ms;
  this->last_measurement_ = result;
  this->last_measurement_stabilized_ = true;
}

uint32_t MeasurementManager::valid_measurement_count() const {
  return this->valid_measurement_count_;
}

const GeometryMeasurement &MeasurementManager::last_measurement() const {
  return this->last_measurement_;
}

const GeometryMeasurement &MeasurementManager::raw_measurement() const {
  return this->raw_measurement_;
}

bool MeasurementManager::last_measurement_stabilized() const {
  return this->last_measurement_stabilized_;
}

uint8_t MeasurementManager::stabilization_sample_count() const {
  return this->stabilization_count_;
}

uint8_t MeasurementManager::stabilization_window_size() const {
  return STABILIZATION_WINDOW;
}

float MeasurementManager::distance_stddev_mm() const {
  return this->distance_stddev_mm_;
}

float MeasurementManager::distance_span_mm() const {
  return this->distance_span_mm_;
}

GeometryMeasurementEngine &MeasurementManager::measurement_engine() {
  return this->measurement_engine_;
}

const GeometryMeasurementEngine &MeasurementManager::measurement_engine() const {
  return this->measurement_engine_;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
