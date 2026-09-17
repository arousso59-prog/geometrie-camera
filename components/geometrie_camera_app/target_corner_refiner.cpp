#include "target_corner_refiner.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "target_corner_refiner";

constexpr uint16_t MIN_SEARCH_RADIUS_PX = 3;
// V5.6 : les coins issus de la localisation basse resolution peuvent etre
// davantage decales lorsque la cible est vue en perspective, surtout en
// PRECISE ou elle occupe beaucoup plus de pixels. On autorise donc une zone
// de recherche plus large tout en gardant la penalite de deplacement.
constexpr uint16_t MAX_SEARCH_RADIUS_PX = 24;
constexpr float SEARCH_RADIUS_RATIO = 0.25f;
constexpr float MIN_EDGE_RATIO = 0.30f;
constexpr float MIN_AREA_RATIO = 0.20f;
constexpr float MIN_TOTAL_SCORE_GAIN = 1.0f;
constexpr float DISPLACEMENT_PENALTY = 0.45f;

float distance_between(const TargetPoint &a, const TargetPoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool normalize(float x, float y, float &nx, float &ny) {
  const float length = std::sqrt(x * x + y * y);
  if (length < 0.001f) {
    nx = 0.0f;
    ny = 0.0f;
    return false;
  }
  nx = x / length;
  ny = y / length;
  return true;
}
}

TargetCornerRefiner::TargetCornerRefiner() {}

bool TargetCornerRefiner::refine(const GrayFrameView &frame, const TargetCandidate &input,
                                 TargetCandidate &output) const {
  output = input;

  if (frame.data == nullptr || frame.width == 0 || frame.height == 0 || frame.stride < frame.width ||
      input.width < 8.0f || input.height < 8.0f) {
    return false;
  }

  const float minimum_side = std::min(input.width, input.height);
  const uint16_t radius = static_cast<uint16_t>(std::max<float>(
      MIN_SEARCH_RADIUS_PX,
      std::min<float>(MAX_SEARCH_RADIUS_PX, std::round(minimum_side * SEARCH_RADIUS_RATIO))));

  TargetCandidate refined = input;
  TargetPoint refined_top_left;
  TargetPoint refined_top_right;
  TargetPoint refined_bottom_right;
  TargetPoint refined_bottom_left;

  const bool tl_ok = this->refine_corner_(frame, input.top_left, input.top_right, input.bottom_left,
                                          radius, refined_top_left);
  const bool tr_ok = this->refine_corner_(frame, input.top_right, input.top_left, input.bottom_right,
                                          radius, refined_top_right);
  const bool br_ok = this->refine_corner_(frame, input.bottom_right, input.bottom_left, input.top_right,
                                          radius, refined_bottom_right);
  const bool bl_ok = this->refine_corner_(frame, input.bottom_left, input.bottom_right, input.top_left,
                                          radius, refined_bottom_left);

  if (!tl_ok || !tr_ok || !br_ok || !bl_ok) {
    return false;
  }

  const float original_score =
      this->corner_score_(frame, input.top_left, input.top_left, input.top_right, input.bottom_left) +
      this->corner_score_(frame, input.top_right, input.top_right, input.top_left, input.bottom_right) +
      this->corner_score_(frame, input.bottom_right, input.bottom_right, input.bottom_left, input.top_right) +
      this->corner_score_(frame, input.bottom_left, input.bottom_left, input.bottom_right, input.top_left);

  refined.top_left = refined_top_left;
  refined.top_right = refined_top_right;
  refined.bottom_right = refined_bottom_right;
  refined.bottom_left = refined_bottom_left;
  this->update_geometry_(refined);

  if (!this->geometry_valid_(refined)) {
    return false;
  }

  const float refined_score =
      this->corner_score_(frame, refined.top_left, refined.top_left, refined.top_right, refined.bottom_left) +
      this->corner_score_(frame, refined.top_right, refined.top_right, refined.top_left, refined.bottom_right) +
      this->corner_score_(frame, refined.bottom_right, refined.bottom_right, refined.bottom_left, refined.top_right) +
      this->corner_score_(frame, refined.bottom_left, refined.bottom_left, refined.bottom_right, refined.top_left);

  if (refined_score < original_score + MIN_TOTAL_SCORE_GAIN) {
    return false;
  }

  output = refined;
  ESP_LOGD(TAG,
           "V5.6 refined center=(%.1f,%.1f) size=%.1fx%.1f radius=%u corner_score=%.1f->%.1f",
           output.center_x, output.center_y, output.width, output.height,
           static_cast<unsigned>(radius), original_score, refined_score);
  return true;
}

bool TargetCornerRefiner::refine_corner_(const GrayFrameView &frame, const TargetPoint &origin,
                                         const TargetPoint &neighbor_a, const TargetPoint &neighbor_b,
                                         uint16_t radius, TargetPoint &refined) const {
  float best_score = -1000000.0f;
  TargetPoint best = origin;

  for (int dy = -static_cast<int>(radius); dy <= static_cast<int>(radius); ++dy) {
    for (int dx = -static_cast<int>(radius); dx <= static_cast<int>(radius); ++dx) {
      TargetPoint point;
      point.x = origin.x + static_cast<float>(dx);
      point.y = origin.y + static_cast<float>(dy);

      if (point.x < 1.0f || point.y < 1.0f || point.x >= frame.width - 1.0f ||
          point.y >= frame.height - 1.0f) {
        continue;
      }

      const float displacement = std::sqrt(static_cast<float>(dx * dx + dy * dy));
      const float score = this->corner_score_(frame, point, origin, neighbor_a, neighbor_b) -
                          displacement * DISPLACEMENT_PENALTY;
      if (score > best_score) {
        best_score = score;
        best = point;
      }
    }
  }

  if (best_score <= -999999.0f) {
    return false;
  }

  refined = best;
  return true;
}

float TargetCornerRefiner::corner_score_(const GrayFrameView &frame, const TargetPoint &point,
                                         const TargetPoint &origin, const TargetPoint &neighbor_a,
                                         const TargetPoint &neighbor_b) const {
  float ax = 0.0f;
  float ay = 0.0f;
  float bx = 0.0f;
  float by = 0.0f;
  if (!normalize(neighbor_a.x - origin.x, neighbor_a.y - origin.y, ax, ay) ||
      !normalize(neighbor_b.x - origin.x, neighbor_b.y - origin.y, bx, by)) {
    return -1000000.0f;
  }

  const float edge_a = distance_between(origin, neighbor_a);
  const float edge_b = distance_between(origin, neighbor_b);
  const float minimum_edge = std::max(8.0f, std::min(edge_a, edge_b));
  const float along = std::max(1.5f, std::min(4.5f, minimum_edge * 0.11f));
  const float normal = std::max(1.0f, std::min(3.0f, minimum_edge * 0.065f));

  const float a_sample_x = point.x + ax * along;
  const float a_sample_y = point.y + ay * along;
  const float a_inside = static_cast<float>(this->sample_(frame, a_sample_x + bx * normal,
                                                          a_sample_y + by * normal));
  const float a_outside = static_cast<float>(this->sample_(frame, a_sample_x - bx * normal,
                                                           a_sample_y - by * normal));

  const float b_sample_x = point.x + bx * along;
  const float b_sample_y = point.y + by * along;
  const float b_inside = static_cast<float>(this->sample_(frame, b_sample_x + ax * normal,
                                                          b_sample_y + ay * normal));
  const float b_outside = static_cast<float>(this->sample_(frame, b_sample_x - ax * normal,
                                                           b_sample_y - ay * normal));

  float diagonal_x = 0.0f;
  float diagonal_y = 0.0f;
  normalize(ax + bx, ay + by, diagonal_x, diagonal_y);
  const float diagonal_inside = static_cast<float>(this->sample_(frame, point.x + diagonal_x * normal,
                                                                 point.y + diagonal_y * normal));
  const float diagonal_outside = static_cast<float>(this->sample_(frame, point.x - diagonal_x * normal,
                                                                  point.y - diagonal_y * normal));

  const float edge_score = (a_outside - a_inside) + (b_outside - b_inside);
  const float diagonal_score = diagonal_outside - diagonal_inside;
  return edge_score + 0.35f * diagonal_score;
}

uint8_t TargetCornerRefiner::sample_(const GrayFrameView &frame, float x, float y) const {
  const int ix = std::max(0, std::min<int>(frame.width - 1, static_cast<int>(std::lround(x))));
  const int iy = std::max(0, std::min<int>(frame.height - 1, static_cast<int>(std::lround(y))));
  return frame.data[static_cast<size_t>(iy) * frame.stride + ix];
}

bool TargetCornerRefiner::geometry_valid_(const TargetCandidate &candidate) const {
  const float top = distance_between(candidate.top_left, candidate.top_right);
  const float right = distance_between(candidate.top_right, candidate.bottom_right);
  const float bottom = distance_between(candidate.bottom_left, candidate.bottom_right);
  const float left = distance_between(candidate.top_left, candidate.bottom_left);

  const float minimum_edge = std::min(std::min(top, bottom), std::min(left, right));
  const float maximum_edge = std::max(std::max(top, bottom), std::max(left, right));
  if (minimum_edge < 8.0f || maximum_edge <= 0.0f || minimum_edge / maximum_edge < MIN_EDGE_RATIO) {
    return false;
  }

  const float area_twice = std::fabs(
      candidate.top_left.x * candidate.top_right.y - candidate.top_left.y * candidate.top_right.x +
      candidate.top_right.x * candidate.bottom_right.y - candidate.top_right.y * candidate.bottom_right.x +
      candidate.bottom_right.x * candidate.bottom_left.y - candidate.bottom_right.y * candidate.bottom_left.x +
      candidate.bottom_left.x * candidate.top_left.y - candidate.bottom_left.y * candidate.top_left.x);
  const float reference_area = std::max(1.0f, candidate.width * candidate.height);
  return (0.5f * area_twice) / reference_area >= MIN_AREA_RATIO;
}

void TargetCornerRefiner::update_geometry_(TargetCandidate &candidate) const {
  candidate.center_x = (candidate.top_left.x + candidate.top_right.x + candidate.bottom_right.x +
                        candidate.bottom_left.x) * 0.25f;
  candidate.center_y = (candidate.top_left.y + candidate.top_right.y + candidate.bottom_right.y +
                        candidate.bottom_left.y) * 0.25f;
  candidate.width = 0.5f * (distance_between(candidate.top_left, candidate.top_right) +
                            distance_between(candidate.bottom_left, candidate.bottom_right));
  candidate.height = 0.5f * (distance_between(candidate.top_left, candidate.bottom_left) +
                             distance_between(candidate.top_right, candidate.bottom_right));
}

}  // namespace geometrie_camera_app
}  // namespace esphome
