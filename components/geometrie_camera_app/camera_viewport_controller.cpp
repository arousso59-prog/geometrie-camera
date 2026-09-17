#include "camera_viewport_controller.h"

#include <algorithm>
#include <cmath>

#include "camera_resolution_controller.h"
#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "camera_viewport";
constexpr uint16_t SENSOR_MARGIN_X = 32;
constexpr uint16_t SENSOR_MARGIN_Y = 16;
constexpr uint16_t SENSOR_TOTAL_X = 2844;
constexpr uint16_t SENSOR_TOTAL_Y = 1968;

float point_distance(const ImagePoint &a, const ImagePoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}
}

CameraViewportSnapshot::CameraViewportSnapshot()
    : supported(false),
      mode(CameraViewportMode::SEARCH_FULL),
      reference_width(CameraViewportController::REFERENCE_WIDTH),
      reference_height(CameraViewportController::REFERENCE_HEIGHT),
      window_x(0),
      window_y(0),
      window_width(CameraViewportController::REFERENCE_WIDTH),
      window_height(CameraViewportController::REFERENCE_HEIGHT),
      output_width(CameraViewportController::OUTPUT_WIDTH),
      output_height(CameraViewportController::OUTPUT_HEIGHT),
      scale_x(static_cast<float>(CameraViewportController::REFERENCE_WIDTH) /
              CameraViewportController::OUTPUT_WIDTH),
      scale_y(static_cast<float>(CameraViewportController::REFERENCE_HEIGHT) /
              CameraViewportController::OUTPUT_HEIGHT) {}

CameraViewportController::CameraViewportController(CameraResolutionController *resolution_controller)
    : resolution_controller_(resolution_controller), snapshot_() {
  this->snapshot_.supported = this->supports_precise_roi();
}

bool CameraViewportController::supports_precise_roi() const {
  sensor_t *sensor = esp_camera_sensor_get();
  return sensor != nullptr && sensor->id.PID == OV5640_PID && sensor->set_res_raw != nullptr;
}

bool CameraViewportController::apply_search() {
  if (this->resolution_controller_ == nullptr || !this->resolution_controller_->apply("800x600")) {
    ESP_LOGE(TAG, "Impossible d'appliquer le viewport SEARCH 800x600");
    return false;
  }

  this->set_search_snapshot_();
  ESP_LOGI(TAG, "Viewport SEARCH: plein champ 2560x1920 -> 800x600");
  return true;
}

bool CameraViewportController::apply_precise_roi(float center_reference_x, float center_reference_y) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->set_res_raw == nullptr) {
    ESP_LOGE(TAG, "ROI precise indisponible sur ce capteur");
    return false;
  }

  const int32_t max_x = static_cast<int32_t>(REFERENCE_WIDTH - OUTPUT_WIDTH);
  const int32_t max_y = static_cast<int32_t>(REFERENCE_HEIGHT - OUTPUT_HEIGHT);
  int32_t x = static_cast<int32_t>(std::lround(center_reference_x)) - OUTPUT_WIDTH / 2;
  int32_t y = static_cast<int32_t>(std::lround(center_reference_y)) - OUTPUT_HEIGHT / 2;
  x = std::max<int32_t>(0, std::min<int32_t>(x, max_x));
  y = std::max<int32_t>(0, std::min<int32_t>(y, max_y));

  // Le repere 2560x1920 correspond au cadrage 4:3 standard de l'OV5640.
  // Ce cadrage utilise une marge capteur de 32 px horizontalement et 16 px
  // verticalement. On conserve les timings complets pour cette premiere version
  // afin de privilegier la stabilite capteur ; ils pourront etre resserres apres
  // validation sur le materiel reel.
  const int start_x = x;
  const int start_y = y;
  const int end_x = x + OUTPUT_WIDTH + 2 * SENSOR_MARGIN_X - 1;
  const int end_y = y + OUTPUT_HEIGHT + 2 * SENSOR_MARGIN_Y - 1;

  if (sensor->set_res_raw(sensor, start_x, start_y, end_x, end_y,
                          SENSOR_MARGIN_X, SENSOR_MARGIN_Y,
                          SENSOR_TOTAL_X, SENSOR_TOTAL_Y,
                          OUTPUT_WIDTH, OUTPUT_HEIGHT,
                          false, false) != 0) {
    ESP_LOGE(TAG, "Echec set_res_raw ROI x=%d y=%d", static_cast<int>(x), static_cast<int>(y));
    return false;
  }

  // esp_camera_fb_get() renseigne width/height depuis status.framesize.
  // Le flux ROI sort en 800x600 : garder explicitement FRAMESIZE_SVGA pour
  // que les metadonnees du framebuffer restent coherentes avec la sortie brute.
  sensor->status.framesize = FRAMESIZE_SVGA;

  this->snapshot_.supported = true;
  this->snapshot_.mode = CameraViewportMode::PRECISE_ROI;
  this->snapshot_.reference_width = REFERENCE_WIDTH;
  this->snapshot_.reference_height = REFERENCE_HEIGHT;
  this->snapshot_.window_x = static_cast<uint16_t>(x);
  this->snapshot_.window_y = static_cast<uint16_t>(y);
  this->snapshot_.window_width = OUTPUT_WIDTH;
  this->snapshot_.window_height = OUTPUT_HEIGHT;
  this->snapshot_.output_width = OUTPUT_WIDTH;
  this->snapshot_.output_height = OUTPUT_HEIGHT;
  this->snapshot_.scale_x = 1.0f;
  this->snapshot_.scale_y = 1.0f;

  ESP_LOGI(TAG, "Viewport PRECISE: ROI native x=%u y=%u %ux%u -> %ux%u",
           static_cast<unsigned>(this->snapshot_.window_x),
           static_cast<unsigned>(this->snapshot_.window_y),
           static_cast<unsigned>(this->snapshot_.window_width),
           static_cast<unsigned>(this->snapshot_.window_height),
           static_cast<unsigned>(this->snapshot_.output_width),
           static_cast<unsigned>(this->snapshot_.output_height));
  return true;
}

TargetObservation CameraViewportController::to_reference(const TargetObservation &observation) const {
  TargetObservation result = observation;
  result.center_x_px = this->snapshot_.window_x +
      (observation.center_x_px + 0.5f) * this->snapshot_.scale_x - 0.5f;
  result.center_y_px = this->snapshot_.window_y +
      (observation.center_y_px + 0.5f) * this->snapshot_.scale_y - 0.5f;
  result.top_left_px = this->to_reference_point_(observation.top_left_px);
  result.top_right_px = this->to_reference_point_(observation.top_right_px);
  result.bottom_right_px = this->to_reference_point_(observation.bottom_right_px);
  result.bottom_left_px = this->to_reference_point_(observation.bottom_left_px);

  if (observation.valid) {
    result.width_px = 0.5f * (point_distance(result.top_left_px, result.top_right_px) +
                              point_distance(result.bottom_left_px, result.bottom_right_px));
    result.height_px = 0.5f * (point_distance(result.top_left_px, result.bottom_left_px) +
                               point_distance(result.top_right_px, result.bottom_right_px));
  } else {
    result.width_px = observation.width_px * this->snapshot_.scale_x;
    result.height_px = observation.height_px * this->snapshot_.scale_y;
  }
  return result;
}

bool CameraViewportController::target_near_edge(const TargetObservation &observation,
                                                uint8_t central_percent) const {
  if (!observation.valid || this->snapshot_.output_width == 0 || this->snapshot_.output_height == 0) {
    return false;
  }
  central_percent = std::max<uint8_t>(50, std::min<uint8_t>(90, central_percent));
  const float margin_fraction = (100.0f - central_percent) / 200.0f;
  const float margin_x = this->snapshot_.output_width * margin_fraction;
  const float margin_y = this->snapshot_.output_height * margin_fraction;
  return observation.center_x_px < margin_x ||
         observation.center_x_px > this->snapshot_.output_width - margin_x ||
         observation.center_y_px < margin_y ||
         observation.center_y_px > this->snapshot_.output_height - margin_y;
}

const CameraViewportSnapshot &CameraViewportController::snapshot() const { return this->snapshot_; }

const char *CameraViewportController::mode_text(CameraViewportMode mode) {
  switch (mode) {
    case CameraViewportMode::PRECISE_ROI: return "precise";
    case CameraViewportMode::SEARCH_FULL:
    default: return "search";
  }
}

ImagePoint CameraViewportController::to_reference_point_(const ImagePoint &point) const {
  ImagePoint result;
  result.x = this->snapshot_.window_x + (point.x + 0.5f) * this->snapshot_.scale_x - 0.5f;
  result.y = this->snapshot_.window_y + (point.y + 0.5f) * this->snapshot_.scale_y - 0.5f;
  return result;
}

void CameraViewportController::set_search_snapshot_() {
  this->snapshot_.supported = this->supports_precise_roi();
  this->snapshot_.mode = CameraViewportMode::SEARCH_FULL;
  this->snapshot_.reference_width = REFERENCE_WIDTH;
  this->snapshot_.reference_height = REFERENCE_HEIGHT;
  this->snapshot_.window_x = 0;
  this->snapshot_.window_y = 0;
  this->snapshot_.window_width = REFERENCE_WIDTH;
  this->snapshot_.window_height = REFERENCE_HEIGHT;
  this->snapshot_.output_width = OUTPUT_WIDTH;
  this->snapshot_.output_height = OUTPUT_HEIGHT;
  this->snapshot_.scale_x = static_cast<float>(REFERENCE_WIDTH) / OUTPUT_WIDTH;
  this->snapshot_.scale_y = static_cast<float>(REFERENCE_HEIGHT) / OUTPUT_HEIGHT;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
