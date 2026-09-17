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

bool CameraViewportController::apply_zoom_wide(float center_reference_x, float center_reference_y) {
  return this->apply_zoom_window_(CameraViewportMode::ZOOM_WIDE,
                                  center_reference_x, center_reference_y,
                                  ZOOM_WIDE_WIDTH, ZOOM_WIDE_HEIGHT);
}

bool CameraViewportController::apply_zoom_medium(float center_reference_x, float center_reference_y) {
  return this->apply_zoom_window_(CameraViewportMode::ZOOM_MEDIUM,
                                  center_reference_x, center_reference_y,
                                  ZOOM_MEDIUM_WIDTH, ZOOM_MEDIUM_HEIGHT);
}

bool CameraViewportController::apply_precise_roi(float center_reference_x, float center_reference_y) {
  return this->apply_zoom_window_(CameraViewportMode::PRECISE_ROI,
                                  center_reference_x, center_reference_y,
                                  OUTPUT_WIDTH, OUTPUT_HEIGHT);
}

bool CameraViewportController::recenter_current_zoom(float center_reference_x, float center_reference_y) {
  switch (this->snapshot_.mode) {
    case CameraViewportMode::ZOOM_WIDE:
      return this->apply_zoom_wide(center_reference_x, center_reference_y);
    case CameraViewportMode::ZOOM_MEDIUM:
      return this->apply_zoom_medium(center_reference_x, center_reference_y);
    case CameraViewportMode::PRECISE_ROI:
      return this->apply_precise_roi(center_reference_x, center_reference_y);
    case CameraViewportMode::SEARCH_FULL:
    default:
      return this->apply_search();
  }
}

bool CameraViewportController::apply_zoom_window_(CameraViewportMode mode,
                                                  float center_reference_x,
                                                  float center_reference_y,
                                                  uint16_t window_width,
                                                  uint16_t window_height) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->id.PID != OV5640_PID || sensor->set_res_raw == nullptr) {
    ESP_LOGE(TAG, "ROI zoom indisponible sur ce capteur");
    return false;
  }
  if (window_width < OUTPUT_WIDTH || window_height < OUTPUT_HEIGHT ||
      window_width > REFERENCE_WIDTH || window_height > REFERENCE_HEIGHT) {
    ESP_LOGE(TAG, "Taille viewport zoom invalide: %ux%u",
             static_cast<unsigned>(window_width), static_cast<unsigned>(window_height));
    return false;
  }

  const int32_t max_x = static_cast<int32_t>(REFERENCE_WIDTH - window_width);
  const int32_t max_y = static_cast<int32_t>(REFERENCE_HEIGHT - window_height);
  int32_t x = static_cast<int32_t>(std::lround(center_reference_x)) -
              static_cast<int32_t>(window_width / 2U);
  int32_t y = static_cast<int32_t>(std::lround(center_reference_y)) -
              static_cast<int32_t>(window_height / 2U);
  x = std::max<int32_t>(0, std::min<int32_t>(x, max_x));
  y = std::max<int32_t>(0, std::min<int32_t>(y, max_y));

  // Les fenetres successives gardent le meme rapport 4:3 que la reference.
  // Les marges 32/16 reproduisent le cadrage 4:3 du driver OV5640. Les niveaux
  // 1920x1440 et 1280x960 sont redimensionnes par l'ISP vers 800x600 ; le
  // niveau final 800x600 est lu en natif sans scaling.
  const int start_x = x;
  const int start_y = y;
  const int end_x = x + window_width + 2 * SENSOR_MARGIN_X - 1;
  const int end_y = y + window_height + 2 * SENSOR_MARGIN_Y - 1;
  const bool scaling = window_width != OUTPUT_WIDTH || window_height != OUTPUT_HEIGHT;

  if (sensor->set_res_raw(sensor, start_x, start_y, end_x, end_y,
                          SENSOR_MARGIN_X, SENSOR_MARGIN_Y,
                          SENSOR_TOTAL_X, SENSOR_TOTAL_Y,
                          OUTPUT_WIDTH, OUTPUT_HEIGHT,
                          scaling, false) != 0) {
    ESP_LOGE(TAG, "Echec set_res_raw viewport mode=%s x=%d y=%d %ux%u",
             mode_text(mode), static_cast<int>(x), static_cast<int>(y),
             static_cast<unsigned>(window_width), static_cast<unsigned>(window_height));
    return false;
  }

  // set_res_raw met a jour status.scale/status.binning mais ne rappelle pas
  // set_image_options() dans le driver OV5640. Reappliquer l'orientation
  // courante force la programmation des registres de binning/increment.
  if (sensor->set_hmirror != nullptr) {
    const int hmirror = sensor->status.hmirror ? 1 : 0;
    if (sensor->set_hmirror(sensor, hmirror) != 0) {
      ESP_LOGE(TAG, "Echec reapplication options OV5640 apres zoom");
      return false;
    }
  } else if (sensor->set_vflip != nullptr) {
    const int vflip = sensor->status.vflip ? 1 : 0;
    if (sensor->set_vflip(sensor, vflip) != 0) {
      ESP_LOGE(TAG, "Echec reapplication options OV5640 apres zoom");
      return false;
    }
  }

  sensor->status.framesize = FRAMESIZE_SVGA;

  this->snapshot_.supported = true;
  this->snapshot_.mode = mode;
  this->snapshot_.reference_width = REFERENCE_WIDTH;
  this->snapshot_.reference_height = REFERENCE_HEIGHT;
  this->snapshot_.window_x = static_cast<uint16_t>(x);
  this->snapshot_.window_y = static_cast<uint16_t>(y);
  this->snapshot_.window_width = window_width;
  this->snapshot_.window_height = window_height;
  this->snapshot_.output_width = OUTPUT_WIDTH;
  this->snapshot_.output_height = OUTPUT_HEIGHT;
  this->snapshot_.scale_x = static_cast<float>(window_width) / OUTPUT_WIDTH;
  this->snapshot_.scale_y = static_cast<float>(window_height) / OUTPUT_HEIGHT;

  ESP_LOGI(TAG,
           "Viewport %s: ref centre=(%.1f,%.1f), fenetre x=%u y=%u %ux%u -> %ux%u scale=(%.3f,%.3f)",
           mode_text(mode), center_reference_x, center_reference_y,
           static_cast<unsigned>(this->snapshot_.window_x),
           static_cast<unsigned>(this->snapshot_.window_y),
           static_cast<unsigned>(this->snapshot_.window_width),
           static_cast<unsigned>(this->snapshot_.window_height),
           static_cast<unsigned>(this->snapshot_.output_width),
           static_cast<unsigned>(this->snapshot_.output_height),
           this->snapshot_.scale_x, this->snapshot_.scale_y);
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
    case CameraViewportMode::ZOOM_WIDE: return "zoom_wide";
    case CameraViewportMode::ZOOM_MEDIUM: return "zoom_medium";
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
