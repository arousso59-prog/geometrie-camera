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
      sensor_window_x(0),
      sensor_window_y(0),
      horizontal_mirror(false),
      vertical_flip(false),
      output_width(CameraViewportController::OUTPUT_WIDTH),
      output_height(CameraViewportController::OUTPUT_HEIGHT),
      scale_x(static_cast<float>(CameraViewportController::REFERENCE_WIDTH) /
              CameraViewportController::OUTPUT_WIDTH),
      scale_y(static_cast<float>(CameraViewportController::REFERENCE_HEIGHT) /
              CameraViewportController::OUTPUT_HEIGHT) {}

CameraViewportController::CameraViewportController(CameraResolutionController *resolution_controller)
    : resolution_controller_(resolution_controller),
      snapshot_(),
      reference_pattern_features_{} {
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

bool CameraViewportController::apply_zoom_fine(float center_reference_x, float center_reference_y) {
  return this->apply_zoom_window_(CameraViewportMode::ZOOM_FINE,
                                  center_reference_x, center_reference_y,
                                  ZOOM_FINE_WIDTH, ZOOM_FINE_HEIGHT);
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
    case CameraViewportMode::ZOOM_FINE:
      return this->apply_zoom_fine(center_reference_x, center_reference_y);
    case CameraViewportMode::PRECISE_ROI:
      return this->apply_precise_roi(center_reference_x, center_reference_y);
    case CameraViewportMode::SEARCH_FULL:
    default:
      return this->apply_search();
  }
}


bool CameraViewportController::recenter_current_zoom_from_local(float local_center_x,
                                                                float local_center_y,
                                                                bool &viewport_moved) {
  viewport_moved = false;
  if (this->snapshot_.mode == CameraViewportMode::SEARCH_FULL ||
      this->snapshot_.output_width == 0 || this->snapshot_.output_height == 0) {
    return false;
  }

  const float desired_local_x = this->snapshot_.output_width * 0.5f;
  const float desired_local_y = this->snapshot_.output_height * 0.5f;
  const float error_local_x = local_center_x - desired_local_x;
  const float error_local_y = local_center_y - desired_local_y;

  // Boucle fermee : on translate la fenetre courante de l'erreur observee
  // multipliee par l'echelle du viewport courant. Un gain de 0.80 evite les
  // sur-corrections si le mapping reel du capteur differe legerement du modele.
  constexpr float FEEDBACK_GAIN = 0.80f;
  const float requested_center_x =
      this->snapshot_.window_x + this->snapshot_.window_width * 0.5f +
      error_local_x * this->snapshot_.scale_x * FEEDBACK_GAIN;
  const float requested_center_y =
      this->snapshot_.window_y + this->snapshot_.window_height * 0.5f +
      error_local_y * this->snapshot_.scale_y * FEEDBACK_GAIN;

  const uint16_t previous_x = this->snapshot_.window_x;
  const uint16_t previous_y = this->snapshot_.window_y;
  const CameraViewportMode mode = this->snapshot_.mode;

  if (!this->recenter_current_zoom(requested_center_x, requested_center_y)) {
    return false;
  }

  viewport_moved = this->snapshot_.window_x != previous_x ||
                   this->snapshot_.window_y != previous_y;

  ESP_LOGI(TAG,
           "Recentrage boucle fermee %s: local=(%.1f,%.1f) erreur=(%.1f,%.1f) "
           "ROI (%u,%u)->(%u,%u) moved=%s",
           mode_text(mode),
           local_center_x, local_center_y,
           error_local_x, error_local_y,
           static_cast<unsigned>(previous_x), static_cast<unsigned>(previous_y),
           static_cast<unsigned>(this->snapshot_.window_x),
           static_cast<unsigned>(this->snapshot_.window_y),
           viewport_moved ? "oui" : "non");
  return true;
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

  // IMPORTANT : window_x/window_y restent dans le repere canonique visible
  // par le detecteur et l'utilisateur. ESPHome active par defaut hmirror et
  // vflip sur esp32_camera ; les registres set_res_raw(), eux, travaillent
  // dans le repere physique du capteur. Il faut donc convertir ici.
  int32_t display_x = static_cast<int32_t>(std::lround(center_reference_x)) -
                      static_cast<int32_t>(window_width / 2U);
  int32_t display_y = static_cast<int32_t>(std::lround(center_reference_y)) -
                      static_cast<int32_t>(window_height / 2U);
  display_x = std::max<int32_t>(0, std::min<int32_t>(display_x, max_x));
  display_y = std::max<int32_t>(0, std::min<int32_t>(display_y, max_y));

  const bool hmirror = sensor->status.hmirror;
  const bool vflip = sensor->status.vflip;

  const int32_t sensor_x = hmirror
                               ? static_cast<int32_t>(REFERENCE_WIDTH - window_width) - display_x
                               : display_x;
  const int32_t sensor_y = vflip
                               ? static_cast<int32_t>(REFERENCE_HEIGHT - window_height) - display_y
                               : display_y;

  // Les fenetres successives gardent le meme rapport 4:3 que la reference.
  // Les marges 32/16 reproduisent le cadrage 4:3 officiel du driver OV5640.
  const int start_x = sensor_x;
  const int start_y = sensor_y;
  const int end_x = sensor_x + window_width + 2 * SENSOR_MARGIN_X - 1;
  const int end_y = sensor_y + window_height + 2 * SENSOR_MARGIN_Y - 1;
  const bool scaling = window_width != OUTPUT_WIDTH || window_height != OUTPUT_HEIGHT;

  if (sensor->set_res_raw(sensor, start_x, start_y, end_x, end_y,
                          SENSOR_MARGIN_X, SENSOR_MARGIN_Y,
                          SENSOR_TOTAL_X, SENSOR_TOTAL_Y,
                          OUTPUT_WIDTH, OUTPUT_HEIGHT,
                          scaling, false) != 0) {
    ESP_LOGE(TAG,
             "Echec set_res_raw viewport mode=%s display=(%d,%d) sensor=(%d,%d) %ux%u",
             mode_text(mode),
             static_cast<int>(display_x), static_cast<int>(display_y),
             static_cast<int>(sensor_x), static_cast<int>(sensor_y),
             static_cast<unsigned>(window_width), static_cast<unsigned>(window_height));
    return false;
  }

  sensor->status.framesize = FRAMESIZE_SVGA;

  this->snapshot_.supported = true;
  this->snapshot_.mode = mode;
  this->snapshot_.reference_width = REFERENCE_WIDTH;
  this->snapshot_.reference_height = REFERENCE_HEIGHT;
  this->snapshot_.window_x = static_cast<uint16_t>(display_x);
  this->snapshot_.window_y = static_cast<uint16_t>(display_y);
  this->snapshot_.window_width = window_width;
  this->snapshot_.window_height = window_height;
  this->snapshot_.sensor_window_x = static_cast<uint16_t>(sensor_x);
  this->snapshot_.sensor_window_y = static_cast<uint16_t>(sensor_y);
  this->snapshot_.horizontal_mirror = hmirror;
  this->snapshot_.vertical_flip = vflip;
  this->snapshot_.output_width = OUTPUT_WIDTH;
  this->snapshot_.output_height = OUTPUT_HEIGHT;
  this->snapshot_.scale_x = static_cast<float>(window_width) / OUTPUT_WIDTH;
  this->snapshot_.scale_y = static_cast<float>(window_height) / OUTPUT_HEIGHT;

  ESP_LOGI(TAG,
           "Viewport %s: centre canonique=(%.1f,%.1f), display=(%u,%u %ux%u), "
           "sensor=(%u,%u), mirror=%s flip=%s -> %ux%u scale=(%.3f,%.3f)",
           mode_text(mode), center_reference_x, center_reference_y,
           static_cast<unsigned>(this->snapshot_.window_x),
           static_cast<unsigned>(this->snapshot_.window_y),
           static_cast<unsigned>(this->snapshot_.window_width),
           static_cast<unsigned>(this->snapshot_.window_height),
           static_cast<unsigned>(this->snapshot_.sensor_window_x),
           static_cast<unsigned>(this->snapshot_.sensor_window_y),
           hmirror ? "ON" : "OFF", vflip ? "ON" : "OFF",
           static_cast<unsigned>(this->snapshot_.output_width),
           static_cast<unsigned>(this->snapshot_.output_height),
           this->snapshot_.scale_x, this->snapshot_.scale_y);
  return true;
}

TargetObservation CameraViewportController::to_reference(
    const TargetObservation &observation) const {
  TargetObservation result = observation;

  const float sx = this->snapshot_.scale_x;
  const float sy = this->snapshot_.scale_y;
  const float pixel_scale = 0.5f * (sx + sy);
  const float offset_x =
      static_cast<float>(this->snapshot_.window_x) + 0.5f * sx - 0.5f;
  const float offset_y =
      static_cast<float>(this->snapshot_.window_y) + 0.5f * sy - 0.5f;

  result.center_x_px = sx * observation.center_x_px + offset_x;
  result.center_y_px = sy * observation.center_y_px + offset_y;
  result.top_left_px = this->to_reference_point_(observation.top_left_px);
  result.top_right_px = this->to_reference_point_(observation.top_right_px);
  result.bottom_right_px = this->to_reference_point_(observation.bottom_right_px);
  result.bottom_left_px = this->to_reference_point_(observation.bottom_left_px);

  auto transform_line = [&](const ImageLine &source) -> ImageLine {
    ImageLine line = source;
    if (!source.valid) return line;
    line.point = this->to_reference_point_(source.point);
    line.dx = source.dx * sx;
    line.dy = source.dy * sy;
    const float norm = std::sqrt(line.dx * line.dx + line.dy * line.dy);
    if (!std::isfinite(norm) || norm < 1.0e-7f) {
      line.valid = false;
      return line;
    }
    line.dx /= norm;
    line.dy /= norm;
    return line;
  };

  result.subpixel_top_line =
      transform_line(observation.subpixel_top_line);
  result.subpixel_right_line =
      transform_line(observation.subpixel_right_line);
  result.subpixel_bottom_line =
      transform_line(observation.subpixel_bottom_line);
  result.subpixel_left_line =
      transform_line(observation.subpixel_left_line);

  if (observation.valid) {
    result.width_px =
        0.5f * (point_distance(result.top_left_px, result.top_right_px) +
                point_distance(result.bottom_left_px, result.bottom_right_px));
    result.height_px =
        0.5f * (point_distance(result.top_left_px, result.bottom_left_px) +
                point_distance(result.top_right_px, result.bottom_right_px));
  } else {
    result.width_px = observation.width_px * sx;
    result.height_px = observation.height_px * sy;
  }

  // Toutes les quantites metrologiques doivent etre dans le MEME repere que
  // fx/fy. Avant V34, seuls les coins et width/height principaux etaient
  // remappes : les droites/homographies restaient en coordonnees ROI, ce qui
  // biaisait surtout la pose angulaire.
  result.subpixel_width_px = observation.subpixel_width_px * sx;
  result.subpixel_height_px = observation.subpixel_height_px * sy;
  result.subpixel_width_sigma_px =
      observation.subpixel_width_sigma_px * sx;
  result.subpixel_height_sigma_px =
      observation.subpixel_height_sigma_px * sy;

  result.subpixel_v5_width_px = observation.subpixel_v5_width_px * sx;
  result.subpixel_v5_height_px = observation.subpixel_v5_height_px * sy;
  result.subpixel_v5_width_sigma_px =
      observation.subpixel_v5_width_sigma_px * sx;
  result.subpixel_v5_height_sigma_px =
      observation.subpixel_v5_height_sigma_px * sy;

  result.subpixel_v6_width_px = observation.subpixel_v6_width_px * sx;
  result.subpixel_v6_height_px = observation.subpixel_v6_height_px * sy;
  result.subpixel_v6_width_sigma_px =
      observation.subpixel_v6_width_sigma_px * sx;
  result.subpixel_v6_height_sigma_px =
      observation.subpixel_v6_height_sigma_px * sy;

  result.subpixel_rms_px = observation.subpixel_rms_px * pixel_scale;
  result.subpixel_max_rms_px =
      observation.subpixel_max_rms_px * pixel_scale;
  result.subpixel_top_rms_px =
      observation.subpixel_top_rms_px * pixel_scale;
  result.subpixel_right_rms_px =
      observation.subpixel_right_rms_px * pixel_scale;
  result.subpixel_bottom_rms_px =
      observation.subpixel_bottom_rms_px * pixel_scale;
  result.subpixel_left_rms_px =
      observation.subpixel_left_rms_px * pixel_scale;

  result.pattern_rms_px = observation.pattern_rms_px * pixel_scale;
  result.pattern_max_residual_px =
      observation.pattern_max_residual_px * pixel_scale;

  if (observation.pattern_refined) {
    // H_ref = A_roi->reference * H_local.
    const float *src = observation.pattern_homography;
    float *dst = result.pattern_homography;
    dst[0] = sx * src[0] + offset_x * src[6];
    dst[1] = sx * src[1] + offset_x * src[7];
    dst[2] = sx * src[2] + offset_x * src[8];
    dst[3] = sy * src[3] + offset_y * src[6];
    dst[4] = sy * src[4] + offset_y * src[7];
    dst[5] = sy * src[5] + offset_y * src[8];
    dst[6] = src[6];
    dst[7] = src[7];
    dst[8] = src[8];
  }

  if (observation.pattern_features != nullptr &&
      observation.pattern_features_count > 0) {
    const uint16_t count = std::min<uint16_t>(
        observation.pattern_features_count,
        MAX_REFERENCE_PATTERN_FEATURES);
    for (uint16_t i = 0; i < count; ++i) {
      this->reference_pattern_features_[i] =
          observation.pattern_features[i];
      this->reference_pattern_features_[i].x =
          sx * observation.pattern_features[i].x + offset_x;
      this->reference_pattern_features_[i].y =
          sy * observation.pattern_features[i].y + offset_y;
      this->reference_pattern_features_[i].residual =
          observation.pattern_features[i].residual * pixel_scale;
    }
    result.pattern_features = this->reference_pattern_features_;
    result.pattern_features_count = count;
  } else {
    result.pattern_features = nullptr;
    result.pattern_features_count = 0;
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
    case CameraViewportMode::ZOOM_FINE: return "zoom_fine";
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
  sensor_t *sensor = esp_camera_sensor_get();
  this->snapshot_.sensor_window_x = 0;
  this->snapshot_.sensor_window_y = 0;
  this->snapshot_.horizontal_mirror = sensor != nullptr ? sensor->status.hmirror : false;
  this->snapshot_.vertical_flip = sensor != nullptr ? sensor->status.vflip : false;
  this->snapshot_.output_width = OUTPUT_WIDTH;
  this->snapshot_.output_height = OUTPUT_HEIGHT;
  this->snapshot_.scale_x = static_cast<float>(REFERENCE_WIDTH) / OUTPUT_WIDTH;
  this->snapshot_.scale_y = static_cast<float>(REFERENCE_HEIGHT) / OUTPUT_HEIGHT;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
