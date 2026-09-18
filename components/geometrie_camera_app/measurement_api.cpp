#include "measurement_api.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>

#include "esphome/core/hal.h"
#include "geometry_measurement.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_manager.h"
#include "target_detection_service.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

MeasurementApiHandler::MeasurementApiHandler(MeasurementManager *manager,
                                             TargetDetectionService *detection_service,
                                             JpegFilteredDiagnostic *source)
    : manager_(manager), detection_service_(detection_service), source_(source) {}

bool MeasurementApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/measurement/compute" ||
         url == "/measurement/status" ||
         url == "/measurement/config" ||
         url == "/measurement/config/set" ||
         url == "/measurement/calibrate";
}

void MeasurementApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/measurement/compute") {
    this->handle_compute_(request);
    return;
  }
  if (url == "/measurement/status") {
    this->handle_status_(request);
    return;
  }
  if (url == "/measurement/config") {
    this->handle_config_(request);
    return;
  }
  if (url == "/measurement/config/set") {
    this->handle_config_set_(request);
    return;
  }
  if (url == "/measurement/calibrate") {
    this->handle_calibrate_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool MeasurementApiHandler::parse_float_param_(AsyncWebServerRequest *request, const char *name,
                                               float &value, bool &present,
                                               std::string &error) const {
  present = request->hasParam(name);
  if (!present) {
    return true;
  }

  const std::string text = request->getParam(name)->value();
  if (text.empty()) {
    error = std::string(name) + "_invalid";
    return false;
  }

  errno = 0;
  char *end = nullptr;
  const float parsed = std::strtof(text.c_str(), &end);
  if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    error = std::string(name) + "_invalid";
    return false;
  }

  value = parsed;
  return true;
}

bool MeasurementApiHandler::detection_is_current_() const {
  return this->detection_service_ != nullptr && this->source_ != nullptr &&
         this->detection_service_->ready() && this->source_->ready() &&
         this->detection_service_->source_process_count() == this->source_->process_count();
}

void MeasurementApiHandler::handle_compute_(AsyncWebServerRequest *request) {
  if (this->manager_ == nullptr || this->detection_service_ == nullptr || this->source_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "measurement_unavailable");
    return;
  }

  if (!this->manager_->measurement_engine().has_calibration()) {
    this->send_snapshot_(request, 409, "error", "calibration_required");
    return;
  }

  if (!this->detection_is_current_()) {
    this->send_snapshot_(request, 409, "error", "current_target_detection_required");
    return;
  }

  if (!this->detection_service_->target_found()) {
    this->send_snapshot_(request, 409, "error", "target_not_found");
    return;
  }

  if (!this->manager_->process(this->detection_service_->last_observation(),
                               this->source_->width(), this->source_->height(), millis())) {
    this->send_snapshot_(request, 500, "error", "measurement_failed");
    return;
  }

  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->manager_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "measurement_unavailable");
    return;
  }
  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::handle_config_(AsyncWebServerRequest *request) const {
  if (this->manager_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "measurement_unavailable");
    return;
  }
  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::handle_config_set_(AsyncWebServerRequest *request) {
  if (this->manager_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "measurement_unavailable");
    return;
  }

  float target_size_mm = 0.0f;
  float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f, k3 = 0.0f;
  float clear_distortion_value = 0.0f;
  bool has_target_size = false;
  bool has_k1 = false, has_k2 = false, has_p1 = false, has_p2 = false, has_k3 = false;
  bool has_clear_distortion = false;
  std::string error;
  if (!this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error) ||
      !this->parse_float_param_(request, "k1", k1, has_k1, error) ||
      !this->parse_float_param_(request, "k2", k2, has_k2, error) ||
      !this->parse_float_param_(request, "p1", p1, has_p1, error) ||
      !this->parse_float_param_(request, "p2", p2, has_p2, error) ||
      !this->parse_float_param_(request, "k3", k3, has_k3, error) ||
      !this->parse_float_param_(request, "clear_distortion", clear_distortion_value,
                                has_clear_distortion, error)) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }

  const bool has_any_distortion = has_k1 || has_k2 || has_p1 || has_p2 || has_k3;
  if (!has_target_size && !has_any_distortion && !has_clear_distortion) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"no_setting_provided\"}");
    return;
  }

  if (has_any_distortion && !(has_k1 && has_k2 && has_p1 && has_p2 && has_k3)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"all_distortion_coefficients_required\"}");
    return;
  }

  if (has_clear_distortion &&
      clear_distortion_value != 0.0f && clear_distortion_value != 1.0f) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"clear_distortion_must_be_0_or_1\"}");
    return;
  }

  GeometryMeasurementEngine &engine = this->manager_->measurement_engine();

  if (has_target_size) {
    if (target_size_mm < 1.0f || target_size_mm > 1000.0f ||
        !engine.set_target_size_mm(target_size_mm)) {
      request->send(400, "application/json",
                    "{\"status\":\"error\",\"error\":\"target_size_mm_out_of_range\"}");
      return;
    }

    // Changer la taille physique invalide fx/fy. Les coefficients de
    // distorsion sont independants de la taille et pourront etre reappliques
    // par la calibration full suivante.
    const CameraCalibration previous = engine.calibration();
    engine.clear_calibration();
    if (has_any_distortion) {
      engine.set_distortion_coefficients(k1, k2, p1, p2, k3);
    } else if (!has_clear_distortion || clear_distortion_value == 0.0f) {
      engine.set_distortion_coefficients(previous.k1, previous.k2,
                                         previous.p1, previous.p2, previous.k3);
    }
  }

  if (has_clear_distortion && clear_distortion_value == 1.0f) {
    engine.clear_distortion();
  }

  if (has_any_distortion &&
      !engine.set_distortion_coefficients(k1, k2, p1, p2, k3)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"distortion_coefficients_invalid\"}");
    return;
  }

  this->manager_->reset();
  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::handle_calibrate_(AsyncWebServerRequest *request) {
  if (this->manager_ == nullptr || this->detection_service_ == nullptr || this->source_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "measurement_unavailable");
    return;
  }

  float distance_mm = 0.0f;
  float target_size_mm = 0.0f;
  float force_value = 0.0f;
  bool has_distance = false;
  bool has_target_size = false;
  bool has_force = false;
  std::string error;
  if (!this->parse_float_param_(request, "distance_mm", distance_mm, has_distance, error) ||
      !this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error) ||
      !this->parse_float_param_(request, "force", force_value, has_force, error)) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }

  if (!has_distance || distance_mm < 50.0f || distance_mm > 20000.0f) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"distance_mm_out_of_range\"}");
    return;
  }

  if (has_force && force_value != 0.0f && force_value != 1.0f) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"force_must_be_0_or_1\"}");
    return;
  }
  const bool force = has_force && force_value == 1.0f;

  if (has_target_size && (target_size_mm < 1.0f || target_size_mm > 1000.0f)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"target_size_mm_out_of_range\"}");
    return;
  }

  GeometryMeasurementEngine &engine = this->manager_->measurement_engine();
  if (engine.has_calibration() && !force) {
    this->send_snapshot_(request, 409, "error", "calibration_locked");
    return;
  }

  // Verifier toute la chaine de detection avant de toucher a la configuration
  // existante. Une requete de recalibration forcee mais stale ne doit rien muter.
  if (!this->detection_is_current_()) {
    this->send_snapshot_(request, 409, "error", "current_target_detection_required");
    return;
  }
  if (!this->detection_service_->target_found()) {
    this->send_snapshot_(request, 409, "error", "target_not_found");
    return;
  }

  const float previous_target_size = engine.target_size_mm();
  const CameraCalibration previous_calibration = engine.calibration();

  if (has_target_size && !engine.set_target_size_mm(target_size_mm)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"target_size_mm_out_of_range\"}");
    return;
  }

  if (!engine.calibrate_from_known_distance(this->detection_service_->last_observation(),
                                            this->source_->width(), this->source_->height(),
                                            distance_mm)) {
    engine.set_target_size_mm(previous_target_size);
    engine.set_calibration(previous_calibration);
    this->send_snapshot_(request, 500, "error", "calibration_failed");
    return;
  }

  this->manager_->reset();
  if (!this->manager_->process(this->detection_service_->last_observation(),
                               this->source_->width(), this->source_->height(), millis())) {
    engine.set_target_size_mm(previous_target_size);
    engine.set_calibration(previous_calibration);
    this->manager_->reset();
    this->send_snapshot_(request, 500, "error", "measurement_after_calibration_failed");
    return;
  }

  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::send_snapshot_(AsyncWebServerRequest *request, int response_code,
                                           const char *status, const char *error) const {
  std::string json;
  json.reserve(12000);
  json += "{\"status\":\"";
  json += status;
  json += "\"";

  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }

  if (this->manager_ != nullptr) {
    const GeometryMeasurementEngine &engine = this->manager_->measurement_engine();
    const GeometryMeasurement &measurement = this->manager_->last_measurement();
    const GeometryMeasurement &raw_measurement = this->manager_->raw_measurement();
    const CameraCalibration &stored = engine.calibration();

    const uint16_t frame_width = this->source_ != nullptr ? this->source_->width() : 0;
    const uint16_t frame_height = this->source_ != nullptr ? this->source_->height() : 0;
    const CameraCalibration effective = engine.effective_calibration(frame_width, frame_height);

    json += ",\"ready\":";
    json += measurement.valid ? "true" : "false";
    json += ",\"measurement_count\":";
    json += std::to_string(this->manager_->valid_measurement_count());
    json += ",\"detection_current\":";
    json += this->detection_is_current_() ? "true" : "false";
    json += ",\"target_found\":";
    json += (this->detection_service_ != nullptr && this->detection_service_->target_found()) ? "true" : "false";

    json += ",\"config\":{\"target_size_mm\":";
    json += std::to_string(engine.target_size_mm());
    json += "}";

    // Methode operationnelle figee apres comparaison V5/V6/V6.1 :
    // dimensions V6.1 (droites robustes V5 + incertitude V6) puis
    // stabilisation robuste sur 5 mesures. V5/V6 restent diagnostics.
    json += ",\"method\":{";
    json += "\"name\":\"V6.1-robust5\"";
    json += ",\"frozen\":true";
    json += ",\"stabilization_window\":5";
    json += ",\"diagnostics_only\":[\"V5\",\"V6\"]";
    json += "}";

    json += ",\"calibration\":{\"valid\":";
    json += engine.has_calibration() ? "true" : "false";
    json += ",\"locked\":";
    json += engine.has_calibration() ? "true" : "false";
    json += ",\"fx_px\":" + std::to_string(stored.fx_px);
    json += ",\"fy_px\":" + std::to_string(stored.fy_px);
    json += ",\"cx_px\":" + std::to_string(stored.cx_px);
    json += ",\"cy_px\":" + std::to_string(stored.cy_px);
    json += ",\"k1\":" + std::to_string(stored.k1);
    json += ",\"k2\":" + std::to_string(stored.k2);
    json += ",\"p1\":" + std::to_string(stored.p1);
    json += ",\"p2\":" + std::to_string(stored.p2);
    json += ",\"k3\":" + std::to_string(stored.k3);
    const bool distortion_enabled =
        std::fabs(stored.k1) > 1.0e-12f || std::fabs(stored.k2) > 1.0e-12f ||
        std::fabs(stored.p1) > 1.0e-12f || std::fabs(stored.p2) > 1.0e-12f ||
        std::fabs(stored.k3) > 1.0e-12f;
    json += ",\"distortion_enabled\":";
    json += distortion_enabled ? "true" : "false";
    json += ",\"reference_width_px\":" + std::to_string(stored.reference_width_px);
    json += ",\"reference_height_px\":" + std::to_string(stored.reference_height_px);
    json += ",\"effective_fx_px\":" + std::to_string(effective.fx_px);
    json += ",\"effective_fy_px\":" + std::to_string(effective.fy_px);
    json += ",\"effective_cx_px\":" + std::to_string(effective.cx_px);
    json += ",\"effective_cy_px\":" + std::to_string(effective.cy_px);
    json += "}";

    json += ",\"measurement\":{\"valid\":";
    json += measurement.valid ? "true" : "false";
    json += ",\"pose_valid\":";
    json += measurement.pose_valid ? "true" : "false";
    json += ",\"edge_v4_used\":";
    json += measurement.edge_v4_used ? "true" : "false";
    json += ",\"edge_v5_used\":";
    json += measurement.edge_v5_used ? "true" : "false";
    json += ",\"edge_v6_used\":";
    json += measurement.edge_v6_used ? "true" : "false";
    json += ",\"apparent_width_px\":" + std::to_string(measurement.apparent_width_px);
    json += ",\"apparent_height_px\":" + std::to_string(measurement.apparent_height_px);
    json += ",\"apparent_width_sigma_px\":" + std::to_string(measurement.apparent_width_sigma_px);
    json += ",\"apparent_height_sigma_px\":" + std::to_string(measurement.apparent_height_sigma_px);
    json += ",\"width_distance_weight\":" + std::to_string(measurement.width_distance_weight);
    json += ",\"height_distance_weight\":" + std::to_string(measurement.height_distance_weight);
    json += ",\"distance_mm\":" + std::to_string(measurement.distance_mm);
    json += ",\"x_mm\":" + std::to_string(measurement.x_mm);
    json += ",\"y_mm\":" + std::to_string(measurement.y_mm);
    json += ",\"z_mm\":" + std::to_string(measurement.z_mm);
    json += ",\"z_from_width_mm\":" + std::to_string(measurement.z_from_width_mm);
    json += ",\"z_from_height_mm\":" + std::to_string(measurement.z_from_height_mm);
    json += ",\"bearing_yaw_deg\":" + std::to_string(measurement.bearing_yaw_deg);
    json += ",\"bearing_pitch_deg\":" + std::to_string(measurement.bearing_pitch_deg);
    json += ",\"target_yaw_deg\":" + std::to_string(measurement.yaw_deg);
    json += ",\"target_pitch_deg\":" + std::to_string(measurement.pitch_deg);
    json += ",\"target_roll_deg\":" + std::to_string(measurement.roll_deg);
    json += ",\"pose_z_mm\":" + std::to_string(measurement.pose_z_mm);
    json += ",\"pose_scale_error_pct\":" + std::to_string(measurement.pose_scale_error_pct);
    json += ",\"pose_method\":\"";
    json += measurement.pose_v4_used ? "pattern_direct_v4" :
            (measurement.pose_v3_used ? "pattern_v3" :
             (measurement.pose_v2_used ? "line_v2" :
              (measurement.pose_v1_valid ? "homography_v1" : "none")));
    json += "\"";
    json += ",\"pose_v1_valid\":";
    json += measurement.pose_v1_valid ? "true" : "false";
    json += ",\"pose_v2_valid\":";
    json += measurement.pose_v2_valid ? "true" : "false";
    json += ",\"pose_v2_used\":";
    json += measurement.pose_v2_used ? "true" : "false";
    json += ",\"pose_v3_valid\":";
    json += measurement.pose_v3_valid ? "true" : "false";
    json += ",\"pose_v3_used\":";
    json += measurement.pose_v3_used ? "true" : "false";
    json += ",\"pose_v4_valid\":";
    json += measurement.pose_v4_valid ? "true" : "false";
    json += ",\"pose_v4_used\":";
    json += measurement.pose_v4_used ? "true" : "false";
    json += ",\"pose_v1_yaw_deg\":" + std::to_string(measurement.pose_v1_yaw_deg);
    json += ",\"pose_v1_pitch_deg\":" + std::to_string(measurement.pose_v1_pitch_deg);
    json += ",\"pose_v1_roll_deg\":" + std::to_string(measurement.pose_v1_roll_deg);
    json += ",\"pose_v2_yaw_deg\":" + std::to_string(measurement.pose_v2_yaw_deg);
    json += ",\"pose_v2_pitch_deg\":" + std::to_string(measurement.pose_v2_pitch_deg);
    json += ",\"pose_v2_roll_deg\":" + std::to_string(measurement.pose_v2_roll_deg);
    json += ",\"pose_v2_line_rms_px\":" + std::to_string(measurement.pose_v2_line_rms_px);
    json += ",\"pose_v2_corner_rms_px\":" + std::to_string(measurement.pose_v2_corner_rms_px);
    json += ",\"pose_v3_yaw_deg\":" + std::to_string(measurement.pose_v3_yaw_deg);
    json += ",\"pose_v3_pitch_deg\":" + std::to_string(measurement.pose_v3_pitch_deg);
    json += ",\"pose_v3_roll_deg\":" + std::to_string(measurement.pose_v3_roll_deg);
    json += ",\"pose_v3_pattern_rms_px\":" + std::to_string(measurement.pose_v3_pattern_rms_px);
    json += ",\"pose_v3_fit_rms_px\":" + std::to_string(measurement.pose_v3_fit_rms_px);
    json += ",\"pose_v3_feature_count\":" + std::to_string(measurement.pose_v3_feature_count);
    json += ",\"pose_v3_inlier_count\":" + std::to_string(measurement.pose_v3_inlier_count);
    json += ",\"pose_v4_yaw_deg\":" + std::to_string(measurement.pose_v4_yaw_deg);
    json += ",\"pose_v4_pitch_deg\":" + std::to_string(measurement.pose_v4_pitch_deg);
    json += ",\"pose_v4_roll_deg\":" + std::to_string(measurement.pose_v4_roll_deg);
    json += ",\"pose_v4_rms_px\":" + std::to_string(measurement.pose_v4_rms_px);
    json += ",\"pose_v4_max_residual_px\":" + std::to_string(measurement.pose_v4_max_residual_px);
    json += ",\"pose_v4_feature_count\":" + std::to_string(measurement.pose_v4_feature_count);
    json += ",\"pose_v4_inlier_count\":" + std::to_string(measurement.pose_v4_inlier_count);
    json += ",\"quality\":" + std::to_string(measurement.quality);
    json += ",\"timestamp_ms\":" + std::to_string(measurement.timestamp_ms);
    json += "}";

    json += ",\"stabilization\":{";
    json += "\"active\":";
    json += this->manager_->last_measurement_stabilized() ? "true" : "false";
    json += ",\"sample_count\":" +
            std::to_string(this->manager_->stabilization_sample_count());
    json += ",\"window_size\":" +
            std::to_string(this->manager_->stabilization_window_size());
    json += ",\"distance_stddev_mm\":" +
            std::to_string(this->manager_->distance_stddev_mm());
    json += ",\"distance_span_mm\":" +
            std::to_string(this->manager_->distance_span_mm());
    json += "}";

    json += ",\"raw_measurement\":{\"valid\":";
    json += raw_measurement.valid ? "true" : "false";
    json += ",\"pose_valid\":";
    json += raw_measurement.pose_valid ? "true" : "false";
    json += ",\"edge_v4_used\":";
    json += raw_measurement.edge_v4_used ? "true" : "false";
    json += ",\"edge_v5_used\":";
    json += raw_measurement.edge_v5_used ? "true" : "false";
    json += ",\"edge_v6_used\":";
    json += raw_measurement.edge_v6_used ? "true" : "false";
    json += ",\"apparent_width_px\":" + std::to_string(raw_measurement.apparent_width_px);
    json += ",\"apparent_height_px\":" + std::to_string(raw_measurement.apparent_height_px);
    json += ",\"apparent_width_sigma_px\":" + std::to_string(raw_measurement.apparent_width_sigma_px);
    json += ",\"apparent_height_sigma_px\":" + std::to_string(raw_measurement.apparent_height_sigma_px);

    json += ",\"precision_diag\":{";
    json += "\"corner_width_px\":" + std::to_string(raw_measurement.corner_width_px);
    json += ",\"corner_height_px\":" + std::to_string(raw_measurement.corner_height_px);
    json += ",\"v5_width_px\":" + std::to_string(raw_measurement.v5_width_px);
    json += ",\"v5_height_px\":" + std::to_string(raw_measurement.v5_height_px);
    json += ",\"v6_width_px\":" + std::to_string(raw_measurement.v6_width_px);
    json += ",\"v6_height_px\":" + std::to_string(raw_measurement.v6_height_px);
    json += ",\"v61_width_px\":" + std::to_string(raw_measurement.v61_width_px);
    json += ",\"v61_height_px\":" + std::to_string(raw_measurement.v61_height_px);
    json += ",\"v5_v6_width_delta_px\":" + std::to_string(raw_measurement.v5_v6_width_delta_px);
    json += ",\"v5_v6_height_delta_px\":" + std::to_string(raw_measurement.v5_v6_height_delta_px);
    json += ",\"v5_z_mm\":" + std::to_string(raw_measurement.v5_z_mm);
    json += ",\"v6_z_mm\":" + std::to_string(raw_measurement.v6_z_mm);
    json += ",\"v61_z_mm\":" + std::to_string(raw_measurement.v61_z_mm);
    json += ",\"edge_top_rms_px\":" + std::to_string(raw_measurement.edge_top_rms_px);
    json += ",\"edge_right_rms_px\":" + std::to_string(raw_measurement.edge_right_rms_px);
    json += ",\"edge_bottom_rms_px\":" + std::to_string(raw_measurement.edge_bottom_rms_px);
    json += ",\"edge_left_rms_px\":" + std::to_string(raw_measurement.edge_left_rms_px);
    json += ",\"edge_top_gradient\":" + std::to_string(raw_measurement.edge_top_gradient);
    json += ",\"edge_right_gradient\":" + std::to_string(raw_measurement.edge_right_gradient);
    json += ",\"edge_bottom_gradient\":" + std::to_string(raw_measurement.edge_bottom_gradient);
    json += ",\"edge_left_gradient\":" + std::to_string(raw_measurement.edge_left_gradient);
    json += "}";

    json += ",\"width_distance_weight\":" + std::to_string(raw_measurement.width_distance_weight);
    json += ",\"height_distance_weight\":" + std::to_string(raw_measurement.height_distance_weight);
    json += ",\"distance_mm\":" + std::to_string(raw_measurement.distance_mm);
    json += ",\"x_mm\":" + std::to_string(raw_measurement.x_mm);
    json += ",\"y_mm\":" + std::to_string(raw_measurement.y_mm);
    json += ",\"z_mm\":" + std::to_string(raw_measurement.z_mm);
    json += ",\"z_from_width_mm\":" + std::to_string(raw_measurement.z_from_width_mm);
    json += ",\"z_from_height_mm\":" + std::to_string(raw_measurement.z_from_height_mm);
    json += ",\"target_yaw_deg\":" + std::to_string(raw_measurement.yaw_deg);
    json += ",\"target_pitch_deg\":" + std::to_string(raw_measurement.pitch_deg);
    json += ",\"target_roll_deg\":" + std::to_string(raw_measurement.roll_deg);
    json += ",\"pose_z_mm\":" + std::to_string(raw_measurement.pose_z_mm);
    json += ",\"pose_method\":\"";
    json += raw_measurement.pose_v4_used ? "pattern_direct_v4" :
            (raw_measurement.pose_v3_used ? "pattern_v3" :
             (raw_measurement.pose_v2_used ? "line_v2" :
              (raw_measurement.pose_v1_valid ? "homography_v1" : "none")));
    json += "\"";
    json += ",\"pose_v1_valid\":";
    json += raw_measurement.pose_v1_valid ? "true" : "false";
    json += ",\"pose_v2_valid\":";
    json += raw_measurement.pose_v2_valid ? "true" : "false";
    json += ",\"pose_v2_used\":";
    json += raw_measurement.pose_v2_used ? "true" : "false";
    json += ",\"pose_v3_valid\":";
    json += raw_measurement.pose_v3_valid ? "true" : "false";
    json += ",\"pose_v3_used\":";
    json += raw_measurement.pose_v3_used ? "true" : "false";
    json += ",\"pose_v4_valid\":";
    json += raw_measurement.pose_v4_valid ? "true" : "false";
    json += ",\"pose_v4_used\":";
    json += raw_measurement.pose_v4_used ? "true" : "false";
    json += ",\"pose_v1_yaw_deg\":" + std::to_string(raw_measurement.pose_v1_yaw_deg);
    json += ",\"pose_v1_pitch_deg\":" + std::to_string(raw_measurement.pose_v1_pitch_deg);
    json += ",\"pose_v1_roll_deg\":" + std::to_string(raw_measurement.pose_v1_roll_deg);
    json += ",\"pose_v2_yaw_deg\":" + std::to_string(raw_measurement.pose_v2_yaw_deg);
    json += ",\"pose_v2_pitch_deg\":" + std::to_string(raw_measurement.pose_v2_pitch_deg);
    json += ",\"pose_v2_roll_deg\":" + std::to_string(raw_measurement.pose_v2_roll_deg);
    json += ",\"pose_v2_line_rms_px\":" + std::to_string(raw_measurement.pose_v2_line_rms_px);
    json += ",\"pose_v2_corner_rms_px\":" + std::to_string(raw_measurement.pose_v2_corner_rms_px);
    json += ",\"pose_v3_yaw_deg\":" + std::to_string(raw_measurement.pose_v3_yaw_deg);
    json += ",\"pose_v3_pitch_deg\":" + std::to_string(raw_measurement.pose_v3_pitch_deg);
    json += ",\"pose_v3_roll_deg\":" + std::to_string(raw_measurement.pose_v3_roll_deg);
    json += ",\"pose_v3_pattern_rms_px\":" + std::to_string(raw_measurement.pose_v3_pattern_rms_px);
    json += ",\"pose_v3_fit_rms_px\":" + std::to_string(raw_measurement.pose_v3_fit_rms_px);
    json += ",\"pose_v3_feature_count\":" + std::to_string(raw_measurement.pose_v3_feature_count);
    json += ",\"pose_v3_inlier_count\":" + std::to_string(raw_measurement.pose_v3_inlier_count);
    json += ",\"pose_v4_yaw_deg\":" + std::to_string(raw_measurement.pose_v4_yaw_deg);
    json += ",\"pose_v4_pitch_deg\":" + std::to_string(raw_measurement.pose_v4_pitch_deg);
    json += ",\"pose_v4_roll_deg\":" + std::to_string(raw_measurement.pose_v4_roll_deg);
    json += ",\"pose_v4_rms_px\":" + std::to_string(raw_measurement.pose_v4_rms_px);
    json += ",\"pose_v4_max_residual_px\":" + std::to_string(raw_measurement.pose_v4_max_residual_px);
    json += ",\"pose_v4_feature_count\":" + std::to_string(raw_measurement.pose_v4_feature_count);
    json += ",\"pose_v4_inlier_count\":" + std::to_string(raw_measurement.pose_v4_inlier_count);
    json += ",\"quality\":" + std::to_string(raw_measurement.quality);
    json += ",\"timestamp_ms\":" + std::to_string(raw_measurement.timestamp_ms);
    json += "}";
  }

  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
