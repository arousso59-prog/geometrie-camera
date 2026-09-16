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
  bool has_target_size = false;
  std::string error;
  if (!this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error)) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }

  if (!has_target_size) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"no_setting_provided\"}");
    return;
  }

  if (target_size_mm < 1.0f || target_size_mm > 1000.0f ||
      !this->manager_->measurement_engine().set_target_size_mm(target_size_mm)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"target_size_mm_out_of_range\"}");
    return;
  }

  // La focale issue d'une calibration a distance connue depend de la taille
  // physique de cible utilisee pour le calcul. Un changement de taille invalide
  // donc volontairement la calibration precedente.
  this->manager_->measurement_engine().clear_calibration();
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
  bool has_distance = false;
  bool has_target_size = false;
  std::string error;
  if (!this->parse_float_param_(request, "distance_mm", distance_mm, has_distance, error) ||
      !this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error)) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }

  if (!has_distance || distance_mm < 50.0f || distance_mm > 20000.0f) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"distance_mm_out_of_range\"}");
    return;
  }

  if (has_target_size) {
    if (target_size_mm < 1.0f || target_size_mm > 1000.0f ||
        !this->manager_->measurement_engine().set_target_size_mm(target_size_mm)) {
      request->send(400, "application/json",
                    "{\"status\":\"error\",\"error\":\"target_size_mm_out_of_range\"}");
      return;
    }
  }

  if (!this->detection_is_current_()) {
    this->send_snapshot_(request, 409, "error", "current_target_detection_required");
    return;
  }
  if (!this->detection_service_->target_found()) {
    this->send_snapshot_(request, 409, "error", "target_not_found");
    return;
  }

  GeometryMeasurementEngine &engine = this->manager_->measurement_engine();
  if (!engine.calibrate_from_known_distance(this->detection_service_->last_observation(),
                                            this->source_->width(), this->source_->height(),
                                            distance_mm)) {
    this->send_snapshot_(request, 500, "error", "calibration_failed");
    return;
  }

  this->manager_->reset();
  if (!this->manager_->process(this->detection_service_->last_observation(),
                               this->source_->width(), this->source_->height(), millis())) {
    this->send_snapshot_(request, 500, "error", "measurement_after_calibration_failed");
    return;
  }

  this->send_snapshot_(request, 200, "ok");
}

void MeasurementApiHandler::send_snapshot_(AsyncWebServerRequest *request, int response_code,
                                           const char *status, const char *error) const {
  std::string json;
  json.reserve(1400);
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

    json += ",\"calibration\":{\"valid\":";
    json += engine.has_calibration() ? "true" : "false";
    json += ",\"fx_px\":" + std::to_string(stored.fx_px);
    json += ",\"fy_px\":" + std::to_string(stored.fy_px);
    json += ",\"cx_px\":" + std::to_string(stored.cx_px);
    json += ",\"cy_px\":" + std::to_string(stored.cy_px);
    json += ",\"reference_width_px\":" + std::to_string(stored.reference_width_px);
    json += ",\"reference_height_px\":" + std::to_string(stored.reference_height_px);
    json += ",\"effective_fx_px\":" + std::to_string(effective.fx_px);
    json += ",\"effective_fy_px\":" + std::to_string(effective.fy_px);
    json += ",\"effective_cx_px\":" + std::to_string(effective.cx_px);
    json += ",\"effective_cy_px\":" + std::to_string(effective.cy_px);
    json += "}";

    json += ",\"measurement\":{\"valid\":";
    json += measurement.valid ? "true" : "false";
    json += ",\"distance_mm\":" + std::to_string(measurement.distance_mm);
    json += ",\"x_mm\":" + std::to_string(measurement.x_mm);
    json += ",\"y_mm\":" + std::to_string(measurement.y_mm);
    json += ",\"z_mm\":" + std::to_string(measurement.z_mm);
    json += ",\"bearing_yaw_deg\":" + std::to_string(measurement.bearing_yaw_deg);
    json += ",\"bearing_pitch_deg\":" + std::to_string(measurement.bearing_pitch_deg);
    json += ",\"target_yaw_deg\":" + std::to_string(measurement.yaw_deg);
    json += ",\"target_pitch_deg\":" + std::to_string(measurement.pitch_deg);
    json += ",\"target_roll_deg\":" + std::to_string(measurement.roll_deg);
    json += ",\"quality\":" + std::to_string(measurement.quality);
    json += ",\"timestamp_ms\":" + std::to_string(measurement.timestamp_ms);
    json += "}";
  }

  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
