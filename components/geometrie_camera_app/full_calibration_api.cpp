#include "full_calibration_api.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>

#include "full_calibration_controller.h"

namespace esphome {
namespace geometrie_camera_app {

FullCalibrationApiHandler::FullCalibrationApiHandler(FullCalibrationController *controller)
    : controller_(controller) {}

bool FullCalibrationApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/calibration/full/start" ||
         url == "/calibration/full/status" ||
         url == "/calibration/full/cancel";
}

void FullCalibrationApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (this->controller_ == nullptr) {
    request->send(500, "application/json",
                  "{\"status\":\"error\",\"error\":\"calibration_controller_unavailable\"}");
    return;
  }

  if (url == "/calibration/full/status") {
    this->send_status_(request, 200, "ok");
    return;
  }

  if (url == "/calibration/full/cancel") {
    this->controller_->cancel();
    this->send_status_(request, 200, "ok");
    return;
  }

  if (url == "/calibration/full/start") {
    float distance_mm = 0.0f;
    float target_size_mm = 0.0f;
    int sample_count = FullCalibrationController::DEFAULT_SAMPLE_COUNT;
    int force_value = 0;
    bool has_distance = false;
    bool has_target_size = false;
    bool has_sample_count = false;
    bool has_force = false;
    std::string error;

    if (!this->parse_float_param_(request, "distance_mm", distance_mm, has_distance, error) ||
        !this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error) ||
        !this->parse_int_param_(request, "samples", sample_count, has_sample_count, error) ||
        !this->parse_int_param_(request, "force", force_value, has_force, error)) {
      const std::string body =
          std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
      request->send(400, "application/json", body.c_str());
      return;
    }

    if (!has_distance || !has_target_size) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"distance_mm_and_target_size_mm_required\"}");
      return;
    }
    if (has_force && force_value != 0 && force_value != 1) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"force_must_be_0_or_1\"}");
      return;
    }
    if (sample_count < 3 || sample_count > FullCalibrationController::MAX_SAMPLE_COUNT) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"sample_count_out_of_range\"}");
      return;
    }

    if (!this->controller_->start(
            distance_mm,
            target_size_mm,
            static_cast<uint8_t>(sample_count),
            has_force && force_value == 1)) {
      this->send_status_(request, 409, "error");
      return;
    }

    this->send_status_(request, 202, "accepted");
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool FullCalibrationApiHandler::parse_float_param_(
    AsyncWebServerRequest *request, const char *name,
    float &value, bool &present, std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;

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

bool FullCalibrationApiHandler::parse_int_param_(
    AsyncWebServerRequest *request, const char *name,
    int &value, bool &present, std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;

  const std::string text = request->getParam(name)->value();
  if (text.empty()) {
    error = std::string(name) + "_invalid";
    return false;
  }

  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0') {
    error = std::string(name) + "_invalid";
    return false;
  }

  value = static_cast<int>(parsed);
  return true;
}

void FullCalibrationApiHandler::send_status_(
    AsyncWebServerRequest *request, int response_code, const char *status) const {
  const CameraCalibration &calibration = this->controller_->result_calibration();

  std::string json;
  json.reserve(1600);
  json += "{\"status\":\"";
  json += status;
  json += "\"";
  json += ",\"state\":\"" + std::string(this->controller_->state_text()) + "\"";
  json += ",\"running\":";
  json += this->controller_->running() ? "true" : "false";
  json += ",\"acquisition_mode\":\"precise_native_800x600\"";
  json += ",\"reference_resolution\":\"2560x1920\"";
  json += ",\"requested_samples\":" + std::to_string(this->controller_->requested_samples());
  json += ",\"valid_samples\":" + std::to_string(this->controller_->valid_samples());
  json += ",\"attempts\":" + std::to_string(this->controller_->attempts());
  json += ",\"max_attempts\":" + std::to_string(this->controller_->max_attempts());
  json += ",\"known_distance_mm\":" + std::to_string(this->controller_->known_distance_mm());
  json += ",\"target_size_mm\":" + std::to_string(this->controller_->target_size_mm());

  const uint8_t requested = this->controller_->requested_samples();
  const uint8_t valid = this->controller_->valid_samples();
  const unsigned progress = requested > 0
                                ? static_cast<unsigned>(valid) * 100U / requested
                                : 0U;
  json += ",\"progress_pct\":" + std::to_string(progress);

  if (!this->controller_->last_error().empty()) {
    json += ",\"error\":\"" + this->controller_->last_error() + "\"";
  }

  json += ",\"result\":{\"valid\":";
  json += this->controller_->state() == FullCalibrationState::COMPLETE ? "true" : "false";
  json += ",\"mean_fx_px\":" + std::to_string(this->controller_->mean_fx_px());
  json += ",\"mean_fy_px\":" + std::to_string(this->controller_->mean_fy_px());
  json += ",\"stddev_fx_px\":" + std::to_string(this->controller_->stddev_fx_px());
  json += ",\"stddev_fy_px\":" + std::to_string(this->controller_->stddev_fy_px());
  json += ",\"calibration\":{";
  json += "\"fx_px\":" + std::to_string(calibration.fx_px);
  json += ",\"fy_px\":" + std::to_string(calibration.fy_px);
  json += ",\"cx_px\":" + std::to_string(calibration.cx_px);
  json += ",\"cy_px\":" + std::to_string(calibration.cy_px);
  json += ",\"k1\":" + std::to_string(calibration.k1);
  json += ",\"k2\":" + std::to_string(calibration.k2);
  json += ",\"p1\":" + std::to_string(calibration.p1);
  json += ",\"p2\":" + std::to_string(calibration.p2);
  json += ",\"k3\":" + std::to_string(calibration.k3);
  json += ",\"reference_width_px\":" + std::to_string(calibration.reference_width_px);
  json += ",\"reference_height_px\":" + std::to_string(calibration.reference_height_px);
  json += "}}}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
