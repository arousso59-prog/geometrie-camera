#include "continuous_measurement_api.h"

#include <cerrno>
#include <cstdlib>
#include <string>

#include "continuous_measurement_controller.h"

namespace esphome {
namespace geometrie_camera_app {

ContinuousMeasurementApiHandler::ContinuousMeasurementApiHandler(ContinuousMeasurementController *controller)
    : controller_(controller) {}

bool ContinuousMeasurementApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/continuous/start" || url == "/continuous/stop" || url == "/continuous/status";
}

void ContinuousMeasurementApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/continuous/start") {
    this->handle_start_(request);
    return;
  }
  if (url == "/continuous/stop") {
    this->handle_stop_(request);
    return;
  }
  if (url == "/continuous/status") {
    this->handle_status_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool ContinuousMeasurementApiHandler::parse_interval_(AsyncWebServerRequest *request,
                                                       uint32_t &interval_ms,
                                                       bool &present,
                                                       std::string &error) const {
  present = request->hasParam("interval_ms");
  if (!present) {
    return true;
  }

  const std::string text = request->getParam("interval_ms")->value();
  if (text.empty()) {
    error = "interval_ms_invalid";
    return false;
  }

  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' || parsed > 0xFFFFFFFFUL) {
    error = "interval_ms_invalid";
    return false;
  }

  interval_ms = static_cast<uint32_t>(parsed);
  return true;
}

void ContinuousMeasurementApiHandler::handle_start_(AsyncWebServerRequest *request) {
  if (this->controller_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "continuous_measurement_unavailable");
    return;
  }

  uint32_t interval_ms = this->controller_->interval_ms();
  bool has_interval = false;
  std::string error;
  if (!this->parse_interval_(request, interval_ms, has_interval, error)) {
    this->send_snapshot_(request, 400, "error", error.c_str());
    return;
  }

  if (interval_ms < 200 || interval_ms > 10000) {
    this->send_snapshot_(request, 400, "error", "interval_ms_out_of_range");
    return;
  }

  if (!this->controller_->start(interval_ms)) {
    const std::string &start_error = this->controller_->last_error();
    const int response_code = start_error == "calibration_required" ? 409 : 500;
    this->send_snapshot_(request, response_code, "error",
                         start_error.empty() ? "continuous_start_failed" : start_error.c_str());
    return;
  }

  this->send_snapshot_(request, 200, "ok");
}

void ContinuousMeasurementApiHandler::handle_stop_(AsyncWebServerRequest *request) {
  if (this->controller_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "continuous_measurement_unavailable");
    return;
  }

  this->controller_->stop();
  this->send_snapshot_(request, 200, "ok");
}

void ContinuousMeasurementApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->controller_ == nullptr) {
    this->send_snapshot_(request, 500, "error", "continuous_measurement_unavailable");
    return;
  }
  this->send_snapshot_(request, 200, "ok");
}

void ContinuousMeasurementApiHandler::send_snapshot_(AsyncWebServerRequest *request,
                                                       int response_code,
                                                       const char *status,
                                                       const char *error) const {
  std::string json;
  json.reserve(1550);
  json += "{\"status\":\"";
  json += status;
  json += "\"";

  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }

  if (this->controller_ != nullptr) {
    json += ",\"running\":";
    json += this->controller_->running() ? "true" : "false";
    json += ",\"state\":\"";
    json += this->controller_->state_text();
    json += "\"";
    json += ",\"interval_ms\":" + std::to_string(this->controller_->interval_ms());
    json += ",\"cycle_count\":" + std::to_string(this->controller_->cycle_count());
    json += ",\"target_found_count\":" + std::to_string(this->controller_->target_found_count());
    json += ",\"valid_measurement_count\":" + std::to_string(this->controller_->valid_measurement_count());
    json += ",\"target_found\":";
    json += this->controller_->last_cycle_target_found() ? "true" : "false";
    json += ",\"measurement_valid\":";
    json += this->controller_->last_cycle_measurement_valid() ? "true" : "false";

    const uint32_t capture_ms = this->controller_->last_capture_ms();
    const uint32_t sharpness_ms = this->controller_->last_sharpness_ms();
    const uint32_t filter_ms = this->controller_->last_filter_ms();
    const uint32_t detect_ms = this->controller_->last_detect_ms();
    const uint32_t compute_ms = this->controller_->last_compute_ms();
    const uint32_t cycle_ms = this->controller_->last_cycle_ms();
    const uint64_t processing_sum = static_cast<uint64_t>(capture_ms) + sharpness_ms + filter_ms +
                                    detect_ms + compute_ms;
    const uint32_t processing_ms = processing_sum > 0xFFFFFFFFULL
                                       ? 0xFFFFFFFFU
                                       : static_cast<uint32_t>(processing_sum);
    const uint32_t orchestration_ms = cycle_ms > processing_ms ? cycle_ms - processing_ms : 0U;

    json += ",\"timing\":{";
    json += "\"capture_ms\":" + std::to_string(capture_ms);
    json += ",\"sharpness_ms\":" + std::to_string(sharpness_ms);
    json += ",\"filter_ms\":" + std::to_string(filter_ms);
    json += ",\"detect_ms\":" + std::to_string(detect_ms);
    json += ",\"compute_ms\":" + std::to_string(compute_ms);
    json += ",\"processing_ms\":" + std::to_string(processing_ms);
    json += ",\"orchestration_ms\":" + std::to_string(orchestration_ms);
    json += ",\"cycle_ms\":" + std::to_string(cycle_ms);
    json += "}";

    json += ",\"sharpness\":{";
    json += "\"score_x100\":" + std::to_string(this->controller_->last_sharpness_score_x100());
    json += ",\"reference_x100\":" + std::to_string(this->controller_->sharpness_reference_score_x100());
    json += ",\"ok\":";
    json += this->controller_->last_sharpness_ok() ? "true" : "false";
    json += ",\"capture_retries\":" + std::to_string(this->controller_->last_capture_retry_count());
    json += ",\"blur_retry_count\":" + std::to_string(this->controller_->blur_retry_count());
    json += ",\"roi_active\":";
    json += this->controller_->sharpness_roi_valid() ? "true" : "false";
    json += ",\"roi_x\":" + std::to_string(this->controller_->sharpness_roi_x());
    json += ",\"roi_y\":" + std::to_string(this->controller_->sharpness_roi_y());
    json += ",\"roi_width\":" + std::to_string(this->controller_->sharpness_roi_width());
    json += ",\"roi_height\":" + std::to_string(this->controller_->sharpness_roi_height());
    json += "}";

    json += ",\"last_error\":\"";
    json += this->controller_->last_error();
    json += "\"";
  }

  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
