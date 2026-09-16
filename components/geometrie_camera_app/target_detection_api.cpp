#include "target_detection_api.h"

#include <string>

#include "target_detection_service.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetectionApiHandler::TargetDetectionApiHandler(TargetDetectionService *service) : service_(service) {}

bool TargetDetectionApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/target/detect" || url == "/target/status";
}

void TargetDetectionApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/target/detect") {
    this->handle_detect_(request);
    return;
  }
  if (url == "/target/status") {
    this->handle_status_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void TargetDetectionApiHandler::handle_detect_(AsyncWebServerRequest *request) {
  if (this->service_ == nullptr) {
    this->send_status_(request, 500, "error", "target_detection_unavailable");
    return;
  }

  if (!this->service_->detect()) {
    this->send_status_(request, 409, "error", "filtered_image_unavailable");
    return;
  }

  this->send_status_(request, 200, "ok");
}

void TargetDetectionApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->service_ == nullptr) {
    this->send_status_(request, 500, "error", "target_detection_unavailable");
    return;
  }

  this->send_status_(request, 200, "ok");
}

void TargetDetectionApiHandler::send_status_(AsyncWebServerRequest *request, int response_code,
                                             const char *status, const char *error) const {
  std::string json;
  json.reserve(640);
  json += "{\"status\":\"";
  json += status;
  json += "\"";

  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }

  if (this->service_ != nullptr) {
    const TargetObservation &observation = this->service_->last_observation();
    json += ",\"ready\":";
    json += this->service_->ready() ? "true" : "false";
    json += ",\"target_found\":";
    json += this->service_->target_found() ? "true" : "false";
    json += ",\"detection_count\":";
    json += std::to_string(this->service_->detection_count());
    json += ",\"source_process_count\":";
    json += std::to_string(this->service_->source_process_count());
    json += ",\"detection_ms\":";
    json += std::to_string(this->service_->detection_ms());
    json += ",\"target\":{\"center_x_px\":";
    json += std::to_string(observation.center_x_px);
    json += ",\"center_y_px\":";
    json += std::to_string(observation.center_y_px);
    json += ",\"width_px\":";
    json += std::to_string(observation.width_px);
    json += ",\"height_px\":";
    json += std::to_string(observation.height_px);
    json += ",\"rotation_deg\":";
    json += std::to_string(observation.rotation_deg);
    json += ",\"quality\":";
    json += std::to_string(observation.quality);
    json += "}";
  }

  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
