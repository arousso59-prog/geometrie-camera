#include "target_tracking_api.h"

#include "camera_viewport_controller.h"
#include "target_tracking_controller.h"

namespace esphome {
namespace geometrie_camera_app {

TargetTrackingApiHandler::TargetTrackingApiHandler(
    TargetTrackingController *tracking_controller,
    CameraViewportController *viewport_controller)
    : tracking_controller_(tracking_controller),
      viewport_controller_(viewport_controller) {}

bool TargetTrackingApiHandler::canHandle(
    AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/tracking/status" ||
         url == "/api/camera/viewport";
}

void TargetTrackingApiHandler::handleRequest(
    AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  if (url == "/tracking/status") {
    this->handle_status_(request);
  } else if (url == "/api/camera/viewport") {
    this->handle_viewport_(request);
  } else {
    request->send(404, "application/json",
                  "{\"error\":\"not_found\"}");
  }
}

void TargetTrackingApiHandler::handle_status_(
    AsyncWebServerRequest *request) const {
  if (this->tracking_controller_ == nullptr) {
    request->send(
        500, "application/json",
        "{\"status\":\"error\",\"error\":\"tracking_unavailable\"}");
    return;
  }

  std::string json;
  json.reserve(900);
  json += "{\"status\":\"ok\"";
  json += ",\"permanent\":true";
  json += ",\"active\":";
  json += this->tracking_controller_->active() ? "true" : "false";
  json += ",\"supported\":";
  json += this->tracking_controller_->supported() ? "true" : "false";
  json += ",\"mode\":\"";
  json += this->tracking_controller_->mode_text();
  json += "\"";
  json += ",\"target_locked\":";
  json += this->tracking_controller_->target_locked() ? "true" : "false";
  json += ",\"lost_count\":" +
          std::to_string(
              this->tracking_controller_->current_lost_count());
  json += ",\"transition_count\":" +
          std::to_string(
              this->tracking_controller_->transition_count());
  json += ",\"last_error\":\"" +
          this->tracking_controller_->last_error() + "\"";
  json += ",\"viewport\":";
  this->append_viewport_json_(json);
  json += "}";

  auto *response =
      request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void TargetTrackingApiHandler::handle_viewport_(
    AsyncWebServerRequest *request) const {
  if (this->viewport_controller_ == nullptr) {
    request->send(
        500, "application/json",
        "{\"status\":\"error\",\"error\":\"viewport_unavailable\"}");
    return;
  }

  std::string json = "{\"status\":\"ok\",\"tracking_permanent\":true";
  if (this->tracking_controller_ != nullptr) {
    json += ",\"tracking_active\":";
    json += this->tracking_controller_->active() ? "true" : "false";
  }
  json += ",\"viewport\":";
  this->append_viewport_json_(json);
  json += "}";

  auto *response =
      request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void TargetTrackingApiHandler::append_viewport_json_(
    std::string &json) const {
  if (this->viewport_controller_ == nullptr) {
    json += "null";
    return;
  }

  const auto &viewport = this->viewport_controller_->snapshot();
  json += "{";
  json += "\"supported\":";
  json += this->viewport_controller_->supports_precise_roi()
              ? "true"
              : "false";
  json += ",\"mode\":\"";
  json += CameraViewportController::mode_text(viewport.mode);
  json += "\"";
  json += ",\"reference\":{\"width\":" +
          std::to_string(viewport.reference_width) +
          ",\"height\":" +
          std::to_string(viewport.reference_height) + "}";
  json += ",\"window\":{\"x\":" +
          std::to_string(viewport.window_x) +
          ",\"y\":" + std::to_string(viewport.window_y) +
          ",\"width\":" +
          std::to_string(viewport.window_width) +
          ",\"height\":" +
          std::to_string(viewport.window_height) + "}";
  json += ",\"sensor_window\":{\"x\":" +
          std::to_string(viewport.sensor_window_x) +
          ",\"y\":" +
          std::to_string(viewport.sensor_window_y) +
          ",\"width\":" +
          std::to_string(viewport.window_width) +
          ",\"height\":" +
          std::to_string(viewport.window_height) + "}";
  json += ",\"orientation\":{\"horizontal_mirror\":";
  json += viewport.horizontal_mirror ? "true" : "false";
  json += ",\"vertical_flip\":";
  json += viewport.vertical_flip ? "true" : "false";
  json += "}";
  json += ",\"output\":{\"width\":" +
          std::to_string(viewport.output_width) +
          ",\"height\":" +
          std::to_string(viewport.output_height) + "}";
  json += ",\"scale\":{\"x\":" +
          std::to_string(viewport.scale_x) +
          ",\"y\":" +
          std::to_string(viewport.scale_y) + "}";
  json += "}";
}

}  // namespace geometrie_camera_app
}  // namespace esphome
