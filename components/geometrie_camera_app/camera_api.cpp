#include "camera_api.h"

#include <cstdio>

#include "geometrie_camera_app.h"
#include "placeholder_image.h"

namespace esphome {
namespace geometrie_camera_app {

bool CameraApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  return url == "/api/status" || url == "/api/capture" || url == "/api/measure" || url == "/image.jpg";
}

void CameraApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/api/status") {
    this->handle_status_(request);
    return;
  }
  if (url == "/api/capture") {
    this->handle_capture_(request);
    return;
  }
  if (url == "/api/measure") {
    this->handle_measure_(request);
    return;
  }
  if (url == "/image.jpg") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void CameraApiHandler::handle_status_(AsyncWebServerRequest *request) {
  const auto &camera = this->parent_->camera_manager();
  const auto &frame = camera.last_frame_info();

  char json[640];
  std::snprintf(
      json, sizeof(json),
      "{\"api_version\":1,\"status\":\"ok\",\"mode\":\"placeholder\","
      "\"camera_ready\":%s,\"physical_camera_ready\":%s,\"capture_count\":%u,"
      "\"last_capture_ms\":%u,\"image\":{\"url\":\"/image.jpg\",\"format\":\"jpeg\","
      "\"width\":%u,\"height\":%u,\"size_bytes\":%u}}",
      camera.ready() ? "true" : "false", camera.physical_camera_ready() ? "true" : "false",
      static_cast<unsigned>(camera.capture_count()), static_cast<unsigned>(frame.timestamp_ms),
      static_cast<unsigned>(PLACEHOLDER_IMAGE_WIDTH), static_cast<unsigned>(PLACEHOLDER_IMAGE_HEIGHT),
      static_cast<unsigned>(PLACEHOLDER_IMAGE_JPEG_SIZE));

  this->send_json_(request, json);
}

void CameraApiHandler::handle_capture_(AsyncWebServerRequest *request) {
  auto &camera = this->parent_->camera_manager();
  const bool success = camera.request_capture();

  if (!success) {
    request->send(500, "application/json", "{\"success\":false,\"error\":\"capture_failed\"}");
    return;
  }

  const auto &frame = camera.last_frame_info();

  char json[512];
  std::snprintf(
      json, sizeof(json),
      "{\"success\":true,\"placeholder\":true,\"capture_id\":%u,\"timestamp_ms\":%u,"
      "\"width\":%u,\"height\":%u,\"size_bytes\":%u,\"image\":\"/image.jpg\"}",
      static_cast<unsigned>(camera.capture_count()), static_cast<unsigned>(frame.timestamp_ms),
      static_cast<unsigned>(frame.width), static_cast<unsigned>(frame.height),
      static_cast<unsigned>(frame.size_bytes));

  this->send_json_(request, json);
}

void CameraApiHandler::handle_measure_(AsyncWebServerRequest *request) {
  const auto &measurement = this->parent_->last_measurement();

  char json[512];
  std::snprintf(
      json, sizeof(json),
      "{\"valid\":%s,\"placeholder\":true,\"timestamp_ms\":%u,\"yaw_deg\":%.6f,"
      "\"pitch_deg\":%.6f,\"roll_deg\":%.6f,\"quality\":%.3f}",
      measurement.valid ? "true" : "false", static_cast<unsigned>(measurement.timestamp_ms),
      static_cast<double>(measurement.yaw_deg), static_cast<double>(measurement.pitch_deg),
      static_cast<double>(measurement.roll_deg), static_cast<double>(measurement.quality));

  this->send_json_(request, json);
}

void CameraApiHandler::handle_image_(AsyncWebServerRequest *request) {
  auto *response = request->beginResponse(200, "image/jpeg", PLACEHOLDER_IMAGE_JPEG, PLACEHOLDER_IMAGE_JPEG_SIZE);
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

void CameraApiHandler::send_json_(AsyncWebServerRequest *request, const std::string &json) const {
  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
