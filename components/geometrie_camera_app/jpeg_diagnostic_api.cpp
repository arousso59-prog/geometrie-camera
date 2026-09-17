#include "jpeg_diagnostic_api.h"

#include <cstdio>
#include <string>

#include "camera_resolution_controller.h"
#include "jpeg_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

JpegDiagnosticApiHandler::JpegDiagnosticApiHandler(JpegDiagnostic *diagnostic,
                                                   CameraResolutionController *resolution_controller)
    : diagnostic_(diagnostic), resolution_controller_(resolution_controller) {}

bool JpegDiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  return url == "/diagnostic-jpeg/status" || url == "/diagnostic-jpeg/capture" ||
         url == "/diagnostic-jpeg/image.jpg";
}

void JpegDiagnosticApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/diagnostic-jpeg/status") {
    this->handle_status_(request);
    return;
  }

  if (url == "/diagnostic-jpeg/capture") {
    this->handle_capture_(request);
    return;
  }

  if (url == "/diagnostic-jpeg/image.jpg") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool JpegDiagnosticApiHandler::apply_requested_resolution_(AsyncWebServerRequest *request) {
  if (this->resolution_controller_ == nullptr || !request->hasParam("resolution")) {
    return true;
  }

  const std::string requested = request->getParam("resolution")->value();
  if (!this->resolution_controller_->is_supported(requested)) {
    char json[448];
    std::snprintf(json, sizeof(json),
                  "{\"accepted\":false,\"error\":\"unsupported_resolution\",\"requested\":\"%s\","
                  "\"allowed\":\"%s\"}",
                  requested.c_str(), CameraResolutionController::allowed_resolutions_text());
    request->send(400, "application/json", json);
    return false;
  }

  if (!this->resolution_controller_->apply(requested)) {
    request->send(500, "application/json",
                  "{\"accepted\":false,\"error\":\"resolution_apply_failed\"}");
    return false;
  }

  return true;
}

void JpegDiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  const char *active_resolution = this->resolution_controller_ != nullptr
                                      ? this->resolution_controller_->active_resolution().c_str()
                                      : "unknown";

  char json[1024];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\",\"mode\":\"jpeg_sensor\",\"ready\":%s,\"capture_pending\":%s,"
      "\"capture_count\":%u,\"discarded_frame_count\":%u,\"last_capture_ms\":%u,"
      "\"active_resolution\":\"%s\",\"width\":%u,\"height\":%u,\"jpeg_size\":%u,"
      "\"markers\":{\"soi\":%s,\"eoi\":%s},"
      "\"timing\":{\"request_started_ms\":%u,\"stale_frame_received_ms\":%u,"
      "\"fresh_request_started_ms\":%u,\"frame_received_ms\":%u,\"acquisition_ms\":%u,"
      "\"copy_ms\":%u,\"total_cycle_ms\":%u},\"image\":\"/diagnostic-jpeg/image.jpg\"}",
      this->diagnostic_->ready() ? "true" : "false",
      this->diagnostic_->capture_pending() ? "true" : "false",
      static_cast<unsigned>(this->diagnostic_->capture_count()),
      static_cast<unsigned>(this->diagnostic_->discarded_frame_count()),
      static_cast<unsigned>(this->diagnostic_->last_capture_ms()),
      active_resolution,
      static_cast<unsigned>(this->diagnostic_->width()),
      static_cast<unsigned>(this->diagnostic_->height()),
      static_cast<unsigned>(this->diagnostic_->jpeg_size()),
      this->diagnostic_->has_soi() ? "true" : "false",
      this->diagnostic_->has_eoi() ? "true" : "false",
      static_cast<unsigned>(this->diagnostic_->request_started_ms()),
      static_cast<unsigned>(this->diagnostic_->stale_frame_received_ms()),
      static_cast<unsigned>(this->diagnostic_->fresh_request_started_ms()),
      static_cast<unsigned>(this->diagnostic_->frame_received_ms()),
      static_cast<unsigned>(this->diagnostic_->acquisition_ms()),
      static_cast<unsigned>(this->diagnostic_->copy_ms()),
      static_cast<unsigned>(this->diagnostic_->total_cycle_ms()));

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void JpegDiagnosticApiHandler::handle_capture_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"accepted\":false,\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  if (this->diagnostic_->capture_pending()) {
    request->send(503, "application/json", "{\"accepted\":false,\"error\":\"capture_busy\"}");
    return;
  }

  if (!this->apply_requested_resolution_(request)) {
    return;
  }

  if (!this->diagnostic_->request_capture()) {
    request->send(503, "application/json", "{\"accepted\":false,\"error\":\"camera_unavailable\"}");
    return;
  }

  char json[256];
  const char *active_resolution = this->resolution_controller_ != nullptr
                                      ? this->resolution_controller_->active_resolution().c_str()
                                      : "unknown";
  std::snprintf(json, sizeof(json),
                "{\"accepted\":true,\"status\":\"capture_requested\",\"resolution\":\"%s\","
                "\"fresh_frame\":true}",
                active_resolution);
  request->send(200, "application/json", json);
}

void JpegDiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr || !this->diagnostic_->ready() || this->diagnostic_->jpeg_data() == nullptr ||
      this->diagnostic_->jpeg_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"jpeg_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/jpeg", this->diagnostic_->jpeg_data(),
                                          this->diagnostic_->jpeg_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
