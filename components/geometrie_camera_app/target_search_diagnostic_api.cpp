#include "target_search_diagnostic_api.h"

#include <cstdio>

#include "grayscale_diagnostic.h"
#include "target_search_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

TargetSearchDiagnosticApiHandler::TargetSearchDiagnosticApiHandler(TargetSearchDiagnostic *diagnostic,
                                                                   GrayscaleDiagnostic *visualization)
    : diagnostic_(diagnostic), visualization_(visualization) {}

bool TargetSearchDiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/target/search" || url == "/target/status" || url == "/target/image.bmp";
}

void TargetSearchDiagnosticApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/target/search") {
    this->handle_search_(request);
    return;
  }
  if (url == "/target/status") {
    this->handle_status_(request);
    return;
  }
  if (url == "/target/image.bmp") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void TargetSearchDiagnosticApiHandler::handle_search_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"accepted\":false,\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  if (!this->diagnostic_->request_search()) {
    request->send(503, "application/json", "{\"accepted\":false,\"error\":\"search_busy_or_camera_unavailable\"}");
    return;
  }

  request->send(202, "application/json", "{\"accepted\":true,\"status\":\"target_search_requested\"}");
}

void TargetSearchDiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  const auto &observation = this->diagnostic_->last_observation();
  char json[1024];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\",\"ready\":%s,\"search_pending\":%s,\"search_count\":%u,"
      "\"target_found\":%s,\"target\":{\"center_x_px\":%.2f,\"center_y_px\":%.2f,"
      "\"width_px\":%.2f,\"height_px\":%.2f,\"rotation_deg\":%.2f,\"quality\":%.3f},"
      "\"timing\":{\"request_started_ms\":%u,\"frame_received_ms\":%u,\"acquisition_ms\":%u,"
      "\"detection_ms\":%u,\"visualization_ms\":%u,\"total_cycle_ms\":%u},"
      "\"image\":\"/target/image.bmp\"}",
      this->diagnostic_->ready() ? "true" : "false",
      this->diagnostic_->search_pending() ? "true" : "false",
      static_cast<unsigned>(this->diagnostic_->search_count()),
      this->diagnostic_->target_found() ? "true" : "false",
      static_cast<double>(observation.center_x_px),
      static_cast<double>(observation.center_y_px),
      static_cast<double>(observation.width_px),
      static_cast<double>(observation.height_px),
      static_cast<double>(observation.rotation_deg),
      static_cast<double>(observation.quality),
      static_cast<unsigned>(this->diagnostic_->request_started_ms()),
      static_cast<unsigned>(this->diagnostic_->frame_received_ms()),
      static_cast<unsigned>(this->diagnostic_->acquisition_ms()),
      static_cast<unsigned>(this->diagnostic_->detection_ms()),
      static_cast<unsigned>(this->diagnostic_->visualization_ms()),
      static_cast<unsigned>(this->diagnostic_->total_cycle_ms()));

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void TargetSearchDiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) {
  if (this->visualization_ == nullptr || !this->visualization_->ready() ||
      this->visualization_->bmp_data() == nullptr || this->visualization_->bmp_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"target_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp", this->visualization_->bmp_data(),
                                          this->visualization_->bmp_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
