#include "grayscale_diagnostic_api.h"

#include <cstdio>

#include "grayscale_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

GrayscaleDiagnosticApiHandler::GrayscaleDiagnosticApiHandler(GrayscaleDiagnostic *diagnostic)
    : diagnostic_(diagnostic) {}

bool GrayscaleDiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  return url == "/diagnostic/status" || url == "/diagnostic/capture" || url == "/diagnostic/raw.bmp";
}

void GrayscaleDiagnosticApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/diagnostic/status") {
    this->handle_status_(request);
    return;
  }

  if (url == "/diagnostic/capture") {
    this->handle_capture_(request);
    return;
  }

  if (url == "/diagnostic/raw.bmp") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void GrayscaleDiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  char json[1024];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\",\"mode\":\"grayscale_raw\",\"ready\":%s,\"capture_pending\":%s,"
      "\"capture_count\":%u,\"last_capture_ms\":%u,\"width\":%u,\"height\":%u,"
      "\"bmp_size\":%u,"
      "\"timing\":{\"request_started_ms\":%u,\"frame_received_ms\":%u,"
      "\"acquisition_ms\":%u,\"diagnostic_processing_ms\":%u,\"total_cycle_ms\":%u},"
      "\"raw_stats\":{\"pixel_count\":%u,\"min\":%u,\"max\":%u,"
      "\"mean\":%.3f,\"zero_count\":%u,\"full_255_count\":%u},"
      "\"image\":\"/diagnostic/raw.bmp\"}",
      this->diagnostic_->ready() ? "true" : "false",
      this->diagnostic_->capture_pending() ? "true" : "false",
      static_cast<unsigned>(this->diagnostic_->capture_count()),
      static_cast<unsigned>(this->diagnostic_->last_capture_ms()),
      static_cast<unsigned>(this->diagnostic_->width()),
      static_cast<unsigned>(this->diagnostic_->height()),
      static_cast<unsigned>(this->diagnostic_->bmp_size()),
      static_cast<unsigned>(this->diagnostic_->request_started_ms()),
      static_cast<unsigned>(this->diagnostic_->frame_received_ms()),
      static_cast<unsigned>(this->diagnostic_->acquisition_ms()),
      static_cast<unsigned>(this->diagnostic_->diagnostic_processing_ms()),
      static_cast<unsigned>(this->diagnostic_->total_cycle_ms()),
      static_cast<unsigned>(this->diagnostic_->raw_pixel_count()),
      static_cast<unsigned>(this->diagnostic_->raw_min()),
      static_cast<unsigned>(this->diagnostic_->raw_max()),
      static_cast<double>(this->diagnostic_->raw_mean()),
      static_cast<unsigned>(this->diagnostic_->raw_zero_count()),
      static_cast<unsigned>(this->diagnostic_->raw_full_count()));

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void GrayscaleDiagnosticApiHandler::handle_capture_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"accepted\":false,\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  if (!this->diagnostic_->request_capture()) {
    request->send(503, "application/json", "{\"accepted\":false,\"error\":\"capture_busy_or_camera_unavailable\"}");
    return;
  }

  request->send(202, "application/json", "{\"accepted\":true,\"status\":\"capture_requested\"}");
}

void GrayscaleDiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr || !this->diagnostic_->ready() || this->diagnostic_->bmp_data() == nullptr ||
      this->diagnostic_->bmp_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"raw_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp", this->diagnostic_->bmp_data(), this->diagnostic_->bmp_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
