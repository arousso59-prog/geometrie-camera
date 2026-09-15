#include "rgb565_diagnostic_api.h"

#include <cstdio>

#include "rgb565_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

Rgb565DiagnosticApiHandler::Rgb565DiagnosticApiHandler(Rgb565Diagnostic *diagnostic)
    : diagnostic_(diagnostic) {}

bool Rgb565DiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  return url == "/diagnostic-rgb565/status" || url == "/diagnostic-rgb565/capture" ||
         url == "/diagnostic-rgb565/raw.bmp";
}

void Rgb565DiagnosticApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/diagnostic-rgb565/status") {
    this->handle_status_(request);
    return;
  }

  if (url == "/diagnostic-rgb565/capture") {
    this->handle_capture_(request);
    return;
  }

  if (url == "/diagnostic-rgb565/raw.bmp") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void Rgb565DiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  char json[512];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\",\"mode\":\"rgb565_raw\",\"ready\":%s,\"capture_pending\":%s,"
      "\"capture_count\":%u,\"last_capture_ms\":%u,\"width\":%u,\"height\":%u,"
      "\"source_size\":%u,\"bmp_size\":%u,\"image\":\"/diagnostic-rgb565/raw.bmp\"}",
      this->diagnostic_->ready() ? "true" : "false",
      this->diagnostic_->capture_pending() ? "true" : "false",
      static_cast<unsigned>(this->diagnostic_->capture_count()),
      static_cast<unsigned>(this->diagnostic_->last_capture_ms()),
      static_cast<unsigned>(this->diagnostic_->width()),
      static_cast<unsigned>(this->diagnostic_->height()),
      static_cast<unsigned>(this->diagnostic_->source_size()),
      static_cast<unsigned>(this->diagnostic_->bmp_size()));

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void Rgb565DiagnosticApiHandler::handle_capture_(AsyncWebServerRequest *request) {
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

void Rgb565DiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) {
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
