#include "grayscale_diagnostic_api.h"

#include <cstdio>
#include <string>

#include "camera_resolution_controller.h"
#include "grayscale_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

GrayscaleDiagnosticApiHandler::GrayscaleDiagnosticApiHandler(GrayscaleDiagnostic *diagnostic,
                                                             CameraResolutionController *resolution_controller)
    : diagnostic_(diagnostic), resolution_controller_(resolution_controller) {}

bool GrayscaleDiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  return url == "/diagnostic/status" || url == "/diagnostic/capture" || url == "/diagnostic/raw.bmp" ||
         url == "/diagnostic/preview.bmp";
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

  if (url == "/diagnostic/preview.bmp") {
    this->handle_preview_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool GrayscaleDiagnosticApiHandler::apply_requested_resolution_(AsyncWebServerRequest *request) {
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

void GrayscaleDiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"diagnostic_unavailable\"}");
    return;
  }

  if (this->resolution_controller_ != nullptr) {
    this->resolution_controller_->refresh_sensor_identity();
  }

  const char *active_resolution = this->resolution_controller_ != nullptr
                                      ? this->resolution_controller_->active_resolution().c_str()
                                      : "unknown";
  const char *sensor_name = this->resolution_controller_ != nullptr
                                ? this->resolution_controller_->sensor_name().c_str()
                                : "unknown";
  const char *sensor_max_resolution = this->resolution_controller_ != nullptr
                                          ? this->resolution_controller_->sensor_max_resolution().c_str()
                                          : "unknown";
  const unsigned sensor_pid = this->resolution_controller_ != nullptr
                                  ? static_cast<unsigned>(this->resolution_controller_->sensor_pid())
                                  : 0U;

  std::string json;
  json.reserve(1280);
  char chunk[384];

  std::snprintf(chunk, sizeof(chunk),
                "{\"status\":\"ok\",\"mode\":\"grayscale_raw\",\"ready\":%s,\"capture_pending\":%s,"
                "\"camera\":{\"sensor\":\"%s\",\"pid\":\"0x%04X\",\"max_resolution\":\"%s\"},"
                "\"capture_count\":%u,\"last_capture_ms\":%u,\"active_resolution\":\"%s\",",
                this->diagnostic_->ready() ? "true" : "false",
                this->diagnostic_->capture_pending() ? "true" : "false",
                sensor_name,
                sensor_pid,
                sensor_max_resolution,
                static_cast<unsigned>(this->diagnostic_->capture_count()),
                static_cast<unsigned>(this->diagnostic_->last_capture_ms()),
                active_resolution);
  json += chunk;

  std::snprintf(chunk, sizeof(chunk),
                "\"width\":%u,\"height\":%u,\"bmp_size\":%u,\"full_bmp_available\":%s,"
                "\"preview\":{\"available\":%s,\"width\":%u,\"height\":%u,\"bmp_size\":%u,"
                "\"image\":\"/diagnostic/preview.bmp\"},",
                static_cast<unsigned>(this->diagnostic_->width()),
                static_cast<unsigned>(this->diagnostic_->height()),
                static_cast<unsigned>(this->diagnostic_->bmp_size()),
                this->diagnostic_->bmp_data() != nullptr && this->diagnostic_->bmp_size() > 0 ? "true" : "false",
                this->diagnostic_->preview_ready() ? "true" : "false",
                static_cast<unsigned>(this->diagnostic_->preview_width()),
                static_cast<unsigned>(this->diagnostic_->preview_height()),
                static_cast<unsigned>(this->diagnostic_->preview_bmp_size()));
  json += chunk;

  std::snprintf(chunk, sizeof(chunk),
                "\"timing\":{\"request_started_ms\":%u,\"frame_received_ms\":%u,"
                "\"acquisition_ms\":%u,\"diagnostic_processing_ms\":%u,\"total_cycle_ms\":%u},",
                static_cast<unsigned>(this->diagnostic_->request_started_ms()),
                static_cast<unsigned>(this->diagnostic_->frame_received_ms()),
                static_cast<unsigned>(this->diagnostic_->acquisition_ms()),
                static_cast<unsigned>(this->diagnostic_->diagnostic_processing_ms()),
                static_cast<unsigned>(this->diagnostic_->total_cycle_ms()));
  json += chunk;

  std::snprintf(chunk, sizeof(chunk),
                "\"raw_stats\":{\"pixel_count\":%u,\"min\":%u,\"max\":%u,"
                "\"mean\":%.3f,\"zero_count\":%u,\"full_255_count\":%u},"
                "\"image\":\"/diagnostic/raw.bmp\"}",
                static_cast<unsigned>(this->diagnostic_->raw_pixel_count()),
                static_cast<unsigned>(this->diagnostic_->raw_min()),
                static_cast<unsigned>(this->diagnostic_->raw_max()),
                static_cast<double>(this->diagnostic_->raw_mean()),
                static_cast<unsigned>(this->diagnostic_->raw_zero_count()),
                static_cast<unsigned>(this->diagnostic_->raw_full_count()));
  json += chunk;

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void GrayscaleDiagnosticApiHandler::handle_capture_(AsyncWebServerRequest *request) {
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
                "{\"accepted\":true,\"status\":\"capture_requested\",\"resolution\":\"%s\"}",
                active_resolution);
  request->send(202, "application/json", json);
}

void GrayscaleDiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr || this->diagnostic_->bmp_data() == nullptr || this->diagnostic_->bmp_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"raw_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp", this->diagnostic_->bmp_data(), this->diagnostic_->bmp_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

void GrayscaleDiagnosticApiHandler::handle_preview_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr || !this->diagnostic_->preview_ready() ||
      this->diagnostic_->preview_bmp_data() == nullptr || this->diagnostic_->preview_bmp_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"preview_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp", this->diagnostic_->preview_bmp_data(),
                                          this->diagnostic_->preview_bmp_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
