#include "runtime_diagnostics_api.h"

#include <cstdio>

#include "jpeg_diagnostic.h"
#include "runtime_diagnostics.h"

namespace esphome {
namespace geometrie_camera_app {

RuntimeDiagnosticsApiHandler::RuntimeDiagnosticsApiHandler(RuntimeDiagnostics *runtime, JpegDiagnostic *jpeg)
    : runtime_(runtime), jpeg_(jpeg) {}

bool RuntimeDiagnosticsApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(url_buf) == "/api/runtime/status";
}

void RuntimeDiagnosticsApiHandler::handleRequest(AsyncWebServerRequest *request) {
  if (this->runtime_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"runtime_diagnostics_unavailable\"}");
    return;
  }

  char json[768];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\","
      "\"loop\":{\"samples\":%llu,\"last_gap_us\":%u,\"average_gap_us\":%u,\"max_gap_us\":%u},"
      "\"memory\":{\"internal_free_bytes\":%u,\"internal_largest_block_bytes\":%u,"
      "\"psram_free_bytes\":%u,\"psram_largest_block_bytes\":%u},"
      "\"work\":{\"jpeg_capture_pending\":%s}}",
      static_cast<unsigned long long>(this->runtime_->loop_sample_count()),
      static_cast<unsigned>(this->runtime_->last_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->average_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->max_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->internal_free_bytes()),
      static_cast<unsigned>(this->runtime_->internal_largest_block_bytes()),
      static_cast<unsigned>(this->runtime_->psram_free_bytes()),
      static_cast<unsigned>(this->runtime_->psram_largest_block_bytes()),
      this->jpeg_ != nullptr && this->jpeg_->capture_pending() ? "true" : "false");

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
