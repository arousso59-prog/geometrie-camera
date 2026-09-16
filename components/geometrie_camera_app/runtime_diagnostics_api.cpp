#include "runtime_diagnostics_api.h"

#include <cstdio>

#include "grayscale_diagnostic.h"
#include "jpeg_diagnostic.h"
#include "runtime_diagnostics.h"
#include "target_search_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

RuntimeDiagnosticsApiHandler::RuntimeDiagnosticsApiHandler(RuntimeDiagnostics *runtime,
                                                           GrayscaleDiagnostic *grayscale,
                                                           JpegDiagnostic *jpeg,
                                                           TargetSearchDiagnostic *target_search)
    : runtime_(runtime), grayscale_(grayscale), jpeg_(jpeg), target_search_(target_search) {}

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

  char json[960];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"ok\","
      "\"loop\":{\"samples\":%llu,\"last_gap_us\":%u,\"average_gap_us\":%u,\"max_gap_us\":%u},"
      "\"memory\":{\"internal_free_bytes\":%u,\"internal_largest_block_bytes\":%u,"
      "\"psram_free_bytes\":%u,\"psram_largest_block_bytes\":%u},"
      "\"work\":{\"capture_pending\":%s,\"jpeg_capture_pending\":%s,\"target_search_pending\":%s}}",
      static_cast<unsigned long long>(this->runtime_->loop_sample_count()),
      static_cast<unsigned>(this->runtime_->last_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->average_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->max_loop_gap_us()),
      static_cast<unsigned>(this->runtime_->internal_free_bytes()),
      static_cast<unsigned>(this->runtime_->internal_largest_block_bytes()),
      static_cast<unsigned>(this->runtime_->psram_free_bytes()),
      static_cast<unsigned>(this->runtime_->psram_largest_block_bytes()),
      this->grayscale_ != nullptr && this->grayscale_->capture_pending() ? "true" : "false",
      this->jpeg_ != nullptr && this->jpeg_->capture_pending() ? "true" : "false",
      this->target_search_ != nullptr && this->target_search_->search_pending() ? "true" : "false");

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
