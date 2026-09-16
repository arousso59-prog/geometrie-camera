#include "jpeg_filtered_diagnostic_api.h"

#include <string>

#include "jpeg_filtered_diagnostic.h"

namespace esphome {
namespace geometrie_camera_app {

JpegFilteredDiagnosticApiHandler::JpegFilteredDiagnosticApiHandler(JpegFilteredDiagnostic *diagnostic)
    : diagnostic_(diagnostic) {}

bool JpegFilteredDiagnosticApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/diagnostic-jpeg/filter" ||
         url == "/diagnostic-jpeg/filter-status" ||
         url == "/diagnostic-jpeg/filtered.bmp";
}

void JpegFilteredDiagnosticApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/diagnostic-jpeg/filter") {
    this->handle_filter_(request);
    return;
  }
  if (url == "/diagnostic-jpeg/filter-status") {
    this->handle_status_(request);
    return;
  }
  if (url == "/diagnostic-jpeg/filtered.bmp") {
    this->handle_image_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void JpegFilteredDiagnosticApiHandler::handle_filter_(AsyncWebServerRequest *request) {
  if (this->diagnostic_ == nullptr) {
    this->send_status_(request, 500, "error", "filtered_diagnostic_unavailable");
    return;
  }

  if (!this->diagnostic_->process()) {
    this->send_status_(request, 500, "error", "filter_failed");
    return;
  }

  this->send_status_(request, 200, "ok");
}

void JpegFilteredDiagnosticApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->diagnostic_ == nullptr) {
    this->send_status_(request, 500, "error", "filtered_diagnostic_unavailable");
    return;
  }
  this->send_status_(request, 200, "ok");
}

void JpegFilteredDiagnosticApiHandler::handle_image_(AsyncWebServerRequest *request) const {
  if (this->diagnostic_ == nullptr || !this->diagnostic_->ready() ||
      this->diagnostic_->bmp_data() == nullptr || this->diagnostic_->bmp_size() == 0) {
    request->send(404, "application/json", "{\"error\":\"filtered_image_unavailable\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp", this->diagnostic_->bmp_data(),
                                          this->diagnostic_->bmp_size());
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void JpegFilteredDiagnosticApiHandler::send_status_(AsyncWebServerRequest *request, int response_code,
                                                    const char *status, const char *error) const {
  std::string json;
  json.reserve(768);
  json += "{\"status\":\"";
  json += status;
  json += "\"";
  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }

  if (this->diagnostic_ != nullptr) {
    const auto &stats = this->diagnostic_->correction_stats();
    json += ",\"ready\":";
    json += this->diagnostic_->ready() ? "true" : "false";
    json += ",\"process_count\":";
    json += std::to_string(this->diagnostic_->process_count());
    json += ",\"source_capture_count\":";
    json += std::to_string(this->diagnostic_->source_capture_count());
    json += ",\"width\":";
    json += std::to_string(this->diagnostic_->width());
    json += ",\"height\":";
    json += std::to_string(this->diagnostic_->height());
    json += ",\"bmp_size\":";
    json += std::to_string(this->diagnostic_->bmp_size());
    json += ",\"decode_result\":";
    json += std::to_string(this->diagnostic_->decode_result());
    json += ",\"timing\":{\"decode_ms\":";
    json += std::to_string(this->diagnostic_->decode_ms());
    json += ",\"correction_ms\":";
    json += std::to_string(this->diagnostic_->correction_ms());
    json += ",\"total_ms\":";
    json += std::to_string(this->diagnostic_->total_ms());
    json += "},\"artifacts\":{\"green_seed_pixels\":";
    json += std::to_string(stats.green_seed_pixels);
    json += ",\"thin_green_pixels\":";
    json += std::to_string(stats.thin_green_pixels);
    json += ",\"affected_rows\":";
    json += std::to_string(stats.affected_rows);
    json += ",\"corrected_green_pixels\":";
    json += std::to_string(stats.corrected_green_pixels);
    json += ",\"corrected_dark_pixels\":";
    json += std::to_string(stats.corrected_dark_pixels);
    json += ",\"corrected_total_pixels\":";
    json += std::to_string(stats.corrected_total_pixels);
    json += "},\"image\":\"/diagnostic-jpeg/filtered.bmp\"";
  }
  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
