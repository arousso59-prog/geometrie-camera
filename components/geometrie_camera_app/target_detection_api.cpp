#include "target_detection_api.h"

#include <string>

#include "jpeg_filtered_diagnostic.h"
#include "target_detection_preview.h"
#include "target_detection_service.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

TargetDetectionApiHandler::TargetDetectionApiHandler(TargetDetectionService *service,
                                                     JpegFilteredDiagnostic *source,
                                                     TargetDetectionPreview *preview)
    : service_(service), source_(source), preview_(preview) {}

bool TargetDetectionApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/target/status" || url == "/target/preview.bmp";
}

void TargetDetectionApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/target/status") {
    this->handle_status_(request);
    return;
  }
  if (url == "/target/preview.bmp") {
    this->handle_preview_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void TargetDetectionApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->service_ == nullptr) {
    this->send_status_(request, 500, "error", "target_detection_unavailable");
    return;
  }

  this->send_status_(request, 200, "ok");
}

void TargetDetectionApiHandler::handle_preview_(AsyncWebServerRequest *request) {
  if (this->service_ == nullptr || this->source_ == nullptr || this->preview_ == nullptr ||
      !this->service_->ready()) {
    request->send(409, "application/json", "{\"error\":\"target_detection_unavailable\"}");
    return;
  }

  if (!this->preview_->render(this->source_, this->service_->last_observation()) ||
      this->preview_->bmp_data() == nullptr || this->preview_->bmp_size() == 0) {
    request->send(500, "application/json", "{\"error\":\"target_preview_failed\"}");
    return;
  }

  auto *response = request->beginResponse(200, "image/bmp",
                                          this->preview_->bmp_data(),
                                          this->preview_->bmp_size());
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  response->addHeader("Pragma", "no-cache");
  request->send(response);
}

void TargetDetectionApiHandler::send_status_(AsyncWebServerRequest *request, int response_code,
                                             const char *status, const char *error) const {
  std::string json;
  json.reserve(1250);
  json += "{\"status\":\"";
  json += status;
  json += "\"";

  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }

  if (this->service_ != nullptr) {
    const TargetObservation &observation = this->service_->last_observation();
    json += ",\"ready\":";
    json += this->service_->ready() ? "true" : "false";
    json += ",\"target_found\":";
    json += this->service_->target_found() ? "true" : "false";
    json += ",\"detection_count\":";
    json += std::to_string(this->service_->detection_count());
    json += ",\"source_process_count\":";
    json += std::to_string(this->service_->source_process_count());
    json += ",\"detection_ms\":";
    json += std::to_string(this->service_->detection_ms());
    json += ",\"target\":{\"center_x_px\":";
    json += std::to_string(observation.center_x_px);
    json += ",\"center_y_px\":";
    json += std::to_string(observation.center_y_px);
    json += ",\"width_px\":";
    json += std::to_string(observation.width_px);
    json += ",\"height_px\":";
    json += std::to_string(observation.height_px);
    json += ",\"rotation_deg\":";
    json += std::to_string(observation.rotation_deg);
    json += ",\"quality\":";
    json += std::to_string(observation.quality);
    json += ",\"subpixel_refined\":";
    json += observation.subpixel_refined ? "true" : "false";
    json += ",\"subpixel_rms_px\":";
    json += std::to_string(observation.subpixel_rms_px);
    json += ",\"subpixel_max_rms_px\":";
    json += std::to_string(observation.subpixel_max_rms_px);
    json += ",\"subpixel_gradient\":";
    json += std::to_string(observation.subpixel_gradient);
    json += ",\"subpixel_width_px\":";
    json += std::to_string(observation.subpixel_width_px);
    json += ",\"subpixel_height_px\":";
    json += std::to_string(observation.subpixel_height_px);
    json += ",\"subpixel_width_sigma_px\":";
    json += std::to_string(observation.subpixel_width_sigma_px);
    json += ",\"subpixel_height_sigma_px\":";
    json += std::to_string(observation.subpixel_height_sigma_px);
    json += ",\"subpixel_width_gradient\":";
    json += std::to_string(observation.subpixel_width_gradient);
    json += ",\"subpixel_height_gradient\":";
    json += std::to_string(observation.subpixel_height_gradient);
    json += "},\"preview\":\"/target/preview.bmp\"";
  }

  json += "}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
