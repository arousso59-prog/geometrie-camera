#include "ov5640_timing_api.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>

#include "ov5640_timing_controller.h"

namespace esphome {
namespace geometrie_camera_app {

Ov5640TimingApiHandler::Ov5640TimingApiHandler(Ov5640TimingController *controller)
    : controller_(controller) {}

bool Ov5640TimingApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/api/camera/timing" || url == "/api/camera/timing/set";
}

void Ov5640TimingApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (url == "/api/camera/timing") {
    this->handle_status_(request);
    return;
  }

  if (url == "/api/camera/timing/set") {
    this->handle_set_(request);
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

void Ov5640TimingApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->controller_ == nullptr) {
    request->send(500, "application/json",
                  "{\"status\":\"error\",\"error\":\"timing_controller_unavailable\"}");
    return;
  }

  this->send_snapshot_(request, false, "ok", 200);
}

void Ov5640TimingApiHandler::handle_set_(AsyncWebServerRequest *request) const {
  if (this->controller_ == nullptr) {
    request->send(500, "application/json",
                  "{\"status\":\"error\",\"error\":\"timing_controller_unavailable\"}");
    return;
  }

  if (!request->hasParam("pclk_divider")) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"missing_pclk_divider\"}");
    return;
  }

  const String raw = request->getParam("pclk_divider")->value();
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(raw.c_str(), &end, 10);
  if (errno != 0 || end == raw.c_str() || *end != '\0' ||
      parsed < Ov5640TimingController::min_pclk_divider() ||
      parsed > Ov5640TimingController::max_pclk_divider()) {
    char json[192];
    std::snprintf(json, sizeof(json),
                  "{\"status\":\"error\",\"error\":\"invalid_pclk_divider\","
                  "\"allowed\":\"%u..%u\"}",
                  static_cast<unsigned>(Ov5640TimingController::min_pclk_divider()),
                  static_cast<unsigned>(Ov5640TimingController::max_pclk_divider()));
    request->send(400, "application/json", json);
    return;
  }

  if (!this->controller_->set_pclk_divider(static_cast<uint8_t>(parsed))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  this->send_snapshot_(request, true, "ok", 200);
}

void Ov5640TimingApiHandler::send_snapshot_(AsyncWebServerRequest *request, bool applied,
                                           const char *status, int response_code) const {
  const Ov5640TimingSnapshot snapshot = this->controller_->snapshot();

  char json[512];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"%s\",\"applied\":%s,\"sensor_available\":%s,"
      "\"sensor\":{\"is_ov5640\":%s,\"pid\":\"0x%04X\"},"
      "\"timing\":{\"pclk_divider\":%d,\"vfifo_ctrl0c\":%d,\"pclk_manual\":%s},"
      "\"ranges\":{\"pclk_divider\":\"%u..%u\"}}",
      status,
      applied ? "true" : "false",
      snapshot.sensor_available ? "true" : "false",
      snapshot.sensor_is_ov5640 ? "true" : "false",
      static_cast<unsigned>(snapshot.sensor_pid),
      snapshot.pclk_divider,
      snapshot.vfifo_ctrl0c,
      snapshot.pclk_manual ? "true" : "false",
      static_cast<unsigned>(Ov5640TimingController::min_pclk_divider()),
      static_cast<unsigned>(Ov5640TimingController::max_pclk_divider()));

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
