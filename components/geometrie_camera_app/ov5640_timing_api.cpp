#include "ov5640_timing_api.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "ov5640_timing_controller.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
bool parse_integer_parameter(AsyncWebServerRequest *request, const char *name, long min_value,
                             long max_value, long *parsed_value) {
  if (request == nullptr || name == nullptr || parsed_value == nullptr || !request->hasParam(name)) {
    return false;
  }

  const std::string raw = request->getParam(name)->value();
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(raw.c_str(), &end, 0);
  if (errno != 0 || end == raw.c_str() || *end != '\0' || parsed < min_value || parsed > max_value) {
    return false;
  }

  *parsed_value = parsed;
  return true;
}
}

Ov5640TimingApiHandler::Ov5640TimingApiHandler(Ov5640TimingController *controller)
    : controller_(controller) {}

bool Ov5640TimingApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/api/camera/timing" || url == "/api/camera/timing/set" ||
         url == "/api/camera/timing/restore";
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

  if (url == "/api/camera/timing/restore") {
    this->handle_restore_(request);
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

  const bool has_pclk = request->hasParam("pclk_divider");
  const bool has_hts = request->hasParam("hts");
  const bool has_vts = request->hasParam("vts");
  const bool has_jpeg_mode = request->hasParam("jpeg_mode");
  const bool has_href_blanking = request->hasParam("href_blanking");
  if (!has_pclk && !has_hts && !has_vts && !has_jpeg_mode && !has_href_blanking) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"missing_timing_parameter\"}");
    return;
  }

  long pclk = 0;
  long hts = 0;
  long vts = 0;
  long jpeg_mode = 0;
  long href_blanking = 0;

  if (has_pclk &&
      !parse_integer_parameter(request, "pclk_divider", Ov5640TimingController::min_pclk_divider(),
                               Ov5640TimingController::max_pclk_divider(), &pclk)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"invalid_pclk_divider\",\"allowed\":\"1..31\"}");
    return;
  }

  if (has_hts &&
      !parse_integer_parameter(request, "hts", Ov5640TimingController::min_total_timing(),
                               Ov5640TimingController::max_total_timing(), &hts)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"invalid_hts\",\"allowed\":\"1..65535\"}");
    return;
  }

  if (has_vts &&
      !parse_integer_parameter(request, "vts", Ov5640TimingController::min_total_timing(),
                               Ov5640TimingController::max_total_timing(), &vts)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"invalid_vts\",\"allowed\":\"1..65535\"}");
    return;
  }

  if (has_jpeg_mode &&
      !parse_integer_parameter(request, "jpeg_mode", 2, 3, &jpeg_mode)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"invalid_jpeg_mode\",\"allowed\":\"2,3\"}");
    return;
  }

  if (has_href_blanking &&
      !parse_integer_parameter(request, "href_blanking", Ov5640TimingController::min_href_blanking(),
                               Ov5640TimingController::max_href_blanking(), &href_blanking)) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"error\":\"invalid_href_blanking\",\"allowed\":\"0..255\"}");
    return;
  }

  if (has_pclk && !this->controller_->set_pclk_divider(static_cast<uint8_t>(pclk))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  if (has_hts && !this->controller_->set_hts(static_cast<uint16_t>(hts))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  if (has_vts && !this->controller_->set_vts(static_cast<uint16_t>(vts))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  if (has_jpeg_mode && !this->controller_->set_jpeg_mode(static_cast<uint8_t>(jpeg_mode))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  if (has_href_blanking &&
      !this->controller_->set_href_blanking(static_cast<uint8_t>(href_blanking))) {
    this->send_snapshot_(request, false, "apply_failed", 500);
    return;
  }

  this->send_snapshot_(request, true, "ok", 200);
}

void Ov5640TimingApiHandler::handle_restore_(AsyncWebServerRequest *request) const {
  if (this->controller_ == nullptr) {
    request->send(500, "application/json",
                  "{\"status\":\"error\",\"error\":\"timing_controller_unavailable\"}");
    return;
  }

  if (!this->controller_->baseline_available()) {
    request->send(409, "application/json",
                  "{\"status\":\"error\",\"error\":\"no_timing_baseline\"}");
    return;
  }

  if (!this->controller_->restore_baseline()) {
    this->send_snapshot_(request, false, "restore_failed", 500);
    return;
  }

  this->send_snapshot_(request, true, "ok", 200);
}

void Ov5640TimingApiHandler::send_snapshot_(AsyncWebServerRequest *request, bool applied,
                                           const char *status, int response_code) const {
  const Ov5640TimingSnapshot snapshot = this->controller_->snapshot();

  char json[1152];
  std::snprintf(
      json, sizeof(json),
      "{\"status\":\"%s\",\"applied\":%s,\"sensor_available\":%s,"
      "\"sensor\":{\"is_ov5640\":%s,\"pid\":\"0x%04X\"},"
      "\"timing\":{\"pclk_divider\":%d,\"vfifo_ctrl0c\":%d,\"pclk_manual\":%s,"
      "\"hts\":%d,\"vts\":%d,\"hts_hex\":\"0x%04X\",\"vts_hex\":\"0x%04X\","
      "\"jpeg_mode\":%d,\"jpeg_mode_hex\":\"0x%02X\","
      "\"href_blanking\":%d,\"href_blanking_hex\":\"0x%02X\"},"
      "\"baseline\":{\"available\":%s,\"hts\":%u,\"vts\":%u,"
      "\"hts_hex\":\"0x%04X\",\"vts_hex\":\"0x%04X\","
      "\"jpeg_mode\":%u,\"jpeg_mode_hex\":\"0x%02X\","
      "\"href_blanking\":%u,\"href_blanking_hex\":\"0x%02X\"},"
      "\"ranges\":{\"pclk_divider\":\"1..31\",\"hts\":\"1..65535\","
      "\"vts\":\"1..65535\",\"jpeg_mode\":\"2,3\",\"href_blanking\":\"0..255\"}}",
      status,
      applied ? "true" : "false",
      snapshot.sensor_available ? "true" : "false",
      snapshot.sensor_is_ov5640 ? "true" : "false",
      static_cast<unsigned>(snapshot.sensor_pid),
      snapshot.pclk_divider,
      snapshot.vfifo_ctrl0c,
      snapshot.pclk_manual ? "true" : "false",
      snapshot.hts,
      snapshot.vts,
      static_cast<unsigned>(snapshot.hts >= 0 ? snapshot.hts : 0),
      static_cast<unsigned>(snapshot.vts >= 0 ? snapshot.vts : 0),
      snapshot.jpeg_mode,
      static_cast<unsigned>(snapshot.jpeg_mode >= 0 ? snapshot.jpeg_mode : 0),
      snapshot.href_blanking,
      static_cast<unsigned>(snapshot.href_blanking >= 0 ? snapshot.href_blanking : 0),
      snapshot.baseline_available ? "true" : "false",
      static_cast<unsigned>(snapshot.baseline_hts),
      static_cast<unsigned>(snapshot.baseline_vts),
      static_cast<unsigned>(snapshot.baseline_hts),
      static_cast<unsigned>(snapshot.baseline_vts),
      static_cast<unsigned>(snapshot.baseline_jpeg_mode),
      static_cast<unsigned>(snapshot.baseline_jpeg_mode),
      static_cast<unsigned>(snapshot.baseline_href_blanking),
      static_cast<unsigned>(snapshot.baseline_href_blanking));

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
