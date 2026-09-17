#include "target_tracking_api.h"

#include <cerrno>
#include <cstdlib>

#include "camera_viewport_controller.h"
#include "target_tracking_controller.h"

namespace esphome {
namespace geometrie_camera_app {

TargetTrackingApiHandler::TargetTrackingApiHandler(TargetTrackingController *tracking_controller,
                                                   CameraViewportController *viewport_controller)
    : tracking_controller_(tracking_controller), viewport_controller_(viewport_controller) {}

bool TargetTrackingApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/tracking/config" || url == "/tracking/config/set" ||
         url == "/tracking/status" || url == "/api/camera/viewport";
}

void TargetTrackingApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  if (url == "/tracking/config") {
    this->handle_config_(request);
  } else if (url == "/tracking/config/set") {
    this->handle_config_set_(request);
  } else if (url == "/tracking/status") {
    this->handle_status_(request);
  } else if (url == "/api/camera/viewport") {
    this->handle_viewport_(request);
  } else {
    request->send(404, "application/json", "{\"error\":\"not_found\"}");
  }
}

void TargetTrackingApiHandler::handle_config_(AsyncWebServerRequest *request) const {
  if (this->tracking_controller_ == nullptr) {
    this->send_config_(request, 500, "error", "tracking_unavailable");
    return;
  }
  this->send_config_(request, 200, "ok");
}

void TargetTrackingApiHandler::handle_config_set_(AsyncWebServerRequest *request) {
  if (this->tracking_controller_ == nullptr) {
    this->send_config_(request, 500, "error", "tracking_unavailable");
    return;
  }

  uint32_t enabled = 0;
  uint32_t lost_cycles = 0;
  uint32_t recenter = 0;
  bool has_enabled = false;
  bool has_lost = false;
  bool has_recenter = false;
  std::string error;

  if (!this->parse_uint_(request, "enabled", enabled, has_enabled, error) ||
      !this->parse_uint_(request, "lost_cycles", lost_cycles, has_lost, error) ||
      !this->parse_uint_(request, "recenter_threshold_pct", recenter, has_recenter, error)) {
    this->send_config_(request, 400, "error", error.c_str());
    return;
  }

  if (!has_enabled && !has_lost && !has_recenter) {
    this->send_config_(request, 400, "error", "no_setting_provided");
    return;
  }
  if (has_enabled && enabled > 1) {
    this->send_config_(request, 400, "error", "enabled_out_of_range");
    return;
  }
  if (has_lost && (lost_cycles < 1 || lost_cycles > 10)) {
    this->send_config_(request, 400, "error", "lost_cycles_out_of_range");
    return;
  }
  if (has_recenter && (recenter < 50 || recenter > 90)) {
    this->send_config_(request, 400, "error", "recenter_threshold_pct_out_of_range");
    return;
  }

  if (has_lost && !this->tracking_controller_->set_lost_cycles(static_cast<uint8_t>(lost_cycles))) {
    this->send_config_(request, 500, "error", "lost_cycles_apply_failed");
    return;
  }
  if (has_recenter &&
      !this->tracking_controller_->set_recenter_threshold_pct(static_cast<uint8_t>(recenter))) {
    this->send_config_(request, 500, "error", "recenter_threshold_apply_failed");
    return;
  }
  if (has_enabled && !this->tracking_controller_->set_enabled(enabled != 0)) {
    this->send_config_(request, 500, "error",
                       this->tracking_controller_->last_error().empty()
                           ? "enabled_apply_failed"
                           : this->tracking_controller_->last_error().c_str());
    return;
  }

  this->send_config_(request, 200, "ok");
}

void TargetTrackingApiHandler::handle_status_(AsyncWebServerRequest *request) const {
  if (this->tracking_controller_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"tracking_unavailable\"}");
    return;
  }

  std::string json;
  json.reserve(900);
  json += "{\"status\":\"ok\"";
  json += ",\"enabled\":";
  json += this->tracking_controller_->enabled() ? "true" : "false";
  json += ",\"supported\":";
  json += this->tracking_controller_->supported() ? "true" : "false";
  json += ",\"mode\":\"";
  json += this->tracking_controller_->mode_text();
  json += "\"";
  json += ",\"target_locked\":";
  json += this->tracking_controller_->target_locked() ? "true" : "false";
  json += ",\"lost_count\":" + std::to_string(this->tracking_controller_->current_lost_count());
  json += ",\"transition_count\":" + std::to_string(this->tracking_controller_->transition_count());
  json += ",\"last_error\":\"" + this->tracking_controller_->last_error() + "\"";
  json += ",\"viewport\":";
  this->append_viewport_json_(json);
  json += "}";

  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void TargetTrackingApiHandler::handle_viewport_(AsyncWebServerRequest *request) const {
  if (this->viewport_controller_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"viewport_unavailable\"}");
    return;
  }
  std::string json = "{\"status\":\"ok\",\"viewport\":";
  this->append_viewport_json_(json);
  json += "}";
  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

bool TargetTrackingApiHandler::parse_uint_(AsyncWebServerRequest *request, const char *name,
                                           uint32_t &value, bool &present,
                                           std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;
  const std::string text = request->getParam(name)->value();
  if (text.empty()) {
    error = std::string(name) + "_invalid";
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' || parsed > 0xFFFFFFFFUL) {
    error = std::string(name) + "_invalid";
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

void TargetTrackingApiHandler::send_config_(AsyncWebServerRequest *request, int response_code,
                                            const char *status, const char *error) const {
  std::string json;
  json.reserve(700);
  json += "{\"status\":\"";
  json += status;
  json += "\"";
  if (error != nullptr) {
    json += ",\"error\":\"";
    json += error;
    json += "\"";
  }
  if (this->tracking_controller_ != nullptr) {
    json += ",\"config\":{";
    json += "\"enabled\":";
    json += this->tracking_controller_->enabled() ? "true" : "false";
    json += ",\"lost_cycles\":" + std::to_string(this->tracking_controller_->lost_cycles());
    json += ",\"recenter_threshold_pct\":" +
            std::to_string(this->tracking_controller_->recenter_threshold_pct());
    json += ",\"search_resolution\":\"800x600\"";
    json += ",\"precise_output\":\"800x600\"";
    json += ",\"reference_resolution\":\"2560x1920\"";
    json += "}";
  }
  json += "}";
  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void TargetTrackingApiHandler::append_viewport_json_(std::string &json) const {
  if (this->viewport_controller_ == nullptr) {
    json += "null";
    return;
  }
  const auto &viewport = this->viewport_controller_->snapshot();
  json += "{";
  json += "\"supported\":";
  json += this->viewport_controller_->supports_precise_roi() ? "true" : "false";
  json += ",\"mode\":\"";
  json += CameraViewportController::mode_text(viewport.mode);
  json += "\"";
  json += ",\"reference\":{\"width\":" + std::to_string(viewport.reference_width) +
          ",\"height\":" + std::to_string(viewport.reference_height) + "}";
  json += ",\"window\":{\"x\":" + std::to_string(viewport.window_x) +
          ",\"y\":" + std::to_string(viewport.window_y) +
          ",\"width\":" + std::to_string(viewport.window_width) +
          ",\"height\":" + std::to_string(viewport.window_height) + "}";
  json += ",\"output\":{\"width\":" + std::to_string(viewport.output_width) +
          ",\"height\":" + std::to_string(viewport.output_height) + "}";
  json += ",\"scale\":{\"x\":" + std::to_string(viewport.scale_x) +
          ",\"y\":" + std::to_string(viewport.scale_y) + "}";
  json += "}";
}

}  // namespace geometrie_camera_app
}  // namespace esphome
