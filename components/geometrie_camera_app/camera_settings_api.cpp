#include "camera_settings_api.h"

#include <cerrno>
#include <cstdlib>
#include <string>

#include "camera_resolution_controller.h"
#include "camera_settings_controller.h"

namespace esphome {
namespace geometrie_camera_app {

CameraSettingsApiHandler::CameraSettingsApiHandler(CameraSettingsController *controller,
                                                   CameraResolutionController *resolution_controller)
    : controller_(controller), resolution_controller_(resolution_controller) {}

bool CameraSettingsApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/api/camera/settings" || url == "/api/camera/settings/set";
}

void CameraSettingsApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  if (url == "/api/camera/settings") return this->handle_status_(request);
  if (url == "/api/camera/settings/set") return this->handle_set_(request);
  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool CameraSettingsApiHandler::parse_int_param_(AsyncWebServerRequest *request, const char *name, int &value,
                                                bool &present, std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;
  const std::string text = request->getParam(name)->value();
  if (text.empty()) { error = std::string(name) + "_invalid"; return false; }
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0') {
    error = std::string(name) + "_invalid";
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool CameraSettingsApiHandler::parse_bool_param_(AsyncWebServerRequest *request, const char *name, bool &value,
                                                 bool &present, std::string &error) const {
  int parsed = 0;
  if (!this->parse_int_param_(request, name, parsed, present, error)) return false;
  if (!present) return true;
  if (parsed != 0 && parsed != 1) {
    error = std::string(name) + "_must_be_0_or_1";
    return false;
  }
  value = parsed != 0;
  return true;
}

void CameraSettingsApiHandler::send_snapshot_(AsyncWebServerRequest *request, bool applied, const char *status) const {
  if (this->controller_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"camera_settings_unavailable\"}");
    return;
  }
  const CameraSettingsSnapshot snapshot = this->controller_->read();
  if (!snapshot.available) {
    request->send(503, "application/json", "{\"status\":\"error\",\"error\":\"camera_sensor_unavailable\"}");
    return;
  }

  std::string json;
  json.reserve(1000);
  json += "{\"status\":\"";
  json += status;
  json += "\",\"applied\":";
  json += applied ? "true" : "false";
  json += ",\"settings\":{";
  if (this->resolution_controller_ != nullptr) {
    json += "\"resolution\":\"" + this->resolution_controller_->active_resolution() + "\"";
    json += ",\"width\":" + std::to_string(this->resolution_controller_->active_width());
    json += ",\"height\":" + std::to_string(this->resolution_controller_->active_height()) + ",";
  }
  json += "\"pixel_format\":\"" + snapshot.pixel_format + "\"";
  json += ",\"brightness\":" + std::to_string(snapshot.brightness);
  json += ",\"contrast\":" + std::to_string(snapshot.contrast);
  json += ",\"exposure_ctrl\":" + std::string(snapshot.exposure_ctrl ? "true" : "false");
  json += ",\"ae_level\":" + std::to_string(snapshot.ae_level);
  json += ",\"aec_value\":" + std::to_string(snapshot.aec_value);
  json += ",\"gain_ctrl\":" + std::string(snapshot.gain_ctrl ? "true" : "false");
  json += ",\"agc_gain\":" + std::to_string(snapshot.agc_gain);
  json += "},\"ranges\":{\"resolution\":\"";
  json += CameraResolutionController::allowed_resolutions_text();
  json += "\",\"pixel_format\":\"jpeg|grayscale (boot only)\",\"brightness\":\"-2..2\",\"contrast\":\"-2..2\",\"exposure_ctrl\":\"0|1\",\"ae_level\":\"-2..2\",\"aec_value\":\"0..1200\",\"gain_ctrl\":\"0|1\",\"agc_gain\":\"0..30\"}}";
  auto *response = request->beginResponse(200, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

void CameraSettingsApiHandler::handle_status_(AsyncWebServerRequest *request) {
  this->send_snapshot_(request, false, "ok");
}

void CameraSettingsApiHandler::handle_set_(AsyncWebServerRequest *request) {
  if (this->controller_ == nullptr) {
    request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"camera_settings_unavailable\"}");
    return;
  }

  const bool has_resolution = request->hasParam("resolution");
  std::string resolution;
  if (has_resolution) {
    resolution = request->getParam("resolution")->value();
    if (resolution.empty()) {
      request->send(400, "application/json", "{\"status\":\"error\",\"error\":\"resolution_invalid\"}");
      return;
    }
  }

  int brightness = 0, contrast = 0, ae_level = 0, aec_value = 0, agc_gain = 0;
  bool exposure_ctrl = false, gain_ctrl = false;
  bool has_brightness = false, has_contrast = false, has_exposure_ctrl = false;
  bool has_ae_level = false, has_aec_value = false, has_gain_ctrl = false, has_agc_gain = false;
  std::string error;
  if (!this->parse_int_param_(request, "brightness", brightness, has_brightness, error) ||
      !this->parse_int_param_(request, "contrast", contrast, has_contrast, error) ||
      !this->parse_bool_param_(request, "exposure_ctrl", exposure_ctrl, has_exposure_ctrl, error) ||
      !this->parse_int_param_(request, "ae_level", ae_level, has_ae_level, error) ||
      !this->parse_int_param_(request, "aec_value", aec_value, has_aec_value, error) ||
      !this->parse_bool_param_(request, "gain_ctrl", gain_ctrl, has_gain_ctrl, error) ||
      !this->parse_int_param_(request, "agc_gain", agc_gain, has_agc_gain, error)) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }

  if (!has_resolution && !has_brightness && !has_contrast && !has_exposure_ctrl && !has_ae_level &&
      !has_aec_value && !has_gain_ctrl && !has_agc_gain) {
    request->send(400, "application/json", "{\"status\":\"error\",\"error\":\"no_setting_provided\"}");
    return;
  }

  if (has_resolution) {
    if (this->resolution_controller_ == nullptr) {
      request->send(500, "application/json", "{\"status\":\"error\",\"error\":\"camera_resolution_unavailable\"}");
      return;
    }
    if (!this->resolution_controller_->is_supported(resolution) || !this->resolution_controller_->apply(resolution)) {
      request->send(400, "application/json", "{\"status\":\"error\",\"error\":\"resolution_unsupported_or_unavailable\"}");
      return;
    }
  }

  if ((has_brightness && !this->controller_->set_brightness(brightness, error)) ||
      (has_contrast && !this->controller_->set_contrast(contrast, error)) ||
      (has_exposure_ctrl && !this->controller_->set_exposure_ctrl(exposure_ctrl, error)) ||
      (has_ae_level && !this->controller_->set_ae_level(ae_level, error)) ||
      (has_aec_value && !this->controller_->set_aec_value(aec_value, error)) ||
      (has_gain_ctrl && !this->controller_->set_gain_ctrl(gain_ctrl, error)) ||
      (has_agc_gain && !this->controller_->set_agc_gain(agc_gain, error))) {
    const std::string body = std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
    request->send(400, "application/json", body.c_str());
    return;
  }
  this->send_snapshot_(request, true, "ok");
}

}  // namespace geometrie_camera_app
}  // namespace esphome
