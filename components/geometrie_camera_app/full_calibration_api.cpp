#include "full_calibration_api.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>

#include "full_calibration_controller.h"
#include "target_detection_preview.h"

namespace esphome {
namespace geometrie_camera_app {

FullCalibrationApiHandler::FullCalibrationApiHandler(FullCalibrationController *controller,
                                                     TargetDetectionPreview *preview)
    : controller_(controller), preview_(preview) {}

bool FullCalibrationApiHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);
  return url == "/calibration/full/start" ||
         url == "/calibration/full/status" ||
         url == "/calibration/full/cancel" ||
         url == "/calibration/full/preview.bmp";
}

void FullCalibrationApiHandler::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buf);

  if (this->controller_ == nullptr) {
    request->send(500, "application/json",
                  "{\"status\":\"error\",\"error\":\"calibration_controller_unavailable\"}");
    return;
  }

  if (url == "/calibration/full/status") {
    this->send_status_(request, 200, "ok");
    return;
  }

  if (url == "/calibration/full/cancel") {
    this->controller_->cancel();
    this->send_status_(request, 200, "ok");
    return;
  }

  if (url == "/calibration/full/preview.bmp") {
    if (this->preview_ == nullptr || this->controller_->preview_attempt() == 0 ||
        this->preview_->bmp_data() == nullptr || this->preview_->bmp_size() == 0) {
      request->send(404, "application/json",
                    "{\"error\":\"calibration_preview_unavailable\"}");
      return;
    }

    auto *response = request->beginResponse(200, "image/bmp",
                                            this->preview_->bmp_data(),
                                            this->preview_->bmp_size());
    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response->addHeader("Pragma", "no-cache");
    request->send(response);
    return;
  }

  if (url == "/calibration/full/start") {
    // Demarrage idempotent : si une calibration est deja en cours, ne pas
    // renvoyer 409. Le PC peut reprendre le suivi via /status sans relancer
    // ni interrompre l'operation ESP.
    if (this->controller_->running()) {
      request->send(200, "application/json",
                    "{\"status\":\"already_running\",\"running\":true}");
      return;
    }

    float distance_mm = 0.0f;
    float target_size_mm = 0.0f;
    int sample_count = FullCalibrationController::DEFAULT_SAMPLE_COUNT;
    int force_value = 0;
    bool has_distance = false;
    bool has_target_size = false;
    bool has_sample_count = false;
    bool has_force = false;
    std::string error;

    if (!this->parse_float_param_(request, "distance_mm", distance_mm, has_distance, error) ||
        !this->parse_float_param_(request, "target_size_mm", target_size_mm, has_target_size, error) ||
        !this->parse_int_param_(request, "samples", sample_count, has_sample_count, error) ||
        !this->parse_int_param_(request, "force", force_value, has_force, error)) {
      const std::string body =
          std::string("{\"status\":\"error\",\"error\":\"") + error + "\"}";
      request->send(400, "application/json", body.c_str());
      return;
    }

    if (!has_distance || !has_target_size) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"distance_mm_and_target_size_mm_required\"}");
      return;
    }
    if (has_force && force_value != 0 && force_value != 1) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"force_must_be_0_or_1\"}");
      return;
    }
    if (sample_count < 3 || sample_count > FullCalibrationController::MAX_SAMPLE_COUNT) {
      request->send(
          400, "application/json",
          "{\"status\":\"error\",\"error\":\"sample_count_out_of_range\"}");
      return;
    }

    if (!this->controller_->start(
            distance_mm,
            target_size_mm,
            static_cast<uint8_t>(sample_count),
            has_force && force_value == 1)) {
      this->send_status_(request, 409, "error");
      return;
    }

    // Reponse volontairement minimale. Ne pas serialiser tout l'etat dans
    // la requete qui vient elle-meme de lancer capture + tracking.
    request->send(202, "application/json",
                  "{\"status\":\"accepted\",\"running\":true}");
    return;
  }

  request->send(404, "application/json", "{\"error\":\"not_found\"}");
}

bool FullCalibrationApiHandler::parse_float_param_(
    AsyncWebServerRequest *request, const char *name,
    float &value, bool &present, std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;

  const std::string text = request->getParam(name)->value();
  if (text.empty()) {
    error = std::string(name) + "_invalid";
    return false;
  }

  errno = 0;
  char *end = nullptr;
  const float parsed = std::strtof(text.c_str(), &end);
  if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    error = std::string(name) + "_invalid";
    return false;
  }

  value = parsed;
  return true;
}

bool FullCalibrationApiHandler::parse_int_param_(
    AsyncWebServerRequest *request, const char *name,
    int &value, bool &present, std::string &error) const {
  present = request->hasParam(name);
  if (!present) return true;

  const std::string text = request->getParam(name)->value();
  if (text.empty()) {
    error = std::string(name) + "_invalid";
    return false;
  }

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

void FullCalibrationApiHandler::send_status_(
    AsyncWebServerRequest *request, int response_code, const char *status) const {
  const CameraCalibration &calibration = this->controller_->result_calibration();

  std::string json;
  json.reserve(3000);
  json += "{\"status\":\"";
  json += status;
  json += "\"";
  json += ",\"state\":\"" + std::string(this->controller_->state_text()) + "\"";
  json += ",\"running\":";
  json += this->controller_->running() ? "true" : "false";
  json += ",\"acquisition_mode\":\"precise_native_800x600_optical_tuning\"";
  json += ",\"phase\":\"" + std::string(this->controller_->phase_text()) + "\"";
  json += ",\"reference_resolution\":\"2560x1920\"";
  json += ",\"requested_samples\":" + std::to_string(this->controller_->requested_samples());
  json += ",\"valid_samples\":" + std::to_string(this->controller_->valid_samples());
  json += ",\"attempts\":" + std::to_string(this->controller_->attempts());
  json += ",\"max_attempts\":" + std::to_string(this->controller_->max_attempts());
  json += ",\"known_distance_mm\":" + std::to_string(this->controller_->known_distance_mm());
  json += ",\"target_size_mm\":" + std::to_string(this->controller_->target_size_mm());

  const uint8_t requested = this->controller_->requested_samples();
  const uint8_t valid = this->controller_->valid_samples();
  const unsigned progress = requested > 0
                                ? static_cast<unsigned>(valid) * 100U / requested
                                : 0U;
  json += ",\"progress_pct\":" + std::to_string(progress);
  json += ",\"tracking_mode\":\"" + std::string(this->controller_->tracking_mode_text()) + "\"";
  json += ",\"preview_attempt\":" + std::to_string(this->controller_->preview_attempt());
  json += ",\"preview_mode\":\"" + this->controller_->preview_mode() + "\"";
  json += ",\"last_target_found\":";
  json += this->controller_->last_target_found() ? "true" : "false";
  json += ",\"last_sample_valid\":";
  json += this->controller_->last_sample_valid() ? "true" : "false";
  json += ",\"last_sample_fx_px\":" + std::to_string(this->controller_->last_sample_fx_px());
  json += ",\"last_sample_fy_px\":" + std::to_string(this->controller_->last_sample_fy_px());

  json += ",\"optical_tuning\":{";
  json += "\"attempt\":" + std::to_string(this->controller_->tuning_attempts());
  json += ",\"max_attempts\":" + std::to_string(this->controller_->tuning_max_attempts());
  json += ",\"current_ae_level\":" + std::to_string(this->controller_->current_ae_level());
  json += ",\"current_exposure\":" + std::to_string(this->controller_->current_exposure());
  json += ",\"current_gain\":" + std::to_string(this->controller_->current_gain());
  json += ",\"current_brightness\":" + std::to_string(this->controller_->current_brightness());
  json += ",\"current_contrast\":" + std::to_string(this->controller_->current_contrast());
  json += ",\"current_score\":" + std::to_string(this->controller_->current_optical_score());
  json += ",\"detection_quality\":" + std::to_string(this->controller_->current_detection_quality());
  json += ",\"subpixel_rms_px\":" + std::to_string(this->controller_->current_subpixel_rms_px());
  json += ",\"width_gradient\":" + std::to_string(this->controller_->current_width_gradient());
  json += ",\"height_gradient\":" + std::to_string(this->controller_->current_height_gradient());
  json += ",\"mean_luma_x100\":" + std::to_string(this->controller_->current_mean_luma_x100());
  json += ",\"dark_percent_x100\":" + std::to_string(this->controller_->current_dark_percent_x100());
  json += ",\"bright_percent_x100\":" + std::to_string(this->controller_->current_bright_percent_x100());
  json += ",\"p10_luma\":" + std::to_string(this->controller_->current_p10_luma());
  json += ",\"p90_luma\":" + std::to_string(this->controller_->current_p90_luma());
  json += ",\"contrast_luma\":" + std::to_string(this->controller_->current_contrast_luma());
  json += ",\"best_ae_level\":" + std::to_string(this->controller_->best_ae_level());
  json += ",\"best_exposure\":" + std::to_string(this->controller_->best_exposure());
  json += ",\"best_gain\":" + std::to_string(this->controller_->best_gain());
  json += ",\"best_brightness\":" + std::to_string(this->controller_->best_brightness());
  json += ",\"best_contrast\":" + std::to_string(this->controller_->best_contrast());
  json += ",\"best_score\":" + std::to_string(this->controller_->best_optical_score());
  json += "}";

  json += ",\"preview\":\"/calibration/full/preview.bmp\"";

  if (!this->controller_->last_error().empty()) {
    json += ",\"error\":\"" + this->controller_->last_error() + "\"";
  }

  json += ",\"result\":{\"valid\":";
  const bool complete = this->controller_->state() == FullCalibrationState::COMPLETE;
  json += complete ? "true" : "false";
  json += ",\"mean_fx_px\":" + std::to_string(this->controller_->mean_fx_px());
  json += ",\"mean_fy_px\":" + std::to_string(this->controller_->mean_fy_px());
  json += ",\"stddev_fx_px\":" + std::to_string(this->controller_->stddev_fx_px());
  json += ",\"stddev_fy_px\":" + std::to_string(this->controller_->stddev_fy_px());

  // Pendant l'operation, garder la reponse courte : le PC ne requiert que
  // progression + statistiques. La calibration complete n'est serialisee
  // qu'une fois l'operation terminee.
  if (complete) {
    json += ",\"calibration\":{";
    json += "\"fx_px\":" + std::to_string(calibration.fx_px);
    json += ",\"fy_px\":" + std::to_string(calibration.fy_px);
    json += ",\"cx_px\":" + std::to_string(calibration.cx_px);
    json += ",\"cy_px\":" + std::to_string(calibration.cy_px);
    json += ",\"k1\":" + std::to_string(calibration.k1);
    json += ",\"k2\":" + std::to_string(calibration.k2);
    json += ",\"p1\":" + std::to_string(calibration.p1);
    json += ",\"p2\":" + std::to_string(calibration.p2);
    json += ",\"k3\":" + std::to_string(calibration.k3);
    json += ",\"reference_width_px\":" + std::to_string(calibration.reference_width_px);
    json += ",\"reference_height_px\":" + std::to_string(calibration.reference_height_px);
    json += "}";
  }
  json += "}}";

  auto *response = request->beginResponse(response_code, "application/json", json);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
