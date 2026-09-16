#pragma once

#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;

class MeasurementApiHandler : public AsyncWebHandler {
 public:
  MeasurementApiHandler(MeasurementManager *manager,
                        TargetDetectionService *detection_service,
                        JpegFilteredDiagnostic *source);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  bool parse_float_param_(AsyncWebServerRequest *request, const char *name,
                          float &value, bool &present, std::string &error) const;
  bool detection_is_current_() const;
  void handle_compute_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request) const;
  void handle_config_(AsyncWebServerRequest *request) const;
  void handle_config_set_(AsyncWebServerRequest *request);
  void handle_calibrate_(AsyncWebServerRequest *request);
  void send_snapshot_(AsyncWebServerRequest *request, int response_code,
                      const char *status, const char *error = nullptr) const;

  MeasurementManager *manager_;
  TargetDetectionService *detection_service_;
  JpegFilteredDiagnostic *source_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
