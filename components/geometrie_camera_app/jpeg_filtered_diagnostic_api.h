#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegFilteredDiagnostic;

class JpegFilteredDiagnosticApiHandler : public AsyncWebHandler {
 public:
  explicit JpegFilteredDiagnosticApiHandler(JpegFilteredDiagnostic *diagnostic);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_filter_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request) const;
  void handle_image_(AsyncWebServerRequest *request) const;
  void send_status_(AsyncWebServerRequest *request, int response_code,
                    const char *status, const char *error = nullptr) const;

  JpegFilteredDiagnostic *diagnostic_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
