#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class GrayscaleDiagnostic;

class GrayscaleDiagnosticApiHandler : public AsyncWebHandler {
 public:
  explicit GrayscaleDiagnosticApiHandler(GrayscaleDiagnostic *diagnostic);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  GrayscaleDiagnostic *diagnostic_;

  void handle_status_(AsyncWebServerRequest *request);
  void handle_capture_(AsyncWebServerRequest *request);
  void handle_image_(AsyncWebServerRequest *request);
};

}  // namespace geometrie_camera_app
}  // namespace esphome
