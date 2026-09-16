#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;

class JpegDiagnosticApiHandler : public AsyncWebHandler {
 public:
  explicit JpegDiagnosticApiHandler(JpegDiagnostic *diagnostic);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_status_(AsyncWebServerRequest *request);
  void handle_capture_(AsyncWebServerRequest *request);
  void handle_image_(AsyncWebServerRequest *request);

  JpegDiagnostic *diagnostic_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
