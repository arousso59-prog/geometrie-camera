#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController;
class GrayscaleDiagnostic;
class TargetSearchDiagnostic;

class TargetSearchDiagnosticApiHandler : public AsyncWebHandler {
 public:
  TargetSearchDiagnosticApiHandler(TargetSearchDiagnostic *diagnostic, GrayscaleDiagnostic *visualization,
                                   CameraResolutionController *resolution_controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  bool apply_requested_resolution_(AsyncWebServerRequest *request);
  void handle_search_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request);
  void handle_image_(AsyncWebServerRequest *request);

  TargetSearchDiagnostic *diagnostic_;
  GrayscaleDiagnostic *visualization_;
  CameraResolutionController *resolution_controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
