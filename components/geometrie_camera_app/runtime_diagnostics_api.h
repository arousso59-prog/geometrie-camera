#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class GrayscaleDiagnostic;
class JpegDiagnostic;
class RuntimeDiagnostics;
class TargetSearchDiagnostic;

class RuntimeDiagnosticsApiHandler : public AsyncWebHandler {
 public:
  RuntimeDiagnosticsApiHandler(RuntimeDiagnostics *runtime, GrayscaleDiagnostic *grayscale, JpegDiagnostic *jpeg,
                               TargetSearchDiagnostic *target_search);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  RuntimeDiagnostics *runtime_;
  GrayscaleDiagnostic *grayscale_;
  JpegDiagnostic *jpeg_;
  TargetSearchDiagnostic *target_search_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
