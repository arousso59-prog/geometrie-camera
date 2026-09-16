#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;
class RuntimeDiagnostics;

class RuntimeDiagnosticsApiHandler : public AsyncWebHandler {
 public:
  RuntimeDiagnosticsApiHandler(RuntimeDiagnostics *runtime, JpegDiagnostic *jpeg);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  RuntimeDiagnostics *runtime_;
  JpegDiagnostic *jpeg_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
