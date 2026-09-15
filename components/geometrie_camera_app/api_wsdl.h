#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController;

class ApiWsdlHandler : public AsyncWebHandler {
 public:
  explicit ApiWsdlHandler(CameraResolutionController *resolution_controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  CameraResolutionController *resolution_controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
