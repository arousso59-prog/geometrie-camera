#pragma once

#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class TargetTrackingController;
class CameraViewportController;

class TargetTrackingApiHandler : public AsyncWebHandler {
 public:
  TargetTrackingApiHandler(TargetTrackingController *tracking_controller,
                           CameraViewportController *viewport_controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_status_(AsyncWebServerRequest *request) const;
  void handle_viewport_(AsyncWebServerRequest *request) const;
  void append_viewport_json_(std::string &json) const;

  TargetTrackingController *tracking_controller_;
  CameraViewportController *viewport_controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
