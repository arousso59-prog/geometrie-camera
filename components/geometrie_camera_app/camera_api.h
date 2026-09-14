#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometrieCameraApp;

class CameraApiHandler : public AsyncWebHandler {
 public:
  explicit CameraApiHandler(GeometrieCameraApp *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  GeometrieCameraApp *parent_{nullptr};

  void handle_status_(AsyncWebServerRequest *request);
  void handle_capture_(AsyncWebServerRequest *request);
  void handle_measure_(AsyncWebServerRequest *request);
  void handle_image_(AsyncWebServerRequest *request);
  void send_json_(AsyncWebServerRequest *request, const std::string &json) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
