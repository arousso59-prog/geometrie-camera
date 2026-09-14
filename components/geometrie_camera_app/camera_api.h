#pragma once

#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraManager;
class MeasurementManager;

class CameraApiHandler : public AsyncWebHandler {
 public:
  CameraApiHandler(CameraManager *camera_manager, MeasurementManager *measurement_manager);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  CameraManager *camera_manager_;
  MeasurementManager *measurement_manager_;

  void handle_status_(AsyncWebServerRequest *request);
  void handle_capture_(AsyncWebServerRequest *request);
  void handle_measure_(AsyncWebServerRequest *request);
  void handle_image_(AsyncWebServerRequest *request);
  void send_json_(AsyncWebServerRequest *request, const std::string &json) const;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
