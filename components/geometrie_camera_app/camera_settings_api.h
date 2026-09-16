#pragma once

#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController;
class CameraSettingsController;

class CameraSettingsApiHandler : public AsyncWebHandler {
 public:
  CameraSettingsApiHandler(CameraSettingsController *controller,
                           CameraResolutionController *resolution_controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_status_(AsyncWebServerRequest *request);
  void handle_set_(AsyncWebServerRequest *request);
  bool parse_int_param_(AsyncWebServerRequest *request, const char *name, int &value, bool &present,
                        std::string &error) const;
  bool parse_bool_param_(AsyncWebServerRequest *request, const char *name, bool &value, bool &present,
                         std::string &error) const;
  void send_snapshot_(AsyncWebServerRequest *request, bool applied, const char *status) const;

  CameraSettingsController *controller_;
  CameraResolutionController *resolution_controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
