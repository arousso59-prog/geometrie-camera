#pragma once

#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class FullCalibrationController;

class FullCalibrationApiHandler : public AsyncWebHandler {
 public:
  explicit FullCalibrationApiHandler(FullCalibrationController *controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  bool parse_float_param_(AsyncWebServerRequest *request, const char *name,
                          float &value, bool &present, std::string &error) const;
  bool parse_int_param_(AsyncWebServerRequest *request, const char *name,
                        int &value, bool &present, std::string &error) const;
  void send_status_(AsyncWebServerRequest *request, int response_code,
                    const char *status) const;

  FullCalibrationController *controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
