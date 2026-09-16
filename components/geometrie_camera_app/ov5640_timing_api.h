#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class Ov5640TimingController;

class Ov5640TimingApiHandler : public AsyncWebHandler {
 public:
  explicit Ov5640TimingApiHandler(Ov5640TimingController *controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_status_(AsyncWebServerRequest *request) const;
  void handle_set_(AsyncWebServerRequest *request) const;
  void send_snapshot_(AsyncWebServerRequest *request, bool applied, const char *status, int response_code) const;

  Ov5640TimingController *controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
