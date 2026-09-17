#pragma once

#include <cstdint>
#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class ContinuousMeasurementController;
class TargetTrackingController;

class ContinuousMeasurementApiHandler : public AsyncWebHandler {
 public:
  ContinuousMeasurementApiHandler(ContinuousMeasurementController *controller,
                                  TargetTrackingController *tracking_controller);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  bool parse_interval_(AsyncWebServerRequest *request, uint32_t &interval_ms,
                       bool &present, std::string &error) const;
  bool parse_bool_option_(AsyncWebServerRequest *request, const char *name,
                          bool current_value, bool &value, std::string &error) const;
  void handle_start_(AsyncWebServerRequest *request);
  void handle_stop_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request) const;
  void send_snapshot_(AsyncWebServerRequest *request, int response_code,
                      const char *status, const char *error = nullptr) const;

  ContinuousMeasurementController *controller_;
  TargetTrackingController *tracking_controller_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
