#pragma once

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace geometrie_camera_app {

class JpegFilteredDiagnostic;
class TargetDetectionPreview;
class TargetDetectionService;

class TargetDetectionApiHandler : public AsyncWebHandler {
 public:
  TargetDetectionApiHandler(TargetDetectionService *service,
                            JpegFilteredDiagnostic *source,
                            TargetDetectionPreview *preview);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 private:
  void handle_detect_(AsyncWebServerRequest *request);
  void handle_status_(AsyncWebServerRequest *request) const;
  void handle_preview_(AsyncWebServerRequest *request);
  void send_status_(AsyncWebServerRequest *request, int response_code, const char *status,
                    const char *error = nullptr) const;

  TargetDetectionService *service_;
  JpegFilteredDiagnostic *source_;
  TargetDetectionPreview *preview_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
