#pragma once

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "camera_api.h"
#include "camera_manager.h"
#include "measurement_manager.h"
#include "ov3660_camera_configurator.h"
#include "placeholder_image_provider.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class GeometrieCameraApp : public Component {
 public:
  GeometrieCameraApp();

  void setup() override;
  void loop() override;
  void dump_config() override;

  std::string status_text() const;
  uint32_t valid_measurement_count() const;
  const GeometryMeasurement &last_measurement() const;

  CameraManager &camera_manager();
  MeasurementManager &measurement_manager();
  TargetDetector &target_detector();
  GeometryMeasurementEngine &measurement_engine();
  Ov3660CameraConfigurator &camera_configurator();

 private:
  void register_api_if_possible_();

  PlaceholderImageProvider placeholder_image_provider_;
  CameraManager camera_manager_;
  MeasurementManager measurement_manager_;
  Ov3660CameraConfigurator camera_configurator_;
  CameraApiHandler api_handler_;
  bool api_registered_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
