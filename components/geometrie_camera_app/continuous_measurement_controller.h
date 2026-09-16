#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;
class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;

enum class ContinuousMeasurementState : uint8_t {
  STOPPED,
  REQUEST_CAPTURE,
  WAIT_CAPTURE,
  FILTER,
  DETECT,
  COMPUTE,
  WAIT_INTERVAL,
  ERROR,
};

class ContinuousMeasurementController {
 public:
  ContinuousMeasurementController(JpegDiagnostic *jpeg_source,
                                  JpegFilteredDiagnostic *filtered_source,
                                  TargetDetectionService *detection_service,
                                  MeasurementManager *measurement_manager);

  bool start(uint32_t interval_ms);
  void stop();
  void loop();
  bool set_interval_ms(uint32_t interval_ms);

  bool running() const;
  uint32_t interval_ms() const;
  ContinuousMeasurementState state() const;
  const char *state_text() const;
  uint32_t cycle_count() const;
  uint32_t target_found_count() const;
  uint32_t valid_measurement_count() const;
  uint32_t last_cycle_ms() const;
  bool last_cycle_target_found() const;
  bool last_cycle_measurement_valid() const;
  const std::string &last_error() const;

 private:
  void begin_cycle_();
  void finish_cycle_(bool target_found, bool measurement_valid);
  void fail_cycle_(const char *error);
  void stop_with_error_(const char *error);

  JpegDiagnostic *jpeg_source_;
  JpegFilteredDiagnostic *filtered_source_;
  TargetDetectionService *detection_service_;
  MeasurementManager *measurement_manager_;

  bool running_;
  uint32_t interval_ms_;
  ContinuousMeasurementState state_;
  uint32_t cycle_count_;
  uint32_t target_found_count_;
  uint32_t valid_measurement_count_;
  uint32_t cycle_started_ms_;
  uint32_t last_cycle_completed_ms_;
  uint32_t last_cycle_ms_;
  uint32_t capture_count_before_request_;
  bool last_cycle_target_found_;
  bool last_cycle_measurement_valid_;
  std::string last_error_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
