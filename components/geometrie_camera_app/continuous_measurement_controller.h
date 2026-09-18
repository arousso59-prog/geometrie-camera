#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

class JpegDiagnostic;
class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;
class TargetTrackingController;

enum class ContinuousMeasurementState : uint8_t {
  STOPPED,
  REQUEST_CAPTURE,
  WAIT_CAPTURE,
  DECODE,
  DETECT,
  COMPUTE,
  WAIT_INTERVAL,
  ERROR,
};

class ContinuousMeasurementController {
 public:
  ContinuousMeasurementController(JpegDiagnostic *jpeg_source,
                                  JpegFilteredDiagnostic *decoded_source,
                                  TargetDetectionService *detection_service,
                                  MeasurementManager *measurement_manager,
                                  TargetTrackingController *tracking_controller);

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
  const std::string &last_cycle_viewport_mode() const;
  uint16_t last_cycle_viewport_x() const;
  uint16_t last_cycle_viewport_y() const;
  uint16_t last_cycle_viewport_width() const;
  uint16_t last_cycle_viewport_height() const;
  const std::string &last_error() const;

  uint32_t last_capture_ms() const;
  uint32_t last_decode_ms() const;
  uint32_t last_detect_ms() const;
  uint32_t last_compute_ms() const;
  bool last_capture_pipelined() const;

 private:
  void begin_cycle_();
  void publish_cycle_timing_(uint32_t cycle_ms);
  void finish_cycle_(bool target_found, bool measurement_valid);
  void fail_cycle_(const char *error);
  void stop_with_error_(const char *error);
  bool request_capture_();
  void reset_local_tracking_after_viewport_change_();

  JpegDiagnostic *jpeg_source_;
  JpegFilteredDiagnostic *decoded_source_;
  TargetDetectionService *detection_service_;
  MeasurementManager *measurement_manager_;
  TargetTrackingController *tracking_controller_;

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

  std::string current_cycle_viewport_mode_;
  uint16_t current_cycle_viewport_x_;
  uint16_t current_cycle_viewport_y_;
  uint16_t current_cycle_viewport_width_;
  uint16_t current_cycle_viewport_height_;
  std::string last_cycle_viewport_mode_;
  uint16_t last_cycle_viewport_x_;
  uint16_t last_cycle_viewport_y_;
  uint16_t last_cycle_viewport_width_;
  uint16_t last_cycle_viewport_height_;
  std::string last_error_;

  uint32_t current_capture_ms_;
  uint32_t current_decode_ms_;
  uint32_t current_detect_ms_;
  uint32_t current_compute_ms_;
  uint32_t last_capture_ms_;
  uint32_t last_decode_ms_;
  uint32_t last_detect_ms_;
  uint32_t last_compute_ms_;
  bool force_fresh_capture_;
  bool current_capture_pipelined_;
  bool last_capture_pipelined_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
