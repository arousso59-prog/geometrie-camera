#pragma once

#include <cstdint>
#include <string>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class ContinuousMeasurementController;
class JpegDiagnostic;
class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;
class TargetTrackingController;

enum class FullCalibrationState : uint8_t {
  IDLE,
  WAIT_IDLE,
  WAIT_CAPTURE,
  FILTER,
  DETECT,
  COMPLETE,
  ERROR,
};

class FullCalibrationController {
 public:
  static constexpr uint8_t DEFAULT_SAMPLE_COUNT = 10;
  static constexpr uint8_t MAX_SAMPLE_COUNT = 20;

  FullCalibrationController(JpegDiagnostic *jpeg_source,
                            JpegFilteredDiagnostic *filtered_source,
                            TargetDetectionService *detection_service,
                            MeasurementManager *measurement_manager,
                            ContinuousMeasurementController *continuous_controller,
                            TargetTrackingController *tracking_controller);

  bool start(float known_distance_mm, float target_size_mm,
             uint8_t sample_count = DEFAULT_SAMPLE_COUNT,
             bool force = false);
  void loop();
  void cancel();

  bool running() const;
  FullCalibrationState state() const;
  const char *state_text() const;
  const std::string &last_error() const;

  uint8_t requested_samples() const;
  uint8_t valid_samples() const;
  uint8_t attempts() const;
  uint8_t max_attempts() const;
  float known_distance_mm() const;
  float target_size_mm() const;
  float mean_fx_px() const;
  float mean_fy_px() const;
  float stddev_fx_px() const;
  float stddev_fy_px() const;
  const CameraCalibration &result_calibration() const;

 private:
  bool begin_native_tracking_();
  bool request_next_capture_();
  bool derive_current_sample_(const TargetObservation &reference_observation,
                              CameraCalibration &sample);
  void finish_success_();
  void fail_(const char *error);
  void restore_nominal_camera_();
  void restore_previous_measurement_config_();
  void reset_run_();

  JpegDiagnostic *jpeg_source_;
  JpegFilteredDiagnostic *filtered_source_;
  TargetDetectionService *detection_service_;
  MeasurementManager *measurement_manager_;
  ContinuousMeasurementController *continuous_controller_;
  TargetTrackingController *tracking_controller_;

  FullCalibrationState state_;
  std::string last_error_;

  float known_distance_mm_;
  float target_size_mm_;
  uint8_t requested_samples_;
  uint8_t valid_samples_;
  uint8_t attempts_;
  uint8_t max_attempts_;
  uint32_t capture_count_before_request_;
  uint32_t capture_started_ms_;

  CameraCalibration samples_[MAX_SAMPLE_COUNT];
  CameraCalibration result_calibration_;
  float mean_fx_px_;
  float mean_fy_px_;
  float stddev_fx_px_;
  float stddev_fy_px_;

  float previous_target_size_mm_;
  CameraCalibration previous_calibration_;
  bool previous_config_saved_;
  bool previous_tracking_enabled_;
  bool tracking_setting_saved_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
