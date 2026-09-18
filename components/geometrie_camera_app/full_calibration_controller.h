#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

#include "camera_settings_controller.h"
#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class ContinuousMeasurementController;
class JpegDiagnostic;
class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;
class TargetTrackingController;
class TargetDetectionPreview;

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
                            CameraSettingsController *settings_controller,
                            JpegFilteredDiagnostic *filtered_source,
                            TargetDetectionService *detection_service,
                            MeasurementManager *measurement_manager,
                            ContinuousMeasurementController *continuous_controller,
                            TargetTrackingController *tracking_controller,
                            TargetDetectionPreview *preview);

  bool start(float known_distance_mm,
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
  float mean_fx_px() const;
  float mean_fy_px() const;
  float stddev_fx_px() const;
  float stddev_fy_px() const;
  uint8_t preview_attempt() const;
  const std::string &preview_mode() const;
  bool last_target_found() const;
  bool last_sample_valid() const;
  float last_sample_fx_px() const;
  float last_sample_fy_px() const;
  const char *tracking_mode_text() const;
  const char *phase_text() const;
  uint8_t tuning_attempts() const;
  uint8_t tuning_max_attempts() const;
  int current_ae_level() const;
  int current_exposure() const;
  int current_gain() const;
  int current_brightness() const;
  int current_contrast() const;
  float current_optical_score() const;
  float current_detection_quality() const;
  float current_subpixel_rms_px() const;
  float current_width_gradient() const;
  float current_height_gradient() const;
  uint32_t current_mean_luma_x100() const;
  uint32_t current_dark_percent_x100() const;
  uint32_t current_bright_percent_x100() const;
  uint8_t current_p10_luma() const;
  uint8_t current_p90_luma() const;
  uint8_t current_contrast_luma() const;
  int best_ae_level() const;
  int best_exposure() const;
  int best_gain() const;
  int best_brightness() const;
  int best_contrast() const;
  float best_optical_score() const;
  bool using_auto_fallback() const;
  const CameraCalibration &result_calibration() const;

 private:
  enum class CalibrationPhase : uint8_t {
    TRACKING,
    TUNE_AUTO_SETTLE,
    TUNE_MANUAL_VALIDATE,
    TUNE_MANUAL_EXPOSURE,
    TUNE_MANUAL_GAIN,
    TUNE_CONTRAST,
    TUNE_BRIGHTNESS,
    SAMPLING,
  };

  static constexpr uint8_t OPTICAL_TUNING_MAX_ATTEMPTS = 19;
  static constexpr uint8_t AUTO_SETTLE_FRAMES = 4;
  static constexpr uint8_t AUTO_RECOVERY_FRAMES = 3;

  bool begin_native_tracking_();
  bool begin_optical_tuning_(const TargetObservation &observation);
  bool enable_auto_controls_();
  bool prepare_manual_candidate_(int exposure, int gain);
  bool prepare_postprocess_candidate_(int brightness, int contrast);
  bool handle_tuning_result_(const TargetObservation &observation, bool target_found);
  float evaluate_optical_score_(const TargetObservation &observation, bool target_found);
  bool start_manual_exposure_round_();
  bool start_manual_gain_round_();
  bool start_contrast_tuning_();
  bool start_brightness_tuning_();
  bool advance_manual_pair_(bool exposure_axis);
  bool finish_optical_tuning_();
  bool fallback_to_auto_sampling_(const char *reason);
  bool apply_camera_snapshot_(const CameraSettingsSnapshot &snapshot);
  void restore_previous_camera_settings_();
  bool request_next_capture_();
  bool derive_current_sample_(const TargetObservation &reference_observation,
                              CameraCalibration &sample);
  void update_running_stats_();
  void finish_success_();
  void fail_(const char *error);
  void restore_nominal_camera_();
  void restore_previous_measurement_config_();
  void reset_run_();

  JpegDiagnostic *jpeg_source_;
  CameraSettingsController *settings_controller_;
  JpegFilteredDiagnostic *filtered_source_;
  TargetDetectionService *detection_service_;
  MeasurementManager *measurement_manager_;
  ContinuousMeasurementController *continuous_controller_;
  TargetTrackingController *tracking_controller_;
  TargetDetectionPreview *preview_;

  FullCalibrationState state_;
  CalibrationPhase phase_;
  std::string last_error_;

  float known_distance_mm_;
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
  uint8_t preview_attempt_;
  std::string preview_mode_;
  bool last_target_found_;
  bool last_sample_valid_;
  float last_sample_fx_px_;
  float last_sample_fy_px_;

  uint8_t tuning_attempts_;
  uint8_t tuning_round_;
  uint8_t tuning_side_;
  uint8_t tuning_index_;
  int tuning_pair_base_;
  int tuning_step_;
  int tuning_pair_best_value_;
  float tuning_pair_best_score_;
  int current_ae_level_;
  int current_exposure_;
  int current_gain_;
  int current_brightness_;
  int current_contrast_;
  float current_optical_score_;
  float current_detection_quality_;
  float current_subpixel_rms_px_;
  float current_width_gradient_;
  float current_height_gradient_;
  uint32_t current_mean_luma_x100_;
  uint32_t current_dark_percent_x100_;
  uint32_t current_bright_percent_x100_;
  uint8_t current_p10_luma_;
  uint8_t current_p90_luma_;
  uint8_t current_contrast_luma_;
  int best_ae_level_;
  int best_exposure_;
  int best_gain_;
  int best_brightness_;
  int best_contrast_;
  float best_optical_score_;
  bool auto_fallback_;
  uint16_t tuning_roi_x_;
  uint16_t tuning_roi_y_;
  uint16_t tuning_roi_width_;
  uint16_t tuning_roi_height_;

  CameraCalibration previous_calibration_;
  bool previous_config_saved_;
  CameraSettingsSnapshot previous_camera_settings_;
  bool previous_camera_settings_saved_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
