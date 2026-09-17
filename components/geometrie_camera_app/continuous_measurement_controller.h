#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace geometrie_camera_app {

class ImageSharpnessEvaluator;
class JpegDiagnostic;
class JpegFilteredDiagnostic;
class MeasurementManager;
class TargetDetectionService;

enum class ContinuousMeasurementState : uint8_t {
  STOPPED,
  REQUEST_CAPTURE,
  WAIT_CAPTURE,
  SHARPNESS,
  FILTER,
  DETECT,
  COMPUTE,
  WAIT_INTERVAL,
  ERROR,
};

class ContinuousMeasurementController {
 public:
  ContinuousMeasurementController(JpegDiagnostic *jpeg_source,
                                  ImageSharpnessEvaluator *sharpness_evaluator,
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

  // Ces valeurs sont un snapshot immuable du dernier cycle TERMINE.
  uint32_t last_capture_ms() const;
  uint32_t last_sharpness_ms() const;
  uint32_t last_filter_ms() const;
  uint32_t last_detect_ms() const;
  uint32_t last_compute_ms() const;
  uint32_t last_sharpness_score_x100() const;
  uint32_t sharpness_reference_score_x100() const;
  bool last_sharpness_ok() const;
  uint8_t last_capture_retry_count() const;
  uint32_t blur_retry_count() const;
  bool sharpness_roi_valid() const;
  uint16_t sharpness_roi_x() const;
  uint16_t sharpness_roi_y() const;
  uint16_t sharpness_roi_width() const;
  uint16_t sharpness_roi_height() const;

 private:
  void begin_cycle_();
  void publish_cycle_timing_(uint32_t cycle_ms);
  void finish_cycle_(bool target_found, bool measurement_valid);
  void fail_cycle_(const char *error);
  void stop_with_error_(const char *error);
  bool request_capture_();
  bool sharpness_is_too_low_(uint32_t score) const;
  void update_sharpness_reference_(uint32_t score);
  void update_sharpness_roi_from_target_();

  JpegDiagnostic *jpeg_source_;
  ImageSharpnessEvaluator *sharpness_evaluator_;
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

  // Chronometres du cycle actuellement en cours. Ils ne sont jamais exposes
  // directement par l'API afin d'eviter les valeurs partielles.
  uint32_t current_capture_ms_;
  uint32_t current_sharpness_ms_;
  uint32_t current_filter_ms_;
  uint32_t current_detect_ms_;
  uint32_t current_compute_ms_;

  // Snapshot du dernier cycle termine, publie par /continuous/status.
  uint32_t last_capture_ms_;
  uint32_t last_sharpness_ms_;
  uint32_t last_filter_ms_;
  uint32_t last_detect_ms_;
  uint32_t last_compute_ms_;

  uint32_t last_sharpness_score_x100_;
  uint32_t sharpness_reference_score_x100_;
  bool last_sharpness_ok_;
  uint8_t last_capture_retry_count_;
  uint32_t blur_retry_count_;

  bool sharpness_roi_valid_;
  uint16_t sharpness_roi_x_;
  uint16_t sharpness_roi_y_;
  uint16_t sharpness_roi_width_;
  uint16_t sharpness_roi_height_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
