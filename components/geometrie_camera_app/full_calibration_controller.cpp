#include "full_calibration_controller.h"

#include "target_board_model.h"

#include <algorithm>
#include <cmath>

#include "camera_viewport_controller.h"
#include "continuous_measurement_controller.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "geometry_measurement.h"
#include "jpeg_diagnostic.h"
#include "jpeg_filtered_diagnostic.h"
#include "measurement_manager.h"
#include "target_detection_preview.h"
#include "target_detection_service.h"
#include "target_tracking_controller.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "full_calibration";
constexpr uint16_t CALIBRATION_REFERENCE_WIDTH = CameraViewportController::REFERENCE_WIDTH;
constexpr uint16_t CALIBRATION_REFERENCE_HEIGHT = CameraViewportController::REFERENCE_HEIGHT;
constexpr uint16_t NATIVE_OUTPUT_WIDTH = CameraViewportController::OUTPUT_WIDTH;
constexpr uint16_t NATIVE_OUTPUT_HEIGHT = CameraViewportController::OUTPUT_HEIGHT;
constexpr uint32_t CAPTURE_TIMEOUT_MS = 15000;
constexpr uint8_t MAX_TOTAL_ATTEMPTS = 100;

int clamp_int(int value, int minimum, int maximum) {
  return std::max(minimum, std::min(maximum, value));
}

float median_float(float *values, uint8_t count) {
  if (values == nullptr || count == 0) return 0.0f;
  for (uint8_t i = 1; i < count; ++i) {
    const float value = values[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
  if ((count & 1U) != 0U) return values[count / 2U];
  return 0.5f * (values[count / 2U - 1U] + values[count / 2U]);
}
}

FullCalibrationController::FullCalibrationController(
    JpegDiagnostic *jpeg_source,
    CameraSettingsController *settings_controller,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager,
    ContinuousMeasurementController *continuous_controller,
    TargetTrackingController *tracking_controller,
    TargetDetectionPreview *preview)
    : jpeg_source_(jpeg_source),
      settings_controller_(settings_controller),
      filtered_source_(filtered_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      continuous_controller_(continuous_controller),
      tracking_controller_(tracking_controller),
      preview_(preview),
      state_(FullCalibrationState::IDLE),
      phase_(CalibrationPhase::TRACKING),
      last_error_(),
      known_distance_mm_(0.0f),
      requested_samples_(DEFAULT_SAMPLE_COUNT),
      valid_samples_(0),
      attempts_(0),
      max_attempts_(DEFAULT_SAMPLE_COUNT * 5),
      capture_count_before_request_(0),
      capture_started_ms_(0),
      samples_{},
      result_calibration_(),
      mean_fx_px_(0.0f),
      mean_fy_px_(0.0f),
      stddev_fx_px_(0.0f),
      stddev_fy_px_(0.0f),
      preview_attempt_(0),
      preview_mode_("search"),
      last_target_found_(false),
      last_marker_count_(0),
      last_marker_mask_(0),
      last_board_complete_(false),
      last_sample_valid_(false),
      last_sample_fx_px_(0.0f),
      last_sample_fy_px_(0.0f),
      tuning_attempts_(0),
      tuning_round_(0),
      tuning_side_(0),
      tuning_index_(0),
      tuning_pair_base_(0),
      tuning_step_(0),
      tuning_pair_best_value_(0),
      tuning_pair_best_score_(-1.0f),
      current_ae_level_(0),
      current_exposure_(0),
      current_gain_(0),
      current_brightness_(0),
      current_contrast_(0),
      current_optical_score_(0.0f),
      current_detection_quality_(0.0f),
      current_subpixel_rms_px_(0.0f),
      current_width_gradient_(0.0f),
      current_height_gradient_(0.0f),
      current_mean_luma_x100_(0),
      current_dark_percent_x100_(0),
      current_bright_percent_x100_(0),
      current_p10_luma_(0),
      current_p90_luma_(0),
      current_contrast_luma_(0),
      best_ae_level_(0),
      best_exposure_(0),
      best_gain_(0),
      best_brightness_(0),
      best_contrast_(0),
      best_optical_score_(-1.0f),
      auto_fallback_(false),
      tuning_roi_x_(0),
      tuning_roi_y_(0),
      tuning_roi_width_(0),
      tuning_roi_height_(0),
      previous_calibration_(),
      previous_config_saved_(false),
      previous_camera_settings_(),
      previous_camera_settings_saved_(false) {}

bool FullCalibrationController::start(float known_distance_mm,
                                      uint8_t sample_count,
                                      bool force) {
  if (this->running()) {
    this->last_error_ = "calibration_already_running";
    return false;
  }
  if (this->jpeg_source_ == nullptr ||
      this->settings_controller_ == nullptr ||
      this->filtered_source_ == nullptr ||
      this->detection_service_ == nullptr ||
      this->measurement_manager_ == nullptr ||
      this->tracking_controller_ == nullptr ||
      !this->tracking_controller_->supported()) {
    this->last_error_ = "calibration_dependencies_unavailable";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (!std::isfinite(known_distance_mm) || known_distance_mm < 50.0f ||
      known_distance_mm > 20000.0f) {
    this->last_error_ = "distance_mm_out_of_range";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }
  if (sample_count < 3 || sample_count > MAX_SAMPLE_COUNT) {
    this->last_error_ = "sample_count_out_of_range";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  if (engine.has_calibration() && !force) {
    this->last_error_ = "calibration_locked";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }

  if (this->continuous_controller_ != nullptr && this->continuous_controller_->running()) {
    this->continuous_controller_->stop();
  }

  this->previous_calibration_ = engine.calibration();
  this->previous_config_saved_ = true;

  this->previous_camera_settings_ = this->settings_controller_->read();
  this->previous_camera_settings_saved_ = this->previous_camera_settings_.available;
  if (!this->previous_camera_settings_saved_) {
    this->last_error_ = "camera_settings_unavailable";
    this->state_ = FullCalibrationState::ERROR;
    return false;
  }

  this->reset_run_();
  this->known_distance_mm_ = known_distance_mm;
  this->requested_samples_ = sample_count;
  this->max_attempts_ = std::min<uint8_t>(
      MAX_TOTAL_ATTEMPTS,
      static_cast<uint8_t>(std::max<int>(
          sample_count * 5 + OPTICAL_TUNING_MAX_ATTEMPTS,
          sample_count + 24 + OPTICAL_TUNING_MAX_ATTEMPTS)));

  ESP_LOGI(TAG,
           "Calibration R1 demandee: PRECISE=%ux%u, distance=%.2f mm, "
           "cible fixe=250x100 mm reference=240x90 mm, marqueurs A+B+C, "
           "echantillons=%u, auto-reglage optique <=%u prises",
           static_cast<unsigned>(NATIVE_OUTPUT_WIDTH),
           static_cast<unsigned>(NATIVE_OUTPUT_HEIGHT),
           known_distance_mm,
           static_cast<unsigned>(sample_count),
           static_cast<unsigned>(OPTICAL_TUNING_MAX_ATTEMPTS));

  if (this->jpeg_source_->capture_pending()) {
    this->capture_started_ms_ = millis();
    this->state_ = FullCalibrationState::WAIT_IDLE;
    return true;
  }

  if (!this->begin_native_tracking_()) {
    this->fail_("native_tracking_start_failed");
    return false;
  }
  return true;
}

void FullCalibrationController::loop() {
  switch (this->state_) {
    case FullCalibrationState::WAIT_IDLE: {
      const uint32_t now = millis();
      if (this->jpeg_source_->capture_pending()) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("camera_idle_timeout");
        }
        return;
      }
      if (!this->begin_native_tracking_()) {
        this->fail_("native_tracking_start_failed");
      }
      return;
    }

    case FullCalibrationState::WAIT_CAPTURE: {
      const uint32_t now = millis();
      if (this->jpeg_source_->capture_pending()) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("capture_timeout");
        }
        return;
      }
      if (!this->jpeg_source_->ready() ||
          this->jpeg_source_->capture_count() <= this->capture_count_before_request_) {
        if (now - this->capture_started_ms_ > CAPTURE_TIMEOUT_MS) {
          this->fail_("fresh_capture_not_received");
        }
        return;
      }
      if (this->jpeg_source_->width() != NATIVE_OUTPUT_WIDTH ||
          this->jpeg_source_->height() != NATIVE_OUTPUT_HEIGHT) {
        this->fail_("unexpected_tracking_output_resolution");
        return;
      }
      this->state_ = FullCalibrationState::FILTER;
      return;
    }

    case FullCalibrationState::FILTER:
      if (!this->filtered_source_->process()) {
        this->fail_("calibration_filter_failed");
        return;
      }
      this->state_ = FullCalibrationState::DETECT;
      return;

    case FullCalibrationState::DETECT: {
      this->attempts_++;

      const CameraViewportMode mode_before =
          this->tracking_controller_->viewport_controller()->snapshot().mode;
      const bool high_precision_detection =
          mode_before == CameraViewportMode::PRECISE_ROI;

      if (!this->detection_service_->detect(high_precision_detection)) {
        this->fail_("calibration_detection_failed");
        return;
      }

      const bool target_found = this->detection_service_->target_found();
      const TargetObservation local_observation =
          this->detection_service_->last_observation();
      const bool precision_target_found =
          target_found && local_observation.board_complete &&
          local_observation.marker_id == TargetMarkerId::BOARD_R1 &&
          local_observation.subpixel_refined;

      this->last_target_found_ = target_found;
      this->last_marker_count_ = local_observation.board_marker_count;
      this->last_marker_mask_ = local_observation.board_marker_mask;
      this->last_board_complete_ = local_observation.board_complete;
      this->last_sample_valid_ = false;
      this->last_sample_fx_px_ = 0.0f;
      this->last_sample_fy_px_ = 0.0f;

      if (this->preview_ != nullptr &&
          this->preview_->render(this->filtered_source_, local_observation)) {
        this->preview_attempt_ = this->attempts_;
      }

      if (this->preview_attempt_ == this->attempts_) {
        this->preview_mode_ = CameraViewportController::mode_text(mode_before);
      }

      // Pendant la recherche du PRECISE et pendant les mesures finales, le
      // tracking reste actif. Pendant l'optimisation optique, on gele le ROI
      // PRECISE afin que tous les candidats soient compares sur exactement la
      // meme zone du capteur.
      if (this->phase_ == CalibrationPhase::TRACKING ||
          this->phase_ == CalibrationPhase::SAMPLING) {
        const TrackingUpdateResult tracking_result =
            this->tracking_controller_->update_after_detection(
                target_found, local_observation);

        if (tracking_result == TrackingUpdateResult::ERROR) {
          this->fail_("calibration_tracking_failed");
          return;
        }
        if (tracking_result == TrackingUpdateResult::VIEWPORT_CHANGED) {
          if (this->attempts_ >= this->max_attempts_) {
            this->fail_("precise_lock_timeout");
            return;
          }
          if (!this->request_next_capture_()) {
            this->fail_("capture_request_failed");
          }
          return;
        }
      }

      if (this->phase_ == CalibrationPhase::TRACKING) {
        if (precision_target_found &&
            mode_before == CameraViewportMode::PRECISE_ROI &&
            this->tracking_controller_->target_locked()) {
          if (!this->begin_optical_tuning_(local_observation)) {
            this->fail_("optical_tuning_start_failed");
            return;
          }
          if (!this->request_next_capture_()) {
            this->fail_("capture_request_failed");
          }
          return;
        }

        if (this->attempts_ >= this->max_attempts_) {
          this->fail_("precise_lock_timeout");
          return;
        }
        if (!this->request_next_capture_()) {
          this->fail_("capture_request_failed");
        }
        return;
      }

      if (this->phase_ != CalibrationPhase::SAMPLING) {
        if (!this->handle_tuning_result_(
                local_observation, precision_target_found)) {
          this->fail_("optical_tuning_failed");
        }
        return;
      }

      if (precision_target_found &&
          mode_before == CameraViewportMode::PRECISE_ROI &&
          this->tracking_controller_->target_locked()) {
        const TargetObservation reference_observation =
            this->tracking_controller_->to_reference(local_observation);

        CameraCalibration sample;
        if (this->derive_current_sample_(reference_observation, sample)) {
          this->samples_[this->valid_samples_] = sample;
          this->valid_samples_++;
          this->last_sample_valid_ = true;
          this->last_sample_fx_px_ = sample.fx_px;
          this->last_sample_fy_px_ = sample.fy_px;
          this->update_running_stats_();
          ESP_LOGI(TAG,
                   "Calibration R1 PRECISE: echantillon %u/%u fx=%.3f fy=%.3f "
                   "reference=%.2fx%.2f px qualite=%.3f",
                   static_cast<unsigned>(this->valid_samples_),
                   static_cast<unsigned>(this->requested_samples_),
                   sample.fx_px, sample.fy_px,
                   reference_observation.subpixel_width_px > 0.0f
                       ? reference_observation.subpixel_width_px
                       : reference_observation.width_px,
                   reference_observation.subpixel_height_px > 0.0f
                       ? reference_observation.subpixel_height_px
                       : reference_observation.height_px,
                   local_observation.quality);
        }
      }

      if (this->valid_samples_ >= this->requested_samples_) {
        this->finish_success_();
        return;
      }
      if (this->attempts_ >= this->max_attempts_) {
        this->fail_("not_enough_valid_precise_samples");
        return;
      }
      if (!this->request_next_capture_()) {
        this->fail_("capture_request_failed");
      }
      return;
    }

    case FullCalibrationState::IDLE:
    case FullCalibrationState::COMPLETE:
    case FullCalibrationState::ERROR:
    default:
      return;
  }
}

void FullCalibrationController::cancel() {
  if (!this->running()) return;
  this->restore_previous_measurement_config_();
  this->restore_previous_camera_settings_();
  this->restore_nominal_camera_();
  this->last_error_ = "cancelled";
  this->state_ = FullCalibrationState::IDLE;
}

bool FullCalibrationController::running() const {
  return this->state_ == FullCalibrationState::WAIT_IDLE ||
         this->state_ == FullCalibrationState::WAIT_CAPTURE ||
         this->state_ == FullCalibrationState::FILTER ||
         this->state_ == FullCalibrationState::DETECT;
}

FullCalibrationState FullCalibrationController::state() const { return this->state_; }

const char *FullCalibrationController::state_text() const {
  switch (this->state_) {
    case FullCalibrationState::IDLE: return "idle";
    case FullCalibrationState::WAIT_IDLE: return "wait_idle";
    case FullCalibrationState::WAIT_CAPTURE: return "wait_capture";
    case FullCalibrationState::FILTER: return "filter";
    case FullCalibrationState::DETECT: return "detect";
    case FullCalibrationState::COMPLETE: return "complete";
    case FullCalibrationState::ERROR: return "error";
    default: return "unknown";
  }
}

const char *FullCalibrationController::phase_text() const {
  switch (this->phase_) {
    case CalibrationPhase::TRACKING: return "tracking";
    case CalibrationPhase::TUNE_AUTO_SETTLE: return "optical_auto_settle";
    case CalibrationPhase::TUNE_MANUAL_VALIDATE: return "optical_manual_validate";
    case CalibrationPhase::TUNE_MANUAL_EXPOSURE: return "optical_exposure";
    case CalibrationPhase::TUNE_MANUAL_GAIN: return "optical_gain";
    case CalibrationPhase::TUNE_CONTRAST: return "optical_contrast";
    case CalibrationPhase::TUNE_BRIGHTNESS: return "optical_brightness";
    case CalibrationPhase::SAMPLING: return "calibration_samples";
    default: return "unknown";
  }
}

const std::string &FullCalibrationController::last_error() const { return this->last_error_; }
uint8_t FullCalibrationController::requested_samples() const { return this->requested_samples_; }
uint8_t FullCalibrationController::valid_samples() const { return this->valid_samples_; }
uint8_t FullCalibrationController::attempts() const { return this->attempts_; }
uint8_t FullCalibrationController::max_attempts() const { return this->max_attempts_; }
float FullCalibrationController::known_distance_mm() const { return this->known_distance_mm_; }
float FullCalibrationController::mean_fx_px() const { return this->mean_fx_px_; }
float FullCalibrationController::mean_fy_px() const { return this->mean_fy_px_; }
float FullCalibrationController::stddev_fx_px() const { return this->stddev_fx_px_; }
float FullCalibrationController::stddev_fy_px() const { return this->stddev_fy_px_; }
uint8_t FullCalibrationController::preview_attempt() const { return this->preview_attempt_; }
const std::string &FullCalibrationController::preview_mode() const { return this->preview_mode_; }
bool FullCalibrationController::last_target_found() const { return this->last_target_found_; }
uint8_t FullCalibrationController::last_marker_count() const { return this->last_marker_count_; }
uint8_t FullCalibrationController::last_marker_mask() const { return this->last_marker_mask_; }
bool FullCalibrationController::last_board_complete() const { return this->last_board_complete_; }
bool FullCalibrationController::last_sample_valid() const { return this->last_sample_valid_; }
float FullCalibrationController::last_sample_fx_px() const { return this->last_sample_fx_px_; }
float FullCalibrationController::last_sample_fy_px() const { return this->last_sample_fy_px_; }
uint8_t FullCalibrationController::tuning_attempts() const { return this->tuning_attempts_; }
uint8_t FullCalibrationController::tuning_max_attempts() const { return OPTICAL_TUNING_MAX_ATTEMPTS; }
int FullCalibrationController::current_ae_level() const { return this->current_ae_level_; }
int FullCalibrationController::current_exposure() const { return this->current_exposure_; }
int FullCalibrationController::current_gain() const { return this->current_gain_; }
int FullCalibrationController::current_brightness() const { return this->current_brightness_; }
int FullCalibrationController::current_contrast() const { return this->current_contrast_; }
float FullCalibrationController::current_optical_score() const { return this->current_optical_score_; }
float FullCalibrationController::current_detection_quality() const { return this->current_detection_quality_; }
float FullCalibrationController::current_subpixel_rms_px() const { return this->current_subpixel_rms_px_; }
float FullCalibrationController::current_width_gradient() const { return this->current_width_gradient_; }
float FullCalibrationController::current_height_gradient() const { return this->current_height_gradient_; }
uint32_t FullCalibrationController::current_mean_luma_x100() const { return this->current_mean_luma_x100_; }
uint32_t FullCalibrationController::current_dark_percent_x100() const { return this->current_dark_percent_x100_; }
uint32_t FullCalibrationController::current_bright_percent_x100() const { return this->current_bright_percent_x100_; }
uint8_t FullCalibrationController::current_p10_luma() const { return this->current_p10_luma_; }
uint8_t FullCalibrationController::current_p90_luma() const { return this->current_p90_luma_; }
uint8_t FullCalibrationController::current_contrast_luma() const { return this->current_contrast_luma_; }
int FullCalibrationController::best_ae_level() const { return this->best_ae_level_; }
int FullCalibrationController::best_exposure() const { return this->best_exposure_; }
int FullCalibrationController::best_gain() const { return this->best_gain_; }
int FullCalibrationController::best_brightness() const { return this->best_brightness_; }
int FullCalibrationController::best_contrast() const { return this->best_contrast_; }
float FullCalibrationController::best_optical_score() const { return this->best_optical_score_; }
bool FullCalibrationController::using_auto_fallback() const { return this->auto_fallback_; }
const char *FullCalibrationController::tracking_mode_text() const {
  return this->tracking_controller_ != nullptr
             ? this->tracking_controller_->mode_text()
             : "unavailable";
}
const CameraCalibration &FullCalibrationController::result_calibration() const {
  return this->result_calibration_;
}

bool FullCalibrationController::begin_native_tracking_() {
  if (this->tracking_controller_->active()) this->tracking_controller_->stop();
  if (!this->tracking_controller_->start()) return false;
  this->detection_service_->reset_tracking();
  this->phase_ = CalibrationPhase::TRACKING;
  ESP_LOGI(TAG, "Calibration: SEARCH actif, progression vers PRECISE avant reglage optique");
  return this->request_next_capture_();
}

bool FullCalibrationController::begin_optical_tuning_(
    const TargetObservation &observation) {
  const float margin_x = std::max(4.0f, observation.width_px * 0.08f);
  const float margin_y = std::max(4.0f, observation.height_px * 0.08f);
  const int left = clamp_int(
      static_cast<int>(std::floor(observation.center_x_px -
                                  observation.width_px * 0.5f - margin_x)),
      0, NATIVE_OUTPUT_WIDTH - 4);
  const int top = clamp_int(
      static_cast<int>(std::floor(observation.center_y_px -
                                  observation.height_px * 0.5f - margin_y)),
      0, NATIVE_OUTPUT_HEIGHT - 4);
  const int right = clamp_int(
      static_cast<int>(std::ceil(observation.center_x_px +
                                 observation.width_px * 0.5f + margin_x)),
      left + 4, NATIVE_OUTPUT_WIDTH);
  const int bottom = clamp_int(
      static_cast<int>(std::ceil(observation.center_y_px +
                                 observation.height_px * 0.5f + margin_y)),
      top + 4, NATIVE_OUTPUT_HEIGHT);

  this->tuning_roi_x_ = static_cast<uint16_t>(left);
  this->tuning_roi_y_ = static_cast<uint16_t>(top);
  this->tuning_roi_width_ = static_cast<uint16_t>(right - left);
  this->tuning_roi_height_ = static_cast<uint16_t>(bottom - top);

  this->tuning_attempts_ = 0;
  this->tuning_round_ = 0;
  this->tuning_side_ = 0;
  this->tuning_index_ = 0;
  this->best_optical_score_ = -1.0f;
  this->best_exposure_ = 0;
  this->best_gain_ = 0;
  this->best_brightness_ = 0;
  this->best_contrast_ = 0;
  this->current_brightness_ = 0;
  this->current_contrast_ = 0;
  this->auto_fallback_ = false;

  const CameraSettingsSnapshot baseline = this->settings_controller_->read();
  this->best_ae_level_ =
      baseline.available ? clamp_int(baseline.ae_level, -2, 2) : 0;
  this->current_ae_level_ = this->best_ae_level_;

  if (!this->enable_auto_controls_()) {
    return false;
  }

  this->phase_ = CalibrationPhase::TUNE_AUTO_SETTLE;

  ESP_LOGI(TAG,
           "Auto-reglage metrologique: ROI %ux%u @%u,%u; stabilisation AEC/AGC "
           "auto sur %u images, lecture registres reels OV5640 puis validation "
           "du verrouillage manuel",
           static_cast<unsigned>(this->tuning_roi_width_),
           static_cast<unsigned>(this->tuning_roi_height_),
           static_cast<unsigned>(this->tuning_roi_x_),
           static_cast<unsigned>(this->tuning_roi_y_),
           static_cast<unsigned>(AUTO_SETTLE_FRAMES));

  // La boucle DETECT demandera l'unique capture suivante.
  return true;
}

bool FullCalibrationController::enable_auto_controls_() {
  if (this->settings_controller_ == nullptr) return false;

  std::string error;
  if (!this->settings_controller_->set_brightness(0, error) ||
      !this->settings_controller_->set_contrast(0, error) ||
      !this->settings_controller_->set_ae_level(this->best_ae_level_, error) ||
      !this->settings_controller_->set_exposure_ctrl(true, error) ||
      !this->settings_controller_->set_gain_ctrl(true, error)) {
    ESP_LOGE(TAG, "Activation AEC/AGC auto impossible: %s", error.c_str());
    return false;
  }

  this->current_brightness_ = 0;
  this->current_contrast_ = 0;
  return true;
}

bool FullCalibrationController::prepare_manual_candidate_(int exposure, int gain) {
  std::string error;
  const int bounded_exposure = clamp_int(exposure, 0, 65535);
  const int bounded_gain = clamp_int(gain, 0, 64);

  if (!this->settings_controller_->set_exposure_ctrl(false, error) ||
      !this->settings_controller_->set_gain_ctrl(false, error) ||
      !this->settings_controller_->set_ae_level(this->best_ae_level_, error) ||
      !this->settings_controller_->set_aec_value(bounded_exposure, error) ||
      !this->settings_controller_->set_agc_gain(bounded_gain, error)) {
    ESP_LOGE(TAG, "Reglage manuel camera impossible: %s", error.c_str());
    return false;
  }

  this->current_ae_level_ = this->best_ae_level_;
  this->current_exposure_ = bounded_exposure;
  this->current_gain_ = bounded_gain;
  return true;
}

bool FullCalibrationController::prepare_postprocess_candidate_(
    int brightness, int contrast) {
  if (this->settings_controller_ == nullptr) return false;

  const int bounded_brightness = clamp_int(brightness, -2, 2);
  const int bounded_contrast = clamp_int(contrast, -2, 2);
  std::string error;
  if (!this->settings_controller_->set_brightness(bounded_brightness, error) ||
      !this->settings_controller_->set_contrast(bounded_contrast, error)) {
    ESP_LOGE(TAG, "Reglage post-traitement camera impossible: %s", error.c_str());
    return false;
  }

  this->current_brightness_ = bounded_brightness;
  this->current_contrast_ = bounded_contrast;
  return true;
}

float FullCalibrationController::evaluate_optical_score_(
    const TargetObservation &observation, bool target_found) {
  this->current_detection_quality_ = target_found ? observation.quality : 0.0f;
  this->current_subpixel_rms_px_ =
      target_found && observation.subpixel_refined
          ? observation.subpixel_rms_px
          : 2.0f;
  this->current_width_gradient_ =
      target_found && observation.subpixel_refined
          ? observation.subpixel_width_gradient
          : 0.0f;
  this->current_height_gradient_ =
      target_found && observation.subpixel_refined
          ? observation.subpixel_height_gradient
          : 0.0f;
  this->current_mean_luma_x100_ = 0;
  this->current_dark_percent_x100_ = 0;
  this->current_bright_percent_x100_ = 0;
  this->current_p10_luma_ = 0;
  this->current_p90_luma_ = 0;
  this->current_contrast_luma_ = 0;

  if (this->filtered_source_ == nullptr ||
      !this->filtered_source_->ready() ||
      this->filtered_source_->grayscale_data() == nullptr ||
      this->tuning_roi_width_ == 0 || this->tuning_roi_height_ == 0) {
    return 0.0f;
  }

  const uint8_t *pixels = this->filtered_source_->grayscale_data();
  const size_t stride = this->filtered_source_->grayscale_stride();
  uint32_t histogram[256] = {};
  uint64_t sum = 0;
  uint32_t dark = 0;
  uint32_t bright = 0;
  uint32_t count = 0;

  const uint16_t x_end = std::min<uint16_t>(
      this->filtered_source_->width(),
      static_cast<uint16_t>(this->tuning_roi_x_ + this->tuning_roi_width_));
  const uint16_t y_end = std::min<uint16_t>(
      this->filtered_source_->height(),
      static_cast<uint16_t>(this->tuning_roi_y_ + this->tuning_roi_height_));

  for (uint16_t y = this->tuning_roi_y_; y < y_end; ++y) {
    const uint8_t *row = pixels + static_cast<size_t>(y) * stride;
    for (uint16_t x = this->tuning_roi_x_; x < x_end; ++x) {
      const uint8_t value = row[x];
      histogram[value]++;
      sum += value;
      if (value <= 20) dark++;
      if (value >= 235) bright++;
      count++;
    }
  }

  if (count < 16) return 0.0f;

  this->current_mean_luma_x100_ =
      static_cast<uint32_t>((sum * 100ULL) / count);
  this->current_dark_percent_x100_ =
      static_cast<uint32_t>((static_cast<uint64_t>(dark) * 10000ULL) / count);
  this->current_bright_percent_x100_ =
      static_cast<uint32_t>((static_cast<uint64_t>(bright) * 10000ULL) / count);

  const uint32_t p10_target = std::max<uint32_t>(1, count / 10U);
  const uint32_t p90_target = std::max<uint32_t>(1, (count * 9U) / 10U);
  uint32_t cumulative = 0;
  bool p10_set = false;
  for (uint16_t value = 0; value < 256; ++value) {
    cumulative += histogram[value];
    if (!p10_set && cumulative >= p10_target) {
      this->current_p10_luma_ = static_cast<uint8_t>(value);
      p10_set = true;
    }
    if (cumulative >= p90_target) {
      this->current_p90_luma_ = static_cast<uint8_t>(value);
      break;
    }
  }
  this->current_contrast_luma_ =
      this->current_p90_luma_ >= this->current_p10_luma_
          ? static_cast<uint8_t>(this->current_p90_luma_ -
                                 this->current_p10_luma_)
          : 0;

  // Rejet dur des images manifestement inutilisables. Le ROI contient le
  // marqueur noir et son fond blanc : si P10 est deja presque blanc, ou P90
  // presque noir, la cible ne peut plus porter une information subpixel
  // exploitable. Cela empeche surtout un candidat "tout blanc" de devenir un
  // point de recentrage de la recherche.
  if (this->current_p10_luma_ >= 235 ||
      this->current_p90_luma_ <= 20 ||
      this->current_contrast_luma_ < 12) {
    return 0.0f;
  }

  // Un candidat sans detection subpixel fiable ne peut pas devenir le
  // meilleur profil optique.
  if (!target_found || !observation.subpixel_refined) return 0.0f;

  const float quality_factor =
      0.55f + 0.45f *
                  std::max(0.0f, std::min(1.0f, observation.quality));
  const float rms = std::max(0.0f, this->current_subpixel_rms_px_);
  const float rms_factor = 1.0f / (1.0f + 1.5f * rms);

  const float worst_axis_sigma =
      std::max(observation.subpixel_width_sigma_px,
               observation.subpixel_height_sigma_px);
  const float axis_precision_factor =
      worst_axis_sigma > 0.0f
          ? 1.0f / (1.0f + 8.0f * worst_axis_sigma)
          : 0.55f;

  // Pour la metrologie, la qualite utile n'est pas une "nettete" generale
  // de l'image mais la pente reelle des transitions noir/blanc des quatre
  // bords. Favoriser un gradient fort ET equilibre entre largeur et hauteur
  // evite de choisir une exposition excellente horizontalement mais molle
  // verticalement.
  const float width_gradient =
      std::max(0.0f, observation.subpixel_width_gradient);
  const float height_gradient =
      std::max(0.0f, observation.subpixel_height_gradient);
  const float min_axis_gradient =
      std::min(width_gradient, height_gradient);
  const float max_axis_gradient =
      std::max(width_gradient, height_gradient);
  const float geometric_gradient =
      std::sqrt(width_gradient * height_gradient);
  const float gradient_factor =
      std::max(0.50f, std::min(1.35f, geometric_gradient / 32.0f));
  const float gradient_balance_factor =
      max_axis_gradient > 0.0f
          ? std::max(0.60f,
                     std::min(1.0f, min_axis_gradient / max_axis_gradient))
          : 0.60f;

  const float white_factor = std::max(
      0.05f, std::min(1.15f,
                      static_cast<float>(this->current_p90_luma_) / 210.0f));
  const float black_factor = std::max(
      0.10f, std::min(1.0f,
                      (220.0f - static_cast<float>(this->current_p10_luma_)) /
                          180.0f));
  const float contrast_factor = std::max(
      0.05f, std::min(1.20f,
                      static_cast<float>(this->current_contrast_luma_) /
                          175.0f));
  const float clipping_percent =
      static_cast<float>(this->current_dark_percent_x100_ +
                         this->current_bright_percent_x100_) /
      100.0f;
  const float clipping_factor =
      1.0f / (1.0f + 0.04f * std::max(0.0f, clipping_percent - 8.0f));
  const float gain_factor =
      1.0f /
      (1.0f + 0.025f *
                  static_cast<float>(std::max(0, this->current_gain_)));

  // Echelle arbitraire mais stable : la geometrie subpixel et la dynamique
  // de luminance forment le score d'optimisation.
  return 10000.0f * quality_factor * rms_factor *
         axis_precision_factor * gradient_factor *
         gradient_balance_factor * white_factor * black_factor *
         contrast_factor * clipping_factor * gain_factor;
}

bool FullCalibrationController::handle_tuning_result_(
    const TargetObservation &observation, bool target_found) {
  this->tuning_attempts_++;

  if (this->phase_ == CalibrationPhase::TUNE_AUTO_SETTLE) {
    std::string live_error;
    int live_exposure = 0;
    int live_gain = 0;
    if (!this->settings_controller_->read_live_exposure_gain(
            live_exposure, live_gain, live_error)) {
      ESP_LOGE(TAG, "Lecture AEC/AGC reels impossible: %s", live_error.c_str());
      return false;
    }
    this->current_exposure_ = live_exposure;
    this->current_gain_ = live_gain;
  } else {
    const CameraSettingsSnapshot actual = this->settings_controller_->read();
    if (actual.available) {
      this->current_exposure_ = actual.aec_value;
      this->current_gain_ = actual.agc_gain;
      this->current_brightness_ = actual.brightness;
      this->current_contrast_ = actual.contrast;
    }
  }

  this->current_optical_score_ =
      this->evaluate_optical_score_(observation, target_found);

  ESP_LOGI(TAG,
           "Optique %u/%u phase=%s AE=%d exp=%d gain=%d lum=%d ctr=%d score=%.1f "
           "qual=%.3f rms=%.3f sigmaW=%.3f sigmaH=%.3f gradW=%.1f gradH=%.1f "
           "P10=%u P90=%u C=%u luma=%.1f clip=%.1f%%",
           static_cast<unsigned>(this->tuning_attempts_),
           static_cast<unsigned>(OPTICAL_TUNING_MAX_ATTEMPTS),
           this->phase_text(), this->current_ae_level_,
           this->current_exposure_, this->current_gain_,
           this->current_brightness_, this->current_contrast_,
           this->current_optical_score_,
           this->current_detection_quality_,
           this->current_subpixel_rms_px_,
           observation.subpixel_width_sigma_px,
           observation.subpixel_height_sigma_px,
           observation.subpixel_width_gradient,
           observation.subpixel_height_gradient,
           static_cast<unsigned>(this->current_p10_luma_),
           static_cast<unsigned>(this->current_p90_luma_),
           static_cast<unsigned>(this->current_contrast_luma_),
           static_cast<float>(this->current_mean_luma_x100_) / 100.0f,
           static_cast<float>(this->current_dark_percent_x100_ +
                              this->current_bright_percent_x100_) /
               100.0f);

  if (this->phase_ == CalibrationPhase::TUNE_AUTO_SETTLE) {
    if (this->current_optical_score_ > this->best_optical_score_) {
      this->best_optical_score_ = this->current_optical_score_;
      this->best_exposure_ = this->current_exposure_;
      this->best_gain_ = this->current_gain_;
      this->best_brightness_ = 0;
      this->best_contrast_ = 0;
    }

    this->tuning_index_++;
    const uint8_t settle_target =
        this->auto_fallback_ ? AUTO_RECOVERY_FRAMES : AUTO_SETTLE_FRAMES;
    if (this->tuning_index_ < settle_target) {
      return this->request_next_capture_();
    }

    if (this->best_optical_score_ <= 0.0f ||
        this->best_exposure_ <= 0 || this->best_gain_ < 0) {
      ESP_LOGE(TAG,
               "AEC/AGC auto inutilisable: aucune image metrologique valide");
      return false;
    }

    if (this->auto_fallback_) {
      this->phase_ = CalibrationPhase::SAMPLING;
      this->current_optical_score_ = this->best_optical_score_;
      ESP_LOGW(TAG,
               "Calibration poursuivie en AEC/AGC AUTO: exp reel=%d gain reel=%d "
               "score=%.1f; verrouillage manuel abandonne",
               this->best_exposure_, this->best_gain_,
               this->best_optical_score_);
      return this->request_next_capture_();
    }

    this->phase_ = CalibrationPhase::TUNE_MANUAL_VALIDATE;
    ESP_LOGI(TAG,
             "AEC/AGC stabilises: verrouillage test sur exp reel=%d gain reel=%d",
             this->best_exposure_, this->best_gain_);
    if (!this->prepare_manual_candidate_(
            this->best_exposure_, this->best_gain_)) {
      return this->fallback_to_auto_sampling_("manual_apply_failed");
    }
    return this->request_next_capture_();
  }

  if (this->phase_ == CalibrationPhase::TUNE_MANUAL_VALIDATE) {
    if (this->current_optical_score_ <= 0.0f ||
        !target_found || !observation.subpixel_refined) {
      return this->fallback_to_auto_sampling_("manual_lock_invalid_image");
    }

    // Le verrouillage manuel reproduit bien l'image auto. Il devient la
    // nouvelle baseline pour un affinage tres local uniquement.
    this->best_optical_score_ = this->current_optical_score_;
    this->best_exposure_ = this->current_exposure_;
    this->best_gain_ = this->current_gain_;
    this->best_brightness_ = 0;
    this->best_contrast_ = 0;
    this->tuning_round_ = 0;
    this->tuning_step_ = 20;
    return this->start_manual_exposure_round_();
  }

  if (this->phase_ == CalibrationPhase::TUNE_MANUAL_EXPOSURE) {
    return this->advance_manual_pair_(true);
  }

  if (this->phase_ == CalibrationPhase::TUNE_MANUAL_GAIN) {
    return this->advance_manual_pair_(false);
  }

  if (this->phase_ == CalibrationPhase::TUNE_CONTRAST) {
    if (this->current_optical_score_ > this->best_optical_score_) {
      this->best_optical_score_ = this->current_optical_score_;
      this->best_contrast_ = this->current_contrast_;
    }

    if (this->tuning_index_ == 0) {
      this->tuning_index_ = 1;
      if (!this->prepare_postprocess_candidate_(0, -1)) return false;
      return this->request_next_capture_();
    }

    if (this->tuning_index_ == 1 && this->best_contrast_ == 1) {
      this->tuning_index_ = 2;
      if (!this->prepare_postprocess_candidate_(0, 2)) return false;
      return this->request_next_capture_();
    }

    return this->start_brightness_tuning_();
  }

  if (this->phase_ == CalibrationPhase::TUNE_BRIGHTNESS) {
    if (this->current_optical_score_ > this->best_optical_score_) {
      this->best_optical_score_ = this->current_optical_score_;
      this->best_brightness_ = this->current_brightness_;
    }

    if (this->tuning_index_ == 0) {
      this->tuning_index_ = 1;
      if (!this->prepare_postprocess_candidate_(
              1, this->best_contrast_)) {
        return false;
      }
      return this->request_next_capture_();
    }

    return this->finish_optical_tuning_();
  }

  return false;
}

bool FullCalibrationController::start_manual_exposure_round_() {
  this->phase_ = CalibrationPhase::TUNE_MANUAL_EXPOSURE;
  this->tuning_pair_base_ = this->best_exposure_;
  this->tuning_pair_best_value_ = this->best_exposure_;
  this->tuning_pair_best_score_ = this->best_optical_score_;
  this->tuning_side_ = 0;

  const int candidate =
      clamp_int(this->tuning_pair_base_ - this->tuning_step_, 0, 65535);
  if (!this->prepare_manual_candidate_(candidate, this->best_gain_)) return false;
  return this->request_next_capture_();
}

bool FullCalibrationController::start_manual_gain_round_() {
  this->phase_ = CalibrationPhase::TUNE_MANUAL_GAIN;
  this->tuning_pair_base_ = this->best_gain_;
  this->tuning_pair_best_value_ = this->best_gain_;
  this->tuning_pair_best_score_ = this->best_optical_score_;
  this->tuning_side_ = 0;

  const int candidate =
      clamp_int(this->tuning_pair_base_ - this->tuning_step_, 0, 64);
  if (!this->prepare_manual_candidate_(this->best_exposure_, candidate)) return false;
  return this->request_next_capture_();
}

bool FullCalibrationController::advance_manual_pair_(bool exposure_axis) {
  const int current_value =
      exposure_axis ? this->current_exposure_ : this->current_gain_;
  if (this->current_optical_score_ > this->tuning_pair_best_score_) {
    this->tuning_pair_best_score_ = this->current_optical_score_;
    this->tuning_pair_best_value_ = current_value;
  }

  if (this->tuning_side_ == 0) {
    this->tuning_side_ = 1;
    const int maximum = exposure_axis ? 65535 : 64;
    const int candidate =
        clamp_int(this->tuning_pair_base_ + this->tuning_step_, 0, maximum);
    if (exposure_axis) {
      if (!this->prepare_manual_candidate_(candidate, this->best_gain_)) return false;
    } else {
      if (!this->prepare_manual_candidate_(this->best_exposure_, candidate)) return false;
    }
    return this->request_next_capture_();
  }

  if (this->tuning_pair_best_score_ > this->best_optical_score_) {
    this->best_optical_score_ = this->tuning_pair_best_score_;
    if (exposure_axis) {
      this->best_exposure_ = this->tuning_pair_best_value_;
    } else {
      this->best_gain_ = this->tuning_pair_best_value_;
    }
  }

  this->tuning_round_++;
  if (exposure_axis) {
    if (this->tuning_round_ < 2) {
      this->tuning_step_ = 10;
      return this->start_manual_exposure_round_();
    }
    this->tuning_round_ = 0;
    this->tuning_step_ = 1;
    return this->start_manual_gain_round_();
  }

  return this->start_contrast_tuning_();
}

bool FullCalibrationController::start_contrast_tuning_() {
  this->phase_ = CalibrationPhase::TUNE_CONTRAST;
  this->tuning_index_ = 0;
  this->best_brightness_ = 0;
  this->best_contrast_ = 0;

  // Revenir explicitement au meilleur couple exp/gain : la derniere image du
  // balayage gain peut correspondre au candidat oppose, pas au gagnant.
  if (!this->prepare_manual_candidate_(
          this->best_exposure_, this->best_gain_)) {
    return false;
  }

  // Baseline = contraste 0 / luminosite 0 avec exp/gain deja optimises.
  // Tester +1, puis -1. +2 n'est teste que si +1 a effectivement gagne.
  if (!this->prepare_postprocess_candidate_(0, 1)) return false;
  return this->request_next_capture_();
}

bool FullCalibrationController::start_brightness_tuning_() {
  this->phase_ = CalibrationPhase::TUNE_BRIGHTNESS;
  this->tuning_index_ = 0;
  this->best_brightness_ = 0;

  // Le meilleur contraste est fixe ; comparer luminosite -1 puis +1 a la
  // baseline neutre 0 deja evaluee pendant la recherche precedente.
  if (!this->prepare_postprocess_candidate_(-1, this->best_contrast_)) return false;
  return this->request_next_capture_();
}

bool FullCalibrationController::finish_optical_tuning_() {
  if (this->best_optical_score_ <= 0.0f) {
    return this->fallback_to_auto_sampling_("manual_tuning_no_valid_profile");
  }

  if (!this->prepare_manual_candidate_(
          this->best_exposure_, this->best_gain_) ||
      !this->prepare_postprocess_candidate_(
          this->best_brightness_, this->best_contrast_)) {
    return this->fallback_to_auto_sampling_("manual_final_apply_failed");
  }

  this->phase_ = CalibrationPhase::SAMPLING;
  this->current_optical_score_ = this->best_optical_score_;

  ESP_LOGI(TAG,
           "Optique verrouillee MANUEL: AE=%d exposition=%d gain=%d lum=%d ctr=%d "
           "score=%.1f AEC=OFF AGC=OFF; debut des %u mesures calibration",
           this->best_ae_level_, this->best_exposure_, this->best_gain_,
           this->best_brightness_, this->best_contrast_,
           this->best_optical_score_,
           static_cast<unsigned>(this->requested_samples_));

  return this->request_next_capture_();
}

bool FullCalibrationController::fallback_to_auto_sampling_(const char *reason) {
  ESP_LOGW(TAG,
           "Verrouillage manuel refuse (%s): retour AEC/AGC AUTO avant calibration",
           reason != nullptr ? reason : "unknown");

  this->auto_fallback_ = true;
  this->best_optical_score_ = -1.0f;
  this->best_exposure_ = 0;
  this->best_gain_ = 0;
  this->best_brightness_ = 0;
  this->best_contrast_ = 0;
  this->tuning_index_ = 0;

  if (!this->enable_auto_controls_()) return false;
  this->phase_ = CalibrationPhase::TUNE_AUTO_SETTLE;
  return this->request_next_capture_();
}

bool FullCalibrationController::apply_camera_snapshot_(
    const CameraSettingsSnapshot &snapshot) {
  if (!snapshot.available || this->settings_controller_ == nullptr) return false;

  std::string error;
  if (!this->settings_controller_->set_monochrome(snapshot.monochrome, error) ||
      !this->settings_controller_->set_brightness(snapshot.brightness, error) ||
      !this->settings_controller_->set_contrast(snapshot.contrast, error) ||
      !this->settings_controller_->set_ae_level(snapshot.ae_level, error) ||
      !this->settings_controller_->set_exposure_ctrl(false, error) ||
      !this->settings_controller_->set_gain_ctrl(false, error) ||
      !this->settings_controller_->set_aec_value(snapshot.aec_value, error) ||
      !this->settings_controller_->set_agc_gain(snapshot.agc_gain, error) ||
      !this->settings_controller_->set_exposure_ctrl(snapshot.exposure_ctrl, error) ||
      !this->settings_controller_->set_gain_ctrl(snapshot.gain_ctrl, error)) {
    ESP_LOGE(TAG, "Restauration camera impossible: %s", error.c_str());
    return false;
  }
  return true;
}

void FullCalibrationController::restore_previous_camera_settings_() {
  if (!this->previous_camera_settings_saved_) return;
  this->apply_camera_snapshot_(this->previous_camera_settings_);
  this->previous_camera_settings_saved_ = false;
}

bool FullCalibrationController::request_next_capture_() {
  if (this->jpeg_source_->capture_pending()) return false;
  this->capture_count_before_request_ = this->jpeg_source_->capture_count();
  this->capture_started_ms_ = millis();
  if (!this->jpeg_source_->request_capture()) return false;
  this->state_ = FullCalibrationState::WAIT_CAPTURE;
  return true;
}

bool FullCalibrationController::derive_current_sample_(
    const TargetObservation &reference_observation,
    CameraCalibration &sample) {
  return this->measurement_manager_->measurement_engine().derive_calibration_from_known_distance(
      reference_observation,
      CALIBRATION_REFERENCE_WIDTH,
      CALIBRATION_REFERENCE_HEIGHT,
      this->known_distance_mm_,
      sample);
}

void FullCalibrationController::update_running_stats_() {
  if (this->valid_samples_ == 0) {
    this->mean_fx_px_ = 0.0f;
    this->mean_fy_px_ = 0.0f;
    this->stddev_fx_px_ = 0.0f;
    this->stddev_fy_px_ = 0.0f;
    return;
  }

  float fx_values[MAX_SAMPLE_COUNT];
  float fy_values[MAX_SAMPLE_COUNT];
  float fx_sorted[MAX_SAMPLE_COUNT];
  float fy_sorted[MAX_SAMPLE_COUNT];

  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    fx_values[i] = this->samples_[i].fx_px;
    fy_values[i] = this->samples_[i].fy_px;
    fx_sorted[i] = fx_values[i];
    fy_sorted[i] = fy_values[i];
  }

  const float median_fx = median_float(fx_sorted, this->valid_samples_);
  const float median_fy = median_float(fy_sorted, this->valid_samples_);

  float fx_dev[MAX_SAMPLE_COUNT];
  float fy_dev[MAX_SAMPLE_COUNT];
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    fx_dev[i] = std::fabs(fx_values[i] - median_fx);
    fy_dev[i] = std::fabs(fy_values[i] - median_fy);
  }

  const float mad_fx = median_float(fx_dev, this->valid_samples_);
  const float mad_fy = median_float(fy_dev, this->valid_samples_);

  const float gate_fx = std::max(
      std::fabs(median_fx) * 0.0005f,
      3.5f * 1.4826f * mad_fx);
  const float gate_fy = std::max(
      std::fabs(median_fy) * 0.0005f,
      3.5f * 1.4826f * mad_fy);

  bool inlier_mask[MAX_SAMPLE_COUNT] = {};
  double sum_fx = 0.0;
  double sum_fy = 0.0;
  uint8_t inlier_count = 0;

  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    const bool inlier =
        std::fabs(fx_values[i] - median_fx) <= gate_fx &&
        std::fabs(fy_values[i] - median_fy) <= gate_fy;
    inlier_mask[i] = inlier;
    if (inlier) {
      sum_fx += fx_values[i];
      sum_fy += fy_values[i];
      ++inlier_count;
    }
  }

  if (inlier_count < 3) {
    sum_fx = 0.0;
    sum_fy = 0.0;
    inlier_count = this->valid_samples_;
    for (uint8_t i = 0; i < this->valid_samples_; ++i) {
      inlier_mask[i] = true;
      sum_fx += fx_values[i];
      sum_fy += fy_values[i];
    }
  }

  this->mean_fx_px_ = static_cast<float>(sum_fx / inlier_count);
  this->mean_fy_px_ = static_cast<float>(sum_fy / inlier_count);

  double variance_fx = 0.0;
  double variance_fy = 0.0;
  uint8_t variance_count = 0;
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    if (!inlier_mask[i]) continue;
    const double dx = fx_values[i] - this->mean_fx_px_;
    const double dy = fy_values[i] - this->mean_fy_px_;
    variance_fx += dx * dx;
    variance_fy += dy * dy;
    ++variance_count;
  }

  this->stddev_fx_px_ = static_cast<float>(
      std::sqrt(variance_fx / std::max<uint8_t>(1, variance_count)));
  this->stddev_fy_px_ = static_cast<float>(
      std::sqrt(variance_fy / std::max<uint8_t>(1, variance_count)));
}

void FullCalibrationController::finish_success_() {
  if (this->valid_samples_ == 0) {
    this->fail_("no_valid_samples");
    return;
  }

  this->update_running_stats_();
  this->result_calibration_ = this->samples_[0];
  this->result_calibration_.fx_px = this->mean_fx_px_;
  this->result_calibration_.fy_px = this->mean_fy_px_;
  this->result_calibration_.cx_px =
      (static_cast<float>(CALIBRATION_REFERENCE_WIDTH) - 1.0f) * 0.5f;
  this->result_calibration_.cy_px =
      (static_cast<float>(CALIBRATION_REFERENCE_HEIGHT) - 1.0f) * 0.5f;
  this->result_calibration_.reference_width_px = CALIBRATION_REFERENCE_WIDTH;
  this->result_calibration_.reference_height_px = CALIBRATION_REFERENCE_HEIGHT;

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  engine.set_calibration(this->result_calibration_);
  this->measurement_manager_->reset();

  this->previous_config_saved_ = false;
  // Les reglages camera optimises sont volontairement conserves pour les
  // mesures suivantes. Une future calibration recalculera automatiquement un
  // nouveau profil si l'eclairage de la piece change.
  this->previous_camera_settings_saved_ = false;
  this->restore_nominal_camera_();
  this->last_error_.clear();
  this->state_ = FullCalibrationState::COMPLETE;

  ESP_LOGI(TAG,
           "Calibration terminee: n=%u fx=%.3f +/- %.3f fy=%.3f +/- %.3f; "
           "camera verrouillee AE=%d exp=%d gain=%d lum=%d ctr=%d",
           static_cast<unsigned>(this->valid_samples_),
           this->mean_fx_px_, this->stddev_fx_px_,
           this->mean_fy_px_, this->stddev_fy_px_,
           this->best_ae_level_, this->best_exposure_, this->best_gain_,
           this->best_brightness_, this->best_contrast_);
}

void FullCalibrationController::fail_(const char *error) {
  this->last_error_ = error != nullptr ? error : "calibration_failed";
  ESP_LOGE(TAG, "Calibration en echec: %s", this->last_error_.c_str());
  this->restore_previous_measurement_config_();
  this->restore_previous_camera_settings_();
  this->restore_nominal_camera_();
  this->state_ = FullCalibrationState::ERROR;
}

void FullCalibrationController::restore_nominal_camera_() {
  if (this->tracking_controller_ != nullptr) {
    if (this->tracking_controller_->active()) {
      this->tracking_controller_->stop();
    }
  }
  if (this->detection_service_ != nullptr) {
    this->detection_service_->reset_tracking();
  }
}

void FullCalibrationController::restore_previous_measurement_config_() {
  if (!this->previous_config_saved_ || this->measurement_manager_ == nullptr) return;

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  engine.set_calibration(this->previous_calibration_);
  this->measurement_manager_->reset();
  this->previous_config_saved_ = false;
}

void FullCalibrationController::reset_run_() {
  this->last_error_.clear();
  this->phase_ = CalibrationPhase::TRACKING;
  this->valid_samples_ = 0;
  this->attempts_ = 0;
  this->capture_count_before_request_ = 0;
  this->capture_started_ms_ = 0;
  this->result_calibration_ = CameraCalibration();
  this->mean_fx_px_ = 0.0f;
  this->mean_fy_px_ = 0.0f;
  this->stddev_fx_px_ = 0.0f;
  this->stddev_fy_px_ = 0.0f;
  this->preview_attempt_ = 0;
  this->preview_mode_ = "search";
  this->last_target_found_ = false;
  this->last_marker_count_ = 0;
  this->last_marker_mask_ = 0;
  this->last_board_complete_ = false;
  this->last_sample_valid_ = false;
  this->last_sample_fx_px_ = 0.0f;
  this->last_sample_fy_px_ = 0.0f;

  this->tuning_attempts_ = 0;
  this->tuning_round_ = 0;
  this->tuning_side_ = 0;
  this->tuning_index_ = 0;
  this->tuning_pair_base_ = 0;
  this->tuning_step_ = 0;
  this->tuning_pair_best_value_ = 0;
  this->tuning_pair_best_score_ = -1.0f;
  this->current_ae_level_ = 0;
  this->current_exposure_ = 0;
  this->current_gain_ = 0;
  this->current_brightness_ = 0;
  this->current_contrast_ = 0;
  this->current_optical_score_ = 0.0f;
  this->current_detection_quality_ = 0.0f;
  this->current_subpixel_rms_px_ = 0.0f;
  this->current_width_gradient_ = 0.0f;
  this->current_height_gradient_ = 0.0f;
  this->current_mean_luma_x100_ = 0;
  this->current_dark_percent_x100_ = 0;
  this->current_bright_percent_x100_ = 0;
  this->current_p10_luma_ = 0;
  this->current_p90_luma_ = 0;
  this->current_contrast_luma_ = 0;
  this->best_ae_level_ = 0;
  this->best_exposure_ = 0;
  this->best_gain_ = 0;
  this->best_brightness_ = 0;
  this->best_contrast_ = 0;
  this->best_optical_score_ = -1.0f;
  this->auto_fallback_ = false;
  this->tuning_roi_x_ = 0;
  this->tuning_roi_y_ = 0;
  this->tuning_roi_width_ = 0;
  this->tuning_roi_height_ = 0;

  for (auto &sample : this->samples_) {
    sample = CameraCalibration();
  }
  this->state_ = FullCalibrationState::IDLE;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
