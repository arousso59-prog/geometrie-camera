#include "full_calibration_controller.h"

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
}

FullCalibrationController::FullCalibrationController(
    JpegDiagnostic *jpeg_source,
    JpegFilteredDiagnostic *filtered_source,
    TargetDetectionService *detection_service,
    MeasurementManager *measurement_manager,
    ContinuousMeasurementController *continuous_controller,
    TargetTrackingController *tracking_controller,
    TargetDetectionPreview *preview)
    : jpeg_source_(jpeg_source),
      filtered_source_(filtered_source),
      detection_service_(detection_service),
      measurement_manager_(measurement_manager),
      continuous_controller_(continuous_controller),
      tracking_controller_(tracking_controller),
      preview_(preview),
      state_(FullCalibrationState::IDLE),
      last_error_(),
      known_distance_mm_(0.0f),
      target_size_mm_(0.0f),
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
      last_sample_valid_(false),
      last_sample_fx_px_(0.0f),
      last_sample_fy_px_(0.0f),
      previous_target_size_mm_(0.0f),
      previous_calibration_(),
      previous_config_saved_(false),
      previous_tracking_enabled_(true),
      tracking_setting_saved_(false) {}

bool FullCalibrationController::start(float known_distance_mm,
                                      float target_size_mm,
                                      uint8_t sample_count,
                                      bool force) {
  if (this->running()) {
    this->last_error_ = "calibration_already_running";
    return false;
  }
  if (this->jpeg_source_ == nullptr ||
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
  if (!std::isfinite(target_size_mm) || target_size_mm < 1.0f ||
      target_size_mm > 1000.0f) {
    this->last_error_ = "target_size_mm_out_of_range";
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

  this->previous_target_size_mm_ = engine.target_size_mm();
  this->previous_calibration_ = engine.calibration();
  this->previous_config_saved_ = true;

  this->previous_tracking_enabled_ = this->tracking_controller_->enabled();
  this->tracking_setting_saved_ = true;

  this->reset_run_();
  this->known_distance_mm_ = known_distance_mm;
  this->target_size_mm_ = target_size_mm;
  this->requested_samples_ = sample_count;
  this->max_attempts_ = std::min<uint8_t>(
      MAX_TOTAL_ATTEMPTS,
      static_cast<uint8_t>(std::max<int>(sample_count * 5, sample_count + 20)));

  if (!engine.set_target_size_mm(target_size_mm)) {
    this->fail_("target_size_apply_failed");
    return false;
  }

  ESP_LOGI(TAG,
           "Calibration native demandee: reference=%ux%u, sortie native PRECISE=%ux%u, "
           "distance=%.2f mm cible=%.2f mm echantillons=%u",
           static_cast<unsigned>(CALIBRATION_REFERENCE_WIDTH),
           static_cast<unsigned>(CALIBRATION_REFERENCE_HEIGHT),
           static_cast<unsigned>(NATIVE_OUTPUT_WIDTH),
           static_cast<unsigned>(NATIVE_OUTPUT_HEIGHT),
           known_distance_mm, target_size_mm,
           static_cast<unsigned>(sample_count));

  // Si une acquisition etait encore en vol au moment ou le continu a ete
  // arrete, la laisser se terminer avant de reprendre la camera.
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

      // Tous les niveaux SEARCH/ZOOM/PRECISE produisent une image 800x600.
      if (this->jpeg_source_->width() != NATIVE_OUTPUT_WIDTH ||
          this->jpeg_source_->height() != NATIVE_OUTPUT_HEIGHT) {
        this->fail_("unexpected_tracking_output_resolution");
        return;
      }

      this->state_ = FullCalibrationState::FILTER;
      return;
    }

    case FullCalibrationState::FILTER:
      // Camera nominalement N/B : decodage JPEG -> gris uniquement.
      if (!this->filtered_source_->process(false)) {
        this->fail_("calibration_filter_failed");
        return;
      }
      this->state_ = FullCalibrationState::DETECT;
      return;

    case FullCalibrationState::DETECT: {
      this->attempts_++;

      if (!this->detection_service_->detect()) {
        this->fail_("calibration_detection_failed");
        return;
      }

      const bool target_found = this->detection_service_->target_found();
      const TargetObservation local_observation =
          this->detection_service_->last_observation();

      this->last_target_found_ = target_found;
      this->last_sample_valid_ = false;
      this->last_sample_fx_px_ = 0.0f;
      this->last_sample_fy_px_ = 0.0f;

      // Figer l'image correspondant exactement a cette tentative avant de
      // demander la capture suivante. La console peut ainsi suivre chaque
      // etape SEARCH/ZOOM/PRECISE sans decalage d'un cycle.
      if (this->preview_ != nullptr &&
          this->preview_->render(this->filtered_source_, local_observation)) {
        this->preview_attempt_ = this->attempts_;
      }

      const CameraViewportMode mode_before =
          this->tracking_controller_->viewport_controller()->snapshot().mode;
      if (this->preview_attempt_ == this->attempts_) {
        this->preview_mode_ = CameraViewportController::mode_text(mode_before);
      }

      const TrackingUpdateResult tracking_result =
          this->tracking_controller_->update_after_detection(
              target_found, local_observation);

      if (tracking_result == TrackingUpdateResult::ERROR) {
        this->fail_("calibration_tracking_failed");
        return;
      }

      // Si le tracking vient de changer de viewport, l'observation appartient
      // a l'ancien viewport. Ne jamais l'utiliser pour calibrer : reprendre une
      // image fraiche dans le nouveau cadrage.
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

      // On n'accumule les echantillons qu'en PRECISE natif. Dans ce mode
      // window=800x600 et scale=1 : chaque pixel est un pixel physique du
      // capteur. La conversion to_reference() ne fait alors qu'ajouter
      // l'origine du crop dans le repere canonique 2560x1920.
      if (target_found &&
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
                   "Calibration native PRECISE: echantillon %u/%u fx=%.3f fy=%.3f "
                   "cible=%.1fx%.1f px qualite=%.3f",
                   static_cast<unsigned>(this->valid_samples_),
                   static_cast<unsigned>(this->requested_samples_),
                   sample.fx_px, sample.fy_px,
                   reference_observation.width_px,
                   reference_observation.height_px,
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
  if (!this->running()) {
    return;
  }
  this->restore_previous_measurement_config_();
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

const std::string &FullCalibrationController::last_error() const { return this->last_error_; }
uint8_t FullCalibrationController::requested_samples() const { return this->requested_samples_; }
uint8_t FullCalibrationController::valid_samples() const { return this->valid_samples_; }
uint8_t FullCalibrationController::attempts() const { return this->attempts_; }
uint8_t FullCalibrationController::max_attempts() const { return this->max_attempts_; }
float FullCalibrationController::known_distance_mm() const { return this->known_distance_mm_; }
float FullCalibrationController::target_size_mm() const { return this->target_size_mm_; }
float FullCalibrationController::mean_fx_px() const { return this->mean_fx_px_; }
float FullCalibrationController::mean_fy_px() const { return this->mean_fy_px_; }
float FullCalibrationController::stddev_fx_px() const { return this->stddev_fx_px_; }
float FullCalibrationController::stddev_fy_px() const { return this->stddev_fy_px_; }
uint8_t FullCalibrationController::preview_attempt() const { return this->preview_attempt_; }
const std::string &FullCalibrationController::preview_mode() const { return this->preview_mode_; }
bool FullCalibrationController::last_target_found() const { return this->last_target_found_; }
bool FullCalibrationController::last_sample_valid() const { return this->last_sample_valid_; }
float FullCalibrationController::last_sample_fx_px() const { return this->last_sample_fx_px_; }
float FullCalibrationController::last_sample_fy_px() const { return this->last_sample_fy_px_; }
const char *FullCalibrationController::tracking_mode_text() const {
  return this->tracking_controller_ != nullptr
             ? this->tracking_controller_->mode_text()
             : "unavailable";
}
const CameraCalibration &FullCalibrationController::result_calibration() const {
  return this->result_calibration_;
}

bool FullCalibrationController::begin_native_tracking_() {
  if (this->tracking_controller_->active()) {
    this->tracking_controller_->stop();
  }

  if (!this->tracking_controller_->enabled() &&
      !this->tracking_controller_->set_enabled(true)) {
    return false;
  }

  if (!this->tracking_controller_->start()) {
    return false;
  }

  this->detection_service_->reset_tracking();
  ESP_LOGI(TAG, "Calibration native: SEARCH 800x600 actif, progression vers PRECISE");

  return this->request_next_capture_();
}

bool FullCalibrationController::request_next_capture_() {
  if (this->jpeg_source_->capture_pending()) {
    return false;
  }

  this->capture_count_before_request_ = this->jpeg_source_->capture_count();
  this->capture_started_ms_ = millis();
  if (!this->jpeg_source_->request_capture()) {
    return false;
  }

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

  double sum_fx = 0.0;
  double sum_fy = 0.0;
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    sum_fx += this->samples_[i].fx_px;
    sum_fy += this->samples_[i].fy_px;
  }
  this->mean_fx_px_ = static_cast<float>(sum_fx / this->valid_samples_);
  this->mean_fy_px_ = static_cast<float>(sum_fy / this->valid_samples_);

  double variance_fx = 0.0;
  double variance_fy = 0.0;
  for (uint8_t i = 0; i < this->valid_samples_; ++i) {
    const double dx = this->samples_[i].fx_px - this->mean_fx_px_;
    const double dy = this->samples_[i].fy_px - this->mean_fy_px_;
    variance_fx += dx * dx;
    variance_fy += dy * dy;
  }
  variance_fx /= this->valid_samples_;
  variance_fy /= this->valid_samples_;
  this->stddev_fx_px_ = static_cast<float>(std::sqrt(variance_fx));
  this->stddev_fy_px_ = static_cast<float>(std::sqrt(variance_fy));
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
  this->restore_nominal_camera_();
  this->last_error_.clear();
  this->state_ = FullCalibrationState::COMPLETE;

  ESP_LOGI(TAG,
           "Calibration native terminee: n=%u fx=%.3f +/- %.3f fy=%.3f +/- %.3f",
           static_cast<unsigned>(this->valid_samples_),
           this->mean_fx_px_, this->stddev_fx_px_,
           this->mean_fy_px_, this->stddev_fy_px_);
}

void FullCalibrationController::fail_(const char *error) {
  this->last_error_ = error != nullptr ? error : "calibration_failed";
  ESP_LOGE(TAG, "Calibration native en echec: %s", this->last_error_.c_str());
  this->restore_previous_measurement_config_();
  this->restore_nominal_camera_();
  this->state_ = FullCalibrationState::ERROR;
}

void FullCalibrationController::restore_nominal_camera_() {
  if (this->tracking_controller_ != nullptr) {
    if (this->tracking_controller_->active()) {
      this->tracking_controller_->stop();
    }

    if (this->tracking_setting_saved_) {
      this->tracking_controller_->set_enabled(this->previous_tracking_enabled_);
      this->tracking_setting_saved_ = false;
    }
  }

  if (this->detection_service_ != nullptr) {
    this->detection_service_->reset_tracking();
  }
}

void FullCalibrationController::restore_previous_measurement_config_() {
  if (!this->previous_config_saved_ || this->measurement_manager_ == nullptr) {
    return;
  }

  GeometryMeasurementEngine &engine = this->measurement_manager_->measurement_engine();
  engine.set_target_size_mm(this->previous_target_size_mm_);
  engine.set_calibration(this->previous_calibration_);
  this->measurement_manager_->reset();
  this->previous_config_saved_ = false;
}

void FullCalibrationController::reset_run_() {
  this->last_error_.clear();
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
  this->last_sample_valid_ = false;
  this->last_sample_fx_px_ = 0.0f;
  this->last_sample_fy_px_ = 0.0f;
  for (auto &sample : this->samples_) {
    sample = CameraCalibration();
  }
  this->state_ = FullCalibrationState::IDLE;
}

}  // namespace geometrie_camera_app
}  // namespace esphome
