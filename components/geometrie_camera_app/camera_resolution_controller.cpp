#include "camera_resolution_controller.h"

#include "esp_camera.h"
#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace geometrie_camera_app {

namespace {
static const char *const TAG = "camera_resolution";

struct ResolutionEntry {
  const char *name;
  framesize_t frame_size;
  uint16_t width;
  uint16_t height;
};

constexpr ResolutionEntry RESOLUTIONS[] = {
    {"320x240", FRAMESIZE_QVGA, 320, 240},
    {"640x480", FRAMESIZE_VGA, 640, 480},
    {"800x600", FRAMESIZE_SVGA, 800, 600},
    {"1024x768", FRAMESIZE_XGA, 1024, 768},
    {"1280x720", FRAMESIZE_HD, 1280, 720},
    {"1280x1024", FRAMESIZE_SXGA, 1280, 1024},
    {"1600x1200", FRAMESIZE_UXGA, 1600, 1200},
    {"1920x1080", FRAMESIZE_FHD, 1920, 1080},
    {"2048x1536", FRAMESIZE_QXGA, 2048, 1536},
};

const ResolutionEntry *find_by_name(const std::string &name) {
  for (const auto &entry : RESOLUTIONS) {
    if (name == entry.name) {
      return &entry;
    }
  }
  return nullptr;
}

const ResolutionEntry *find_by_framesize(framesize_t frame_size) {
  for (const auto &entry : RESOLUTIONS) {
    if (frame_size == entry.frame_size) {
      return &entry;
    }
  }
  return nullptr;
}
}  // namespace

CameraResolutionController::CameraResolutionController()
    : active_resolution_("2048x1536"),
      active_width_(2048),
      active_height_(1536),
      sensor_pid_(0),
      sensor_name_("unknown"),
      sensor_max_resolution_("unknown") {}

bool CameraResolutionController::sync_from_sensor() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return false;
  }

  this->update_sensor_identity_();

  const auto *entry = find_by_framesize(sensor->status.framesize);
  if (entry == nullptr) {
    ESP_LOGW(TAG, "Resolution capteur non mappee: framesize=%d", static_cast<int>(sensor->status.framesize));
    return false;
  }

  this->active_resolution_ = entry->name;
  this->active_width_ = entry->width;
  this->active_height_ = entry->height;
  ESP_LOGI(TAG, "Resolution active detectee: %s", this->active_resolution_.c_str());
  return true;
}

bool CameraResolutionController::refresh_sensor_identity() {
  this->update_sensor_identity_();
  return this->sensor_pid_ != 0;
}

bool CameraResolutionController::apply(const std::string &resolution) {
  const auto *entry = find_by_name(resolution);
  if (entry == nullptr) {
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr || sensor->set_framesize == nullptr) {
    ESP_LOGE(TAG, "Capteur ou fonction set_framesize indisponible");
    return false;
  }

  this->update_sensor_identity_();

  if (sensor->set_framesize(sensor, entry->frame_size) != 0) {
    ESP_LOGE(TAG, "Echec application resolution %s", resolution.c_str());
    return false;
  }

  this->active_resolution_ = entry->name;
  this->active_width_ = entry->width;
  this->active_height_ = entry->height;
  ESP_LOGI(TAG, "Resolution appliquee: %s", this->active_resolution_.c_str());
  return true;
}

bool CameraResolutionController::is_supported(const std::string &resolution) const {
  return find_by_name(resolution) != nullptr;
}

const std::string &CameraResolutionController::active_resolution() const {
  return this->active_resolution_;
}

uint16_t CameraResolutionController::active_width() const {
  return this->active_width_;
}

uint16_t CameraResolutionController::active_height() const {
  return this->active_height_;
}

uint16_t CameraResolutionController::sensor_pid() const {
  return this->sensor_pid_;
}

const std::string &CameraResolutionController::sensor_name() const {
  return this->sensor_name_;
}

const std::string &CameraResolutionController::sensor_max_resolution() const {
  return this->sensor_max_resolution_;
}

void CameraResolutionController::update_sensor_identity_() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    this->sensor_pid_ = 0;
    this->sensor_name_ = "unknown";
    this->sensor_max_resolution_ = "unknown";
    return;
  }

  this->sensor_pid_ = sensor->id.PID;

  if (this->sensor_pid_ == OV5640_PID) {
    this->sensor_name_ = "OV5640";
    this->sensor_max_resolution_ = "2592x1944";
  } else if (this->sensor_pid_ == OV3660_PID) {
    this->sensor_name_ = "OV3660";
    this->sensor_max_resolution_ = "2048x1536";
  } else {
    this->sensor_name_ = "unknown";
    this->sensor_max_resolution_ = "unknown";
  }

  ESP_LOGI(TAG, "Capteur detecte: %s, PID=0x%04X, resolution max=%s",
           this->sensor_name_.c_str(), static_cast<unsigned>(this->sensor_pid_),
           this->sensor_max_resolution_.c_str());
}

const char *CameraResolutionController::allowed_resolutions_text() {
  return "320x240,640x480,800x600,1024x768,1280x720,1280x1024,1600x1200,1920x1080,2048x1536";
}

}  // namespace geometrie_camera_app
}  // namespace esphome
