#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct CameraFrameInfo {
  CameraFrameInfo();

  bool valid;
  uint16_t width;
  uint16_t height;
  size_t size_bytes;
  uint32_t timestamp_ms;
};

struct CameraCalibration {
  CameraCalibration();

  float fx_px;
  float fy_px;
  float cx_px;
  float cy_px;
};

struct TargetObservation {
  TargetObservation();

  bool valid;
  float center_x_px;
  float center_y_px;
  float width_px;
  float height_px;
  float rotation_deg;
  float quality;
};

struct GeometryMeasurement {
  GeometryMeasurement();

  bool valid;
  float yaw_deg;
  float pitch_deg;
  float roll_deg;
  float quality;
  uint32_t timestamp_ms;
};

struct GrayFrameView {
  GrayFrameView();

  const uint8_t *data;
  uint16_t width;
  uint16_t height;
  size_t stride;
};

struct ImageBufferView {
  ImageBufferView();

  const uint8_t *data;
  size_t size_bytes;
  uint16_t width;
  uint16_t height;
  const char *mime_type;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
