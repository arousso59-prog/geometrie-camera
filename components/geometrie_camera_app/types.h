#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace geometrie_camera_app {

struct ImagePoint {
  ImagePoint();

  float x;
  float y;
};

struct CameraCalibration {
  CameraCalibration();

  float fx_px;
  float fy_px;
  float cx_px;
  float cy_px;

  // Modele Brown-Conrady dans les coordonnees normalisees camera.
  // Valeurs nulles = aucune correction de distorsion.
  float k1;
  float k2;
  float p1;
  float p2;
  float k3;

  uint16_t reference_width_px;
  uint16_t reference_height_px;
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

  // Diagnostic du dernier raffinement geometrique haute precision.
  // subpixel_refined=false signifie que le chemin historique au pixel a ete
  // conserve pour cette detection.
  bool subpixel_refined;
  float subpixel_rms_px;
  float subpixel_max_rms_px;
  float subpixel_gradient;

  // Mesure V4 : dimensions apparentes issues directement des paires de
  // droites subpixel opposees. Ces valeurs sont moins sensibles aux petites
  // variations des intersections de coins.
  float subpixel_width_px;
  float subpixel_height_px;

  ImagePoint top_left_px;
  ImagePoint top_right_px;
  ImagePoint bottom_right_px;
  ImagePoint bottom_left_px;
};

struct GeometryMeasurement {
  GeometryMeasurement();

  bool valid;
  bool calibrated;
  bool pose_valid;
  float distance_mm;
  float x_mm;
  float y_mm;
  float z_mm;
  float z_from_width_mm;
  float z_from_height_mm;
  float bearing_yaw_deg;
  float bearing_pitch_deg;
  float yaw_deg;
  float pitch_deg;
  float roll_deg;
  float pose_z_mm;
  float pose_scale_error_pct;
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

}  // namespace geometrie_camera_app
}  // namespace esphome
