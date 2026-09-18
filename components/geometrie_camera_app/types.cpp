#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

ImagePoint::ImagePoint() : x(0.0f), y(0.0f) {}

CameraCalibration::CameraCalibration()
    : fx_px(0.0f),
      fy_px(0.0f),
      cx_px(0.0f),
      cy_px(0.0f),
      k1(0.0f),
      k2(0.0f),
      p1(0.0f),
      p2(0.0f),
      k3(0.0f),
      reference_width_px(0),
      reference_height_px(0) {}

TargetObservation::TargetObservation()
    : valid(false),
      center_x_px(0.0f),
      center_y_px(0.0f),
      width_px(0.0f),
      height_px(0.0f),
      rotation_deg(0.0f),
      quality(0.0f),
      subpixel_refined(false),
      subpixel_rms_px(0.0f),
      subpixel_max_rms_px(0.0f),
      subpixel_gradient(0.0f),
      subpixel_width_px(0.0f),
      subpixel_height_px(0.0f),
      top_left_px(),
      top_right_px(),
      bottom_right_px(),
      bottom_left_px() {}

GeometryMeasurement::GeometryMeasurement()
    : valid(false),
      calibrated(false),
      pose_valid(false),
      edge_v4_used(false),
      apparent_width_px(0.0f),
      apparent_height_px(0.0f),
      distance_mm(0.0f),
      x_mm(0.0f),
      y_mm(0.0f),
      z_mm(0.0f),
      z_from_width_mm(0.0f),
      z_from_height_mm(0.0f),
      bearing_yaw_deg(0.0f),
      bearing_pitch_deg(0.0f),
      yaw_deg(0.0f),
      pitch_deg(0.0f),
      roll_deg(0.0f),
      pose_z_mm(0.0f),
      pose_scale_error_pct(0.0f),
      quality(0.0f),
      timestamp_ms(0) {}

GrayFrameView::GrayFrameView()
    : data(nullptr), width(0), height(0), stride(0) {}

}  // namespace geometrie_camera_app
}  // namespace esphome
