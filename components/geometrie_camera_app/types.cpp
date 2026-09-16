#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

CameraCalibration::CameraCalibration()
    : fx_px(1000.0f), fy_px(1000.0f), cx_px(1024.0f), cy_px(768.0f) {}

TargetObservation::TargetObservation()
    : valid(false), center_x_px(0.0f), center_y_px(0.0f), width_px(0.0f), height_px(0.0f), rotation_deg(0.0f), quality(0.0f) {}

GeometryMeasurement::GeometryMeasurement()
    : valid(false), yaw_deg(0.0f), pitch_deg(0.0f), roll_deg(0.0f), quality(0.0f), timestamp_ms(0) {}

GrayFrameView::GrayFrameView()
    : data(nullptr), width(0), height(0), stride(0) {}

}  // namespace geometrie_camera_app
}  // namespace esphome
