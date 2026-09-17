#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

class CameraResolutionController;

enum class CameraViewportMode : uint8_t {
  SEARCH_FULL,
  ZOOM_WIDE,
  ZOOM_MEDIUM,
  ZOOM_FINE,
  PRECISE_ROI,
};

struct CameraViewportSnapshot {
  CameraViewportSnapshot();

  bool supported;
  CameraViewportMode mode;
  uint16_t reference_width;
  uint16_t reference_height;
  // Fenetre dans le repere canonique tel qu'il est affiche/detecte.
  uint16_t window_x;
  uint16_t window_y;
  uint16_t window_width;
  uint16_t window_height;

  // Fenetre réellement programmee dans les registres bruts OV5640.
  // Elle peut etre symetrique de window_x/window_y quand ESPHome active
  // horizontal_mirror et/ou vertical_flip.
  uint16_t sensor_window_x;
  uint16_t sensor_window_y;
  bool horizontal_mirror;
  bool vertical_flip;
  uint16_t output_width;
  uint16_t output_height;
  float scale_x;
  float scale_y;
};

class CameraViewportController {
 public:
  explicit CameraViewportController(CameraResolutionController *resolution_controller);

  bool supports_precise_roi() const;
  bool apply_search();
  bool apply_zoom_wide(float center_reference_x, float center_reference_y);
  bool apply_zoom_medium(float center_reference_x, float center_reference_y);
  bool apply_zoom_fine(float center_reference_x, float center_reference_y);
  bool apply_precise_roi(float center_reference_x, float center_reference_y);
  bool recenter_current_zoom(float center_reference_x, float center_reference_y);
  bool recenter_current_zoom_from_local(float local_center_x, float local_center_y,
                                        bool &viewport_moved);

  TargetObservation to_reference(const TargetObservation &observation) const;
  bool target_near_edge(const TargetObservation &observation, uint8_t central_percent) const;

  const CameraViewportSnapshot &snapshot() const;
  static const char *mode_text(CameraViewportMode mode);

  static constexpr uint16_t REFERENCE_WIDTH = 2560;
  static constexpr uint16_t REFERENCE_HEIGHT = 1920;
  static constexpr uint16_t OUTPUT_WIDTH = 800;
  static constexpr uint16_t OUTPUT_HEIGHT = 600;
  static constexpr uint16_t ZOOM_WIDE_WIDTH = 1920;
  static constexpr uint16_t ZOOM_WIDE_HEIGHT = 1440;
  static constexpr uint16_t ZOOM_MEDIUM_WIDTH = 1280;
  static constexpr uint16_t ZOOM_MEDIUM_HEIGHT = 960;
  static constexpr uint16_t ZOOM_FINE_WIDTH = 1024;
  static constexpr uint16_t ZOOM_FINE_HEIGHT = 768;

 private:
  bool apply_zoom_window_(CameraViewportMode mode,
                          float center_reference_x, float center_reference_y,
                          uint16_t window_width, uint16_t window_height);
  ImagePoint to_reference_point_(const ImagePoint &point) const;
  void set_search_snapshot_();

  CameraResolutionController *resolution_controller_;
  CameraViewportSnapshot snapshot_;
};

}  // namespace geometrie_camera_app
}  // namespace esphome
