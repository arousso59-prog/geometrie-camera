#pragma once

#include <cstdint>

#include "types.h"

namespace esphome {
namespace geometrie_camera_app {

// Cible physique R1 definitive.
// Coordonnees internes exprimees dans le repere du cadre noir de reference.
constexpr float TARGET_R1_BOARD_WIDTH_MM = 250.0f;
constexpr float TARGET_R1_BOARD_HEIGHT_MM = 100.0f;
constexpr float TARGET_R1_REFERENCE_X_MM = 5.0f;
constexpr float TARGET_R1_REFERENCE_Y_MM = 5.0f;
constexpr float TARGET_R1_REFERENCE_WIDTH_MM = 240.0f;
constexpr float TARGET_R1_REFERENCE_HEIGHT_MM = 90.0f;

struct TargetMarkerSpec {
  TargetMarkerId id;
  float x_mm;
  float y_top_mm;
  float size_mm;
};

const TargetMarkerSpec &target_r1_marker_spec(TargetMarkerId id);
uint8_t target_r1_marker_cell(TargetMarkerId id, uint8_t row, uint8_t column);
int target_r1_marker_index(TargetMarkerId id);

// Construit l'observation globale du cadre R1 a partir d'au moins deux
// marqueurs identifies. Avec les trois marqueurs et leurs points subpixel,
// l'observation est eligible a la mesure haute precision.
bool build_target_r1_observation(
    const TargetObservation (&markers)[3],
    PatternFeature *composite_features,
    uint16_t composite_capacity,
    TargetObservation &result);

}  // namespace geometrie_camera_app
}  // namespace esphome
