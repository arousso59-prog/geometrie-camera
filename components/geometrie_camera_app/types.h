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

struct ImageLine {
  ImageLine();

  bool valid;
  ImagePoint point;
  float dx;
  float dy;
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

  // V6.1 officielle : meme dimension directe que V5, mais incertitude
  // augmentee si la mesure locale V6 diverge.
  float subpixel_width_sigma_px;
  float subpixel_height_sigma_px;
  float subpixel_width_gradient;
  float subpixel_height_gradient;

  // Diagnostic comparatif conserve en parallele, sans modifier le resultat
  // officiel : V5 = separation directe des droites robustes, V6 = separation
  // locale robuste des echantillons de bords.
  float subpixel_v5_width_px;
  float subpixel_v5_height_px;
  float subpixel_v5_width_sigma_px;
  float subpixel_v5_height_sigma_px;
  float subpixel_v6_width_px;
  float subpixel_v6_height_px;
  float subpixel_v6_width_sigma_px;
  float subpixel_v6_height_sigma_px;

  // Qualite individuelle des quatre bords, utile pour diagnostiquer une
  // asymetrie horizontale/verticale de l'image.
  float subpixel_top_rms_px;
  float subpixel_right_rms_px;
  float subpixel_bottom_rms_px;
  float subpixel_left_rms_px;
  float subpixel_top_gradient;
  float subpixel_right_gradient;
  float subpixel_bottom_gradient;
  float subpixel_left_gradient;

  // Droites subpixel directement ajustees sur les profils des quatre bords.
  // Elles sont plus stables pour la pose que les intersections de coins.
  ImageLine subpixel_top_line;
  ImageLine subpixel_right_line;
  ImageLine subpixel_bottom_line;
  ImageLine subpixel_left_line;

  // Pose V3 : homographie robuste issue des transitions internes du motif 7x7.
  // Elle reste totalement separee de la chaine de distance.
  bool pattern_refined;
  uint16_t pattern_feature_count;
  uint16_t pattern_inlier_count;
  float pattern_rms_px;
  float pattern_max_residual_px;
  float pattern_homography[9];

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
  bool pose_v1_valid;
  bool pose_v2_valid;
  bool pose_v2_used;
  bool pose_v3_valid;
  bool pose_v3_used;
  bool edge_v4_used;
  bool edge_v5_used;
  bool edge_v6_used;
  float apparent_width_px;
  float apparent_height_px;
  float apparent_width_sigma_px;
  float apparent_height_sigma_px;

  // Diagnostic de precision : les trois methodes sont calculees en parallele
  // mais n'influencent pas la mesure officielle V6.1.
  float corner_width_px;
  float corner_height_px;
  float v5_width_px;
  float v5_height_px;
  float v6_width_px;
  float v6_height_px;
  float v61_width_px;
  float v61_height_px;
  float v5_v6_width_delta_px;
  float v5_v6_height_delta_px;
  float v5_z_mm;
  float v6_z_mm;
  float v61_z_mm;
  float edge_top_rms_px;
  float edge_right_rms_px;
  float edge_bottom_rms_px;
  float edge_left_rms_px;
  float edge_top_gradient;
  float edge_right_gradient;
  float edge_bottom_gradient;
  float edge_left_gradient;

  float width_distance_weight;
  float height_distance_weight;
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

  // Pose V1 = decomposition homographie historique. Pose V2 = optimisation
  // robuste des quatre droites subpixel avec translation V6.1 figee.
  float pose_v1_yaw_deg;
  float pose_v1_pitch_deg;
  float pose_v1_roll_deg;
  float pose_v2_yaw_deg;
  float pose_v2_pitch_deg;
  float pose_v2_roll_deg;
  float pose_v2_line_rms_px;
  float pose_v2_corner_rms_px;
  float pose_v3_yaw_deg;
  float pose_v3_pitch_deg;
  float pose_v3_roll_deg;
  float pose_v3_pattern_rms_px;
  float pose_v3_fit_rms_px;
  uint16_t pose_v3_feature_count;
  uint16_t pose_v3_inlier_count;
  float pose_normal_x;
  float pose_normal_y;
  float pose_normal_z;

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
