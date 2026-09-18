#include "api_wsdl.h"

#include <string>

#include "camera_resolution_controller.h"

namespace esphome {
namespace geometrie_camera_app {

ApiWsdlHandler::ApiWsdlHandler(CameraResolutionController *resolution_controller)
    : resolution_controller_(resolution_controller) {}

bool ApiWsdlHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(url_buf) == "/api/wsdl";
}

void ApiWsdlHandler::handleRequest(AsyncWebServerRequest *request) {
  const char *allowed_resolutions = this->resolution_controller_ != nullptr
                                        ? CameraResolutionController::allowed_resolutions_text()
                                        : "unknown";

  std::string xml;
  xml.reserve(30000);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"36\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API camera OV5640 : capture JPEG, reglages capteur, viewport ROI haute resolution, tracking cible, detection, calibration, mesure geometrique et acquisition continue.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande utilisent encore HTTP GET pendant la phase de mise au point.</item>\n";
  xml += "    <item>monochrome est un effet grayscale runtime du capteur OV5640 et ne change pas le pixel_format JPEG.</item>\n";
  xml += "    <item>Le repere camera est X vers la droite, Y vers le bas et Z vers l avant.</item>\n";
  xml += "    <item>GeometryMeasurementEngine V3 fusionne z_from_width_mm et z_from_height_mm de facon continue ; pose_z_mm reste un controle independant issu de l homographie.</item>\n";
  xml += "    <item>Le tracking automatique est configure par /tracking/config. /continuous/start ne pilote pas la ROI depuis le PC : l ESP32 gere SEARCH, ZOOM_WIDE, ZOOM_MEDIUM, ZOOM_FINE et PRECISE de facon autonome.</item>\n";
  xml += "    <item>La configuration du tracking est verrouillee pendant une session active ; /tracking/config/set renvoie HTTP 409 dans ce cas.</item>\n";
  xml += "    <item>Le zoom tracking est progressif : SEARCH couvre 2560x1920, ZOOM_WIDE 1920x1440, ZOOM_MEDIUM 1280x960, ZOOM_FINE 1024x768 et PRECISE 800x600. Chaque niveau produit une image 800x600 et utilise une boucle fermee sur la position locale de la cible pour recentrer le viewport avant de passer au niveau suivant.</item>\n    <item>Le repere canonique des mesures suit l image affichee. ESPHome applique vertical_flip=true et horizontal_mirror=true par defaut ; avant set_res_raw, le viewport canonique est donc converti vers les coordonnees physiques OV5640. L API expose window (canonique) et sensor_window (brut).</item>\n";
  xml += "    <item>Les coordonnees ROI sont converties en repere de reference 2560x1920 avant le calcul de mesure ; la calibration existante est redimensionnee par GeometryMeasurementEngine selon sa resolution de reference.</item>\n";
  xml += "    <item>Apres lost_cycles pertes consecutives dans un niveau zoome, le capteur revient automatiquement en SEARCH. En PRECISE, un centrage fin rapproche aussi la cible du centre 400x300.</item>\n";
  xml += "    <item>La calibration automatique utilise le tracking jusqu au mode PRECISE 800x600 natif sans scaling. Les quatre coins sont ensuite convertis dans le repere canonique 2560x1920 ; on obtient ainsi une calibration plein capteur sans decoder une image 5 MP complete.</item>\n";
  xml += "    <item>Le moteur de geometrie supporte le modele de distorsion Brown-Conrady k1,k2,p1,p2,k3. Les coefficients nuls conservent exactement le comportement sans correction.</item>\n";
  xml += "    <item>Apres la detection et le raffinement pixel historiques, la geometrie de cible peut etre affinee au subpixel par ajustement des quatre bords externes. /target/status expose subpixel_refined et les RMS de l ajustement ; en cas de rejet, le chemin pixel historique est conserve.</item>\n";
  xml += "    <item>Distance V4 : 31 echantillons par bord, moyenne locale du gradient le long du bord, ajustement robuste des quatre droites puis largeur/hauteur derivees directement des paires de droites opposees. Si V4 est indisponible, le moteur conserve automatiquement le calcul par coins.</item>\n";
  xml += "    <item>La distance fournie a la calibration est interpretee comme la distance physique camera-centre cible. Le moteur resout iterativement la profondeur Z hors axe avant de calculer fx/fy, au lieu d assimiler directement cette distance a Z.</item>\n";
  xml += "    <item>En mode continu PRECISE, MeasurementManager stabilise les mesures sur une fenetre de 5 acquisitions : amorcage brut sur 2 points, puis mediane/moyenne tronquee robuste. Toute transition/recentrage de viewport remet cette fenetre a zero.</item>\n";
  xml += "    <item>/continuous/stop annule immediatement la sequence JPEG en vol, interdit toute nouvelle frame fraiche et ignore tout resultat FILTER/DETECT/COMPUTE qui terminerait apres l ordre d arret. Les dernieres valeurs valides peuvent rester conservees pour affichage mais ne constituent pas de nouvelles mesures.</item>\n";
  xml += "    <item>Le mode continu exige une calibration valide et n empile jamais les cycles. sharpness=0 saute le controle de nettete et les recaptures pour flou.</item>\n";
  xml += "    <item>Dans timing, processing_ms est la somme capture+nettete+filtre+detection+calcul et orchestration_ms le temps mural restant entre les etapes. filter_decode_ms et filter_correction_ms detaillent filter_ms.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\"><response code=\"200\" content_type=\"application/xml\"/></method>\n";
  xml += "  <method name=\"runtime_status\" http=\"GET\" path=\"/api/runtime/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"camera_settings_get\" http=\"GET\" path=\"/api/camera/settings\">\n";
  xml += "    <comment>Lit resolution, pixel_format, monochrome, luminosite, contraste, exposition et gain.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/><response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\"/>\n";
  xml += "    <parameter name=\"monochrome\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"brightness\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"contrast\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"exposure_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"ae_level\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"aec_value\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..1200\"/>\n";
  xml += "    <parameter name=\"gain_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"agc_gain\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..30\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/><response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_viewport\" http=\"GET\" path=\"/api/camera/viewport\">\n";
  xml += "    <comment>Expose le viewport technique courant : mode, repere 2560x1920, fenetre capteur, sortie et facteurs de conversion.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"tracking_config\" http=\"GET\" path=\"/tracking/config\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"tracking_config_set\" http=\"GET\" path=\"/tracking/config/set\">\n";
  xml += "    <parameter name=\"enabled\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"1\"/>\n";
  xml += "    <parameter name=\"lost_cycles\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"1..10\" default=\"3\"/>\n";
  xml += "    <parameter name=\"recenter_threshold_pct\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"50..90\" default=\"70\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"tracking_status\" http=\"GET\" path=\"/tracking/status\">\n";
  xml += "    <comment>Expose enabled, supported, mode SEARCH/ZOOM_WIDE/ZOOM_MEDIUM/ZOOM_FINE/PRECISE, verrouillage cible, pertes, transitions et viewport.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_capture\" http=\"GET\" path=\"/diagnostic-jpeg/capture\">\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/><response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"jpeg_status\" http=\"GET\" path=\"/diagnostic-jpeg/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_image\" http=\"GET\" path=\"/diagnostic-jpeg/image.jpg\"><response code=\"200\" content_type=\"image/jpeg\"/><response code=\"404\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filter\" http=\"GET\" path=\"/diagnostic-jpeg/filter\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filter_status\" http=\"GET\" path=\"/diagnostic-jpeg/filter-status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filtered_image\" http=\"GET\" path=\"/diagnostic-jpeg/filtered.bmp\"><response code=\"200\" content_type=\"image/bmp\"/><response code=\"404\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"target_detect\" http=\"GET\" path=\"/target/detect\"><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\">\n";
  xml += "    <comment>Expose la cible detectee et le diagnostic de raffinement subpixel : subpixel_refined, subpixel_rms_px, subpixel_max_rms_px, subpixel_gradient, subpixel_width_px et subpixel_height_px.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\"><response code=\"200\" content_type=\"image/bmp\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"measurement_config\" http=\"GET\" path=\"/measurement/config\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_config_set\" http=\"GET\" path=\"/measurement/config/set\">\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <parameter name=\"k1\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "    <parameter name=\"k2\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "    <parameter name=\"p1\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "    <parameter name=\"p2\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "    <parameter name=\"k3\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "    <parameter name=\"clear_distortion\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"measurement_calibrate\" http=\"GET\" path=\"/measurement/calibrate\">\n";
  xml += "    <comment>distance_mm = distance physique entre le centre optique camera et le centre de la cible ; le moteur derive Z hors axe avant fx/fy.</comment>\n";
  xml += "    <parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"50..20000\"/>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"full_calibration_start\" http=\"GET\" path=\"/calibration/full/start\">\n";
  xml += "    <comment>Mode autonome ESP : arret du continu, tracking SEARCH vers PRECISE natif 800x600, 10 mesures valides par defaut puis moyenne fx/fy dans le repere 2560x1920. distance_mm est la distance physique camera-centre cible, pas Z. Le demarrage repond avec un JSON minimal et est idempotent si une calibration est deja en cours.</comment>\n";
  xml += "    <parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"50..20000\"/>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <parameter name=\"samples\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"3..20\" default=\"10\"/>\n";
  xml += "    <parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"full_calibration_status\" http=\"GET\" path=\"/calibration/full/status\">\n";
  xml += "    <comment>Expose progression, mode de tracking, tentative courante, cible, dernier echantillon, moyenne/ecart-type en cours, preview_attempt et preview_mode pour correler exactement chaque image.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"full_calibration_preview\" http=\"GET\" path=\"/calibration/full/preview.bmp\">\n";
  xml += "    <comment>Apercu fige de la derniere tentative de calibration, afin que le PC affiche exactement la capture correspondant a preview_attempt.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/><response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"full_calibration_cancel\" http=\"GET\" path=\"/calibration/full/cancel\"><response code=\"200\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\"><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\">\n";
  xml += "    <comment>Retourne measurement (valeur publiee), raw_measurement (derniere mesure brute) et stabilization avec sample_count/window_size/distance_stddev_mm/distance_span_mm. measurement/raw_measurement exposent edge_v4_used et apparent_width_px/apparent_height_px. La stabilisation est active uniquement sur les acquisitions PRECISE du mode continu.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"continuous_start\" http=\"GET\" path=\"/continuous/start\">\n";
  xml += "    <parameter name=\"interval_ms\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"200..10000\"/>\n";
  xml += "    <parameter name=\"sharpness\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <parameter name=\"artifact_correction\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"continuous_stop\" http=\"GET\" path=\"/continuous/stop\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"continuous_status\" http=\"GET\" path=\"/continuous/status\">\n";
  xml += "    <comment>Expose etat, options pipeline, compteurs, nettete, timing et bloc tracking. tracking.mode/roi_* decrivent le viewport prepare pour la prochaine capture ; tracking.capture_mode/capture_roi_* decrivent le viewport qui a reellement produit le dernier cycle et son apercu.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";
  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
