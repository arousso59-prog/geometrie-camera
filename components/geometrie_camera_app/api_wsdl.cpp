#include "api_wsdl.h"

#include <string>

#include "camera_resolution_controller.h"

namespace esphome {
namespace geometrie_camera_app {

ApiWsdlHandler::ApiWsdlHandler(CameraResolutionController *resolution_controller)
    : resolution_controller_(resolution_controller) {}

bool ApiWsdlHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) {
    return false;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(url_buf) == "/api/wsdl";
}

void ApiWsdlHandler::handleRequest(AsyncWebServerRequest *request) {
  const char *allowed_resolutions = this->resolution_controller_ != nullptr
                                        ? CameraResolutionController::allowed_resolutions_text()
                                        : "unknown";

  std::string xml;
  xml.reserve(19000);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"11\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API camera OV5640 : capture JPEG, correction grayscale, detection cible, calibration optique verrouillee, distance robuste par taille apparente et orientation du plan cible.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande de mise au point utilisent encore HTTP GET.</item>\n";
  xml += "    <item>La detection cible travaille uniquement sur la derniere image grayscale corrigee.</item>\n";
  xml += "    <item>La V5 separe localisation de la cible et lecture du code 7x7 ; target_found indique l acceptation finale.</item>\n";
  xml += "    <item>Depuis la V5.4, les coins des candidats peuvent etre raffines sur l image pleine resolution et le code 7x7 est projete par homographie ; le candidat brut reste teste en secours.</item>\n";
  xml += "    <item>La mesure exige une calibration de focale a distance connue. La cible doit etre approximativement centree et de face pendant cette calibration.</item>\n";
  xml += "    <item>Une calibration valide est verrouillee contre une nouvelle calibration accidentelle. Le parametre force=1 est requis pour la remplacer explicitement.</item>\n";
  xml += "    <item>Le repere camera est X vers la droite, Y vers le bas et Z vers l avant. z_mm provient de la taille apparente de la cible ; distance_mm est la distance euclidienne au centre.</item>\n";
  xml += "    <item>z_from_width_mm et z_from_height_mm exposent les deux estimations independantes. La plus petite est retenue comme z_mm afin de limiter la surestimation due a une dimension raccourcie par perspective.</item>\n";
  xml += "    <item>La decomposition homographique ne pilote plus la distance. Elle ne valide yaw/pitch/roll que si pose_z_mm reste a moins de 25 pourcent de z_mm ; sinon pose_valid=false.</item>\n";
  xml += "    <item>La calibration est stockee pour la resolution de reference puis fx, fy, cx et cy sont redimensionnes proportionnellement pour les autres resolutions de meme cadrage optique.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\">\n";
  xml += "    <comment>Liste les routes HTTP actuellement compilees dans le firmware.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/xml\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"runtime_status\" http=\"GET\" path=\"/api/runtime/status\">\n";
  xml += "    <comment>Expose les intervalles de boucle, la memoire interne/PSRAM et l etat de capture JPEG.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_get\" http=\"GET\" path=\"/api/camera/settings\">\n";
  xml += "    <comment>Lit les reglages d acquisition directement depuis le capteur actif.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <comment>Modifie les reglages utiles a la mise au point et a la calibration optique.</comment>\n";
  xml += "    <parameter name=\"brightness\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"contrast\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"exposure_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"ae_level\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"aec_value\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..1200\"/>\n";
  xml += "    <parameter name=\"gain_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"agc_gain\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..30\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_capture\" http=\"GET\" path=\"/diagnostic-jpeg/capture\">\n";
  xml += "    <comment>Purge la frame pre-acquise par ESPHome puis demande une frame JPEG fraiche a l OV5640.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_status\" http=\"GET\" path=\"/diagnostic-jpeg/status\">\n";
  xml += "    <comment>Expose dimensions, taille JPEG, marqueurs SOI/EOI, purge de frame et timings de capture.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_image\" http=\"GET\" path=\"/diagnostic-jpeg/image.jpg\">\n";
  xml += "    <comment>Retourne sans recompression la derniere frame JPEG fraiche.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_filter\" http=\"GET\" path=\"/diagnostic-jpeg/filter\">\n";
  xml += "    <comment>Decode la derniere frame JPEG en grayscale et applique le correcteur sparse optimise des artefacts verts/noirs.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_filter_status\" http=\"GET\" path=\"/diagnostic-jpeg/filter-status\">\n";
  xml += "    <comment>Expose les temps de decodage/correction et les statistiques du correcteur d artefacts.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_filtered_image\" http=\"GET\" path=\"/diagnostic-jpeg/filtered.bmp\">\n";
  xml += "    <comment>Retourne le BMP grayscale corrige pour validation visuelle. Le pipeline cible utilise directement le buffer grayscale.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_detect\" http=\"GET\" path=\"/target/detect\">\n";
  xml += "    <comment>Lance TargetDetector V5.4 sur la derniere image grayscale corrigee. Les quatre coins du meilleur code valide sont conserves en interne pour la mesure. Ne relance ni capture ni filtrage.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\">\n";
  xml += "    <comment>Relit le dernier resultat V5.4 sans relancer la detection.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\">\n";
  xml += "    <comment>Genere une miniature grayscale annotee du meilleur resultat de la derniere detection.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"measurement_config\" http=\"GET\" path=\"/measurement/config\">\n";
  xml += "    <comment>Expose la taille physique de cible, la calibration, son verrou et les diagnostics de mesure.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"measurement_config_set\" http=\"GET\" path=\"/measurement/config/set\">\n";
  xml += "    <comment>Definit la taille physique du carre cible. Modifier cette taille invalide la calibration de focale precedente et leve son verrou.</comment>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"measurement_calibrate\" http=\"GET\" path=\"/measurement/calibrate\">\n";
  xml += "    <comment>Calibre fx et fy a partir de la derniere cible detectee, placee approximativement de face et centree a une distance connue. Une calibration deja valide est protegee contre l ecrasement accidentel ; force=1 autorise explicitement son remplacement.</comment>\n";
  xml += "    <parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"50..20000\"/>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\">calibration_locked, current_target_detection_required ou target_not_found.</response>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\">\n";
  xml += "    <comment>Calcule d abord z depuis la taille apparente des quatre coins calibres, puis X/Y et distance depuis le rayon du centre cible. Expose z_from_width_mm et z_from_height_mm. L homographie sert seulement a yaw/pitch/roll et pose_valid reste faux si pose_z_mm differe de plus de 25 pourcent de z_mm.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\">\n";
  xml += "    <comment>Relit la derniere mesure, pose_valid, les deux estimations de profondeur et la calibration verrouillee sans nouveau calcul.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";

  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
