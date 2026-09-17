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
  xml.reserve(24000);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"18\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API camera OV5640 : capture JPEG, controle nettete cible, correction grayscale, detection cible, calibration optique verrouillee, distance robuste et acquisition continue.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande de mise au point utilisent encore HTTP GET.</item>\n";
  xml += "    <item>Le format pixel actif est expose par /api/camera/settings. JPEG et GRAYSCALE sont des formats de demarrage : leur changement necessite recompilation/reflash car les buffers camera/DMA sont dimensionnes a l initialisation. Aucun changement runtime de pixel_format n est supporte.</item>\n";
  xml += "    <item>La detection cible travaille uniquement sur la derniere image grayscale corrigee.</item>\n";
  xml += "    <item>La mesure exige une calibration de focale a distance connue ; une calibration valide est verrouillee et force=1 est requis pour la remplacer.</item>\n";
  xml += "    <item>Le repere camera est X vers la droite, Y vers le bas et Z vers l avant. z_mm provient de la taille apparente ; distance_mm est la distance euclidienne au centre.</item>\n";
  xml += "    <item>z_from_width_mm et z_from_height_mm sont les deux estimations independantes ; la plus petite est retenue comme z_mm.</item>\n";
  xml += "    <item>L homographie ne pilote plus la distance et ne valide yaw/pitch/roll que si pose_z_mm reste a moins de 25 pourcent de z_mm.</item>\n";
  xml += "    <item>La calibration est stockee pour une resolution de reference et redimensionnee proportionnellement pour les autres resolutions de meme cadrage.</item>\n";
  xml += "    <item>Le mode continu est interdit sans calibration valide. Si la calibration est invalidee pendant son fonctionnement, il s arrete.</item>\n";
  xml += "    <item>Le mode continu reutilise la resolution camera active et enchaine capture, controle nettete, filtre, detection et mesure sans empiler les cycles.</item>\n";
  xml += "    <item>Le controle nettete utilise uniquement une ROI autour de la derniere cible valide. Sans cible precedente connue, aucune image n est rejetee pour flou avant detection.</item>\n";
  xml += "    <item>La nettete ROI decode le JPEG en 1/4. La ROI vaut environ 2.5 fois la taille de cible, avec un minimum de 48 pixels dans l image source.</item>\n";
  xml += "    <item>Le score de nettete correspond a la moyenne des 20 pourcent de reponses Laplaciennes les plus fortes dans la ROI afin de privilegier les contours noir/blanc de la cible plutot que le decor uniforme.</item>\n";
  xml += "    <item>Une image dont le score tombe sous 45 pourcent de la reference ROI peut etre recapturee immediatement, au maximum deux fois par cycle. Apres deux recaptures, le pipeline continue afin de ne pas se bloquer.</item>\n";
  xml += "    <item>La reference de nettete n est mise a jour qu apres une detection de cible valide. Chaque mesure est bornee a plus ou moins 15 pourcent de la reference precedente puis integree avec un filtre lent 1/8 pour eviter les derives dues aux pics.</item>\n";
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
  xml += "    <comment>Lit la resolution de travail, le pixel_format actif et les reglages d acquisition du capteur. pixel_format est informatif et configure au demarrage.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <comment>Modifie la resolution de travail et les reglages utiles a la mise au point/calibration. pixel_format n est pas modifiable a chaud.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\"/>\n";
  xml += "    <parameter name=\"brightness\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"contrast\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"exposure_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"ae_level\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\"/>\n";
  xml += "    <parameter name=\"aec_value\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..1200\"/>\n";
  xml += "    <parameter name=\"gain_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\"/>\n";
  xml += "    <parameter name=\"agc_gain\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..30\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_capture\" http=\"GET\" path=\"/diagnostic-jpeg/capture\">\n";
  xml += "    <comment>Purge la frame pre-acquise puis demande une frame JPEG fraiche.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture.</parameter>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_status\" http=\"GET\" path=\"/diagnostic-jpeg/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_image\" http=\"GET\" path=\"/diagnostic-jpeg/image.jpg\"><response code=\"200\" content_type=\"image/jpeg\"/><response code=\"404\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filter\" http=\"GET\" path=\"/diagnostic-jpeg/filter\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filter_status\" http=\"GET\" path=\"/diagnostic-jpeg/filter-status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"jpeg_filtered_image\" http=\"GET\" path=\"/diagnostic-jpeg/filtered.bmp\"><response code=\"200\" content_type=\"image/bmp\"/><response code=\"404\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"target_detect\" http=\"GET\" path=\"/target/detect\"><comment>Lance TargetDetector sur la derniere image grayscale corrigee.</comment><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\"><response code=\"200\" content_type=\"image/bmp\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"measurement_config\" http=\"GET\" path=\"/measurement/config\"><comment>Expose taille cible, calibration et diagnostics.</comment><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_config_set\" http=\"GET\" path=\"/measurement/config/set\">\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"measurement_calibrate\" http=\"GET\" path=\"/measurement/calibrate\">\n";
  xml += "    <parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"50..20000\"/>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\" allowed=\"1..1000\"/>\n";
  xml += "    <parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\"><comment>Calcule distance robuste, X/Y/Z, bearing et pose validee.</comment><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"continuous_start\" http=\"GET\" path=\"/continuous/start\">\n";
  xml += "    <comment>Demarre l automate capture-nettete-filtre-detection-mesure. Refuse le demarrage si aucune calibration valide n est presente. Un nouvel appel redemarre les compteurs de session, la reference de nettete et initialise la ROI depuis la derniere cible connue si possible.</comment>\n";
  xml += "    <parameter name=\"interval_ms\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"200..10000\" default=\"valeur_courante\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\">interval_ms invalide.</response>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\">calibration_required.</response>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"continuous_stop\" http=\"GET\" path=\"/continuous/stop\">\n";
  xml += "    <comment>Arrete l automate continu apres l etape courante.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"continuous_status\" http=\"GET\" path=\"/continuous/status\">\n";
  xml += "    <comment>Expose running, state, interval_ms, compteurs et dernier etat cible. timing contient capture_ms, sharpness_ms, filter_ms, detect_ms, compute_ms et cycle_ms. sharpness contient le score des 20 pourcent de contours les plus forts, la reference stabilisee, ok, capture_retries, blur_retry_count, roi_active, roi_x, roi_y, roi_width et roi_height. L etat peut prendre la valeur sharpness pendant le controle de nettete.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";
  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome