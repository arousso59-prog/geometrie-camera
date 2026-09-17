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
  xml += "<api name=\"geometrie-camera\" version=\"21\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API camera OV5640 : capture JPEG, reglages capteur, viewport ROI haute resolution, tracking cible, detection, calibration, mesure geometrique et acquisition continue.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande utilisent encore HTTP GET pendant la phase de mise au point.</item>\n";
  xml += "    <item>monochrome est un effet grayscale runtime du capteur OV5640 et ne change pas le pixel_format JPEG.</item>\n";
  xml += "    <item>Le repere camera est X vers la droite, Y vers le bas et Z vers l avant.</item>\n";
  xml += "    <item>GeometryMeasurementEngine V3 fusionne z_from_width_mm et z_from_height_mm de facon continue ; pose_z_mm reste un controle independant issu de l homographie.</item>\n";
  xml += "    <item>Le tracking automatique est configure par /tracking/config. /continuous/start ne pilote pas la ROI depuis le PC : l ESP32 gere SEARCH et PRECISE de facon autonome.</item>\n";
  xml += "    <item>SEARCH utilise un plein champ 800x600. PRECISE utilise une ROI native 800x600 dans le repere de reference 2560x1920.</item>\n";
  xml += "    <item>Les coordonnees ROI sont converties en repere de reference 2560x1920 avant le calcul de mesure ; la calibration existante est redimensionnee par GeometryMeasurementEngine selon sa resolution de reference.</item>\n";
  xml += "    <item>Apres lost_cycles pertes consecutives en PRECISE, le capteur revient automatiquement en SEARCH. La ROI PRECISE est recentree quand le centre cible quitte la zone centrale configuree.</item>\n";
  xml += "    <item>Le filtre travaille toujours sur l image de sortie, actuellement 800x600, jamais sur un buffer grayscale 2560x1920 complet.</item>\n";
  xml += "    <item>Le mode continu exige une calibration valide et n empile jamais les cycles.</item>\n";
  xml += "    <item>Dans timing, processing_ms est la somme capture+nettete+filtre+detection+calcul et orchestration_ms le temps mural restant entre les etapes.</item>\n";
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
  xml += "    <parameter name=\"enabled\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "    <parameter name=\"lost_cycles\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"1..10\" default=\"3\"/>\n";
  xml += "    <parameter name=\"recenter_threshold_pct\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"50..90\" default=\"70\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"tracking_status\" http=\"GET\" path=\"/tracking/status\">\n";
  xml += "    <comment>Expose enabled, supported, mode SEARCH/PRECISE, verrouillage cible, pertes, transitions et viewport.</comment>\n";
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
  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\"><response code=\"200\" content_type=\"image/bmp\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"measurement_config\" http=\"GET\" path=\"/measurement/config\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_config_set\" http=\"GET\" path=\"/measurement/config/set\"><parameter name=\"target_size_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"1..1000\"/><response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_calibrate\" http=\"GET\" path=\"/measurement/calibrate\"><parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\" allowed=\"50..20000\"/><parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\" allowed=\"1..1000\"/><parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/><response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\"><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"continuous_start\" http=\"GET\" path=\"/continuous/start\"><parameter name=\"interval_ms\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"200..10000\"/><response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"continuous_stop\" http=\"GET\" path=\"/continuous/stop\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"continuous_status\" http=\"GET\" path=\"/continuous/status\">\n";
  xml += "    <comment>Expose etat, compteurs, nettete, timing et bloc tracking avec mode, verrouillage, pertes et ROI de reference.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";
  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
