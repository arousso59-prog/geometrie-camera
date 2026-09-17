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
  xml.reserve(26000);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"19\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API camera OV5640 : capture JPEG, reglages capteur, controle nettete cible, correction grayscale, detection cible, calibration optique verrouillee, distance robuste et acquisition continue.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande de mise au point utilisent encore HTTP GET.</item>\n";
  xml += "    <item>Le pixel_format actif est expose par /api/camera/settings. JPEG et GRAYSCALE sont des formats de demarrage et leur changement necessite recompilation/reflash.</item>\n";
  xml += "    <item>monochrome est un effet grayscale runtime du capteur OV5640. Il ne change pas le pixel_format ni le type JPEG du framebuffer.</item>\n";
  xml += "    <item>La detection cible travaille uniquement sur la derniere image grayscale corrigee.</item>\n";
  xml += "    <item>La mesure exige une calibration de focale a distance connue ; une calibration valide est verrouillee et force=1 est requis pour la remplacer.</item>\n";
  xml += "    <item>Le repere camera est X vers la droite, Y vers le bas et Z vers l avant. z_mm provient de la taille apparente ; distance_mm est la distance euclidienne au centre.</item>\n";
  xml += "    <item>z_from_width_mm et z_from_height_mm sont les deux estimations independantes ; la plus petite est retenue comme z_mm.</item>\n";
  xml += "    <item>L homographie ne pilote plus la distance et ne valide yaw/pitch/roll que si pose_z_mm reste a moins de 25 pourcent de z_mm.</item>\n";
  xml += "    <item>La calibration est stockee pour une resolution de reference et redimensionnee proportionnellement pour les autres resolutions de meme cadrage.</item>\n";
  xml += "    <item>Le mode continu est interdit sans calibration valide et n empile jamais les cycles.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\"><response code=\"200\" content_type=\"application/xml\"/></method>\n";

  xml += "  <method name=\"runtime_status\" http=\"GET\" path=\"/api/runtime/status\">\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_get\" http=\"GET\" path=\"/api/camera/settings\">\n";
  xml += "    <comment>Lit resolution, pixel_format, monochrome, luminosite, contraste, exposition et gain.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/><response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <comment>Modifie les reglages camera runtime. pixel_format reste un reglage de demarrage.</comment>\n";
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
  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\"><response code=\"200\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "  <method name=\"continuous_start\" http=\"GET\" path=\"/continuous/start\">\n";
  xml += "    <parameter name=\"interval_ms\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"200..10000\"/>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/><response code=\"400\" content_type=\"application/json\"/><response code=\"409\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"continuous_stop\" http=\"GET\" path=\"/continuous/stop\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";
  xml += "  <method name=\"continuous_status\" http=\"GET\" path=\"/continuous/status\"><response code=\"200\" content_type=\"application/json\"/><response code=\"500\" content_type=\"application/json\"/></method>\n";

  xml += "</api>\n";
  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
