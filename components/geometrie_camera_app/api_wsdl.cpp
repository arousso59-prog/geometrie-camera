#include "api_wsdl.h"

#include <string>

namespace esphome {
namespace geometrie_camera_app {

ApiWsdlHandler::ApiWsdlHandler(
    CameraResolutionController *resolution_controller)
    : resolution_controller_(resolution_controller) {}

bool ApiWsdlHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET) return false;
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(url_buf) == "/api/wsdl";
}

void ApiWsdlHandler::handleRequest(AsyncWebServerRequest *request) {
  std::string xml;
  xml.reserve(7000);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"27\">\n";
  xml += "  <description>API operationnelle du capteur de geometrie. Le tracking haute precision et les reglages optiques sont automatiques.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Le tracking SEARCH vers PRECISE est permanent et ne possede plus de commande activation/desactivation.</item>\n";
  xml += "    <item>Les reglages camera sont internes a la calibration automatique et ne sont plus exposes en commande manuelle.</item>\n";
  xml += "    <item>Le pipeline image est capture JPEG, decodage gris, detection V6.1 puis mesure. Le controle de nettete et la correction d artefacts ont ete retires.</item>\n";
  xml += "    <item>V27 fige le reglage camera valide V25 et la mesure officielle V6.1 avec stabilisation robuste sur 5 mesures. V5/V6 restent diagnostics uniquement.</item>\n";
  xml += "    <item>SEARCH, ZOOM_WIDE, ZOOM_MEDIUM et ZOOM_FINE servent uniquement au tracking. Seul PRECISE natif 800x600 produit une mesure.</item>\n";
  xml += "    <item>La calibration, la prise de mesure et le mode continu restent les trois usages operationnels conserves.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\"/>\n";
  xml += "  <method name=\"runtime_status\" http=\"GET\" path=\"/api/runtime/status\"/>\n";
  xml += "  <method name=\"camera_viewport\" http=\"GET\" path=\"/api/camera/viewport\"><comment>Diagnostic lecture seule du viewport courant.</comment></method>\n";
  xml += "  <method name=\"tracking_status\" http=\"GET\" path=\"/tracking/status\"><comment>Diagnostic lecture seule du suivi haute precision permanent.</comment></method>\n";
  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\"><comment>Diagnostic lecture seule V5/V6/V6.1 et qualite individuelle des quatre bords.</comment></method>\n";
  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\"/>\n";

  xml += "  <method name=\"measurement_config\" http=\"GET\" path=\"/measurement/config\"/>\n";
  xml += "  <method name=\"measurement_config_set\" http=\"GET\" path=\"/measurement/config/set\">\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"false\" type=\"number\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"measurement_compute\" http=\"GET\" path=\"/measurement/compute\"/>\n";
  xml += "  <method name=\"measurement_status\" http=\"GET\" path=\"/measurement/status\"><comment>Methode officielle V6.1-robust5 figee; raw_measurement.precision_diag conserve V5/V6 en diagnostic.</comment></method>\n";

  xml += "  <method name=\"full_calibration_start\" http=\"GET\" path=\"/calibration/full/start\">\n";
  xml += "    <comment>Calibration autonome : tracking jusqu a PRECISE, auto-reglage optique, puis serie de mesures.</comment>\n";
  xml += "    <parameter name=\"distance_mm\" location=\"query\" required=\"true\" type=\"number\"/>\n";
  xml += "    <parameter name=\"target_size_mm\" location=\"query\" required=\"true\" type=\"number\"/>\n";
  xml += "    <parameter name=\"samples\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"3..20\" default=\"10\"/>\n";
  xml += "    <parameter name=\"force\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\" default=\"0\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"full_calibration_status\" http=\"GET\" path=\"/calibration/full/status\"/>\n";
  xml += "  <method name=\"full_calibration_preview\" http=\"GET\" path=\"/calibration/full/preview.bmp\"/>\n";
  xml += "  <method name=\"full_calibration_cancel\" http=\"GET\" path=\"/calibration/full/cancel\"/>\n";

  xml += "  <method name=\"continuous_start\" http=\"GET\" path=\"/continuous/start\">\n";
  xml += "    <parameter name=\"interval_ms\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"200..10000\"/>\n";
  xml += "  </method>\n";
  xml += "  <method name=\"continuous_stop\" http=\"GET\" path=\"/continuous/stop\"/>\n";
  xml += "  <method name=\"continuous_status\" http=\"GET\" path=\"/continuous/status\">\n";
  xml += "    <comment>Expose compteurs, tracking et timing capture/decode/detection/calcul.</comment>\n";
  xml += "  </method>\n";
  xml += "</api>\n";

  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
