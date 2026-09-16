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
  xml.reserve(13312);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"5\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>API de la voie camera validee : OV5640 JPEG, correction grayscale, detection cible, reglages et diagnostic runtime.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes de commande de mise au point utilisent encore HTTP GET.</item>\n";
  xml += "    <item>La detection cible travaille uniquement sur la derniere image grayscale corrigee.</item>\n";
  xml += "    <item>target_found indique l acceptation finale ; le bloc target expose le meilleur candidat meme s il reste sous le seuil.</item>\n";
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
  xml += "    <comment>Modifie les reglages utiles a la mise au point et a la future calibration optique.</comment>\n";
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
  xml += "    <comment>Decode la derniere frame JPEG en grayscale et applique le correcteur V3 des artefacts verts/noirs.</comment>\n";
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
  xml += "    <comment>Lance TargetDetector V3 sur la derniere image grayscale corrigee. La V3 limite la plage de taille, moyenne localement les cellules et valide la coherence du cadre noir. Ne relance ni capture ni filtrage.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\">\n";
  xml += "    <comment>Relit le dernier resultat. Le bloc target contient toujours le meilleur candidat disponible, meme si target_found vaut false.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_preview\" http=\"GET\" path=\"/target/preview.bmp\">\n";
  xml += "    <comment>Genere une miniature grayscale de largeur maximale 640 px avec un rectangle noir/blanc autour du meilleur candidat de la derniere detection.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";

  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
