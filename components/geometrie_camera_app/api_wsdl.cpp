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
  xml.reserve(17920);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"1\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>Catalogue WSDL-like des routes HTTP du projet. Ce document doit etre maintenu avec toute evolution d API.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes sont actuellement en HTTP GET.</item>\n";
  xml += "    <item>Les reponses de statut et de commande sont en JSON sauf les images et ce document XML.</item>\n";
  xml += "    <item>Les diagnostics dependants d un format image exigent que ESP32Camera soit configure dans ce format.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\">\n";
  xml += "    <comment>Liste l ensemble des methodes HTTP, parametres et commentaires du firmware.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/xml\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"api_status\" http=\"GET\" path=\"/api/status\">\n";
  xml += "    <comment>Etat general du service camera historique et metadonnees de la derniere image.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"runtime_status\" http=\"GET\" path=\"/api/runtime/status\">\n";
  xml += "    <comment>Expose les intervalles entre passages de la boucle applicative, la memoire interne/PSRAM disponible et les traitements camera actuellement en attente.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"api_capture\" http=\"GET\" path=\"/api/capture\">\n";
  xml += "    <comment>Declenche une capture via CameraManager. Cette route utilise encore le chemin historique ImageProvider.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"api_measure\" http=\"GET\" path=\"/api/measure\">\n";
  xml += "    <comment>Retourne la derniere mesure geometrique connue : yaw, pitch, roll et qualite.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_get\" http=\"GET\" path=\"/api/camera/settings\">\n";
  xml += "    <comment>Lit les reglages d acquisition directement depuis le capteur actif et expose leurs plages de test.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <comment>Modifie un ou plusieurs reglages d acquisition du capteur. Route temporaire de mise au point avant stabilisation de l API v1.</comment>\n";
  xml += "    <parameter name=\"brightness\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\">Luminosite capteur.</parameter>\n";
  xml += "    <parameter name=\"contrast\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\">Contraste capteur.</parameter>\n";
  xml += "    <parameter name=\"exposure_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\">Active ou desactive l exposition automatique.</parameter>\n";
  xml += "    <parameter name=\"ae_level\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"-2..2\">Compensation du niveau d exposition automatique.</parameter>\n";
  xml += "    <parameter name=\"aec_value\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..1200\">Valeur d exposition manuelle, utile quand exposure_ctrl vaut 0.</parameter>\n";
  xml += "    <parameter name=\"gain_ctrl\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0,1\">Active ou desactive le gain automatique.</parameter>\n";
  xml += "    <parameter name=\"agc_gain\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..30\">Gain manuel, utile quand gain_ctrl vaut 0.</parameter>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"api_image\" http=\"GET\" path=\"/image.jpg\">\n";
  xml += "    <comment>Retourne l image courante du CameraManager dans son type MIME natif.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_capture\" http=\"GET\" path=\"/diagnostic/capture\">\n";
  xml += "    <comment>Declenche une capture brute GRAYSCALE lorsque la camera est configuree en GRAYSCALE. En QSXGA le diagnostic conserve surtout statistiques et preview sans dupliquer la frame en BMP pleine resolution.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture. Si absente, conserve la resolution active.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_status\" http=\"GET\" path=\"/diagnostic/status\">\n";
  xml += "    <comment>Etat du diagnostic GRAYSCALE : capteur, resolution, disponibilite BMP plein format et preview, statistiques brutes et timings.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_image\" http=\"GET\" path=\"/diagnostic/raw.bmp\">\n";
  xml += "    <comment>Retourne le BMP 8 bits pleine resolution lorsqu il est disponible.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_preview\" http=\"GET\" path=\"/diagnostic/preview.bmp\">\n";
  xml += "    <comment>Retourne un BMP 8 bits reduit a 640x480 maximum pour controle visuel rapide d une capture GRAYSCALE.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_capture\" http=\"GET\" path=\"/diagnostic-jpeg/capture\">\n";
  xml += "    <comment>Purge la frame pre-acquise par ESPHome puis demande une frame JPEG fraiche produite par l ISP du capteur. La frame publiee correspond donc a la requete courante et non a la capture precedente.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture JPEG. La frame en attente est purgee apres le changement de resolution.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_status\" http=\"GET\" path=\"/diagnostic-jpeg/status\">\n";
  xml += "    <comment>Expose resolution active, dimensions, taille JPEG, marqueurs SOI/EOI, nombre de frames purgees et timings de la demande fraiche.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_image\" http=\"GET\" path=\"/diagnostic-jpeg/image.jpg\">\n";
  xml += "    <comment>Retourne sans recompression la derniere frame JPEG fraiche copiee depuis le framebuffer camera.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"rgb565_capture\" http=\"GET\" path=\"/diagnostic-rgb565/capture\">\n";
  xml += "    <comment>Declenche le diagnostic temporaire RGB565 lorsque la camera est configuree en RGB565.</comment>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"rgb565_status\" http=\"GET\" path=\"/diagnostic-rgb565/status\">\n";
  xml += "    <comment>Etat et dimensions de la derniere acquisition RGB565.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"rgb565_image\" http=\"GET\" path=\"/diagnostic-rgb565/raw.bmp\">\n";
  xml += "    <comment>Retourne le BMP 24 bits converti depuis la derniere frame RGB565.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_search\" http=\"GET\" path=\"/target/search\">\n";
  xml += "    <comment>Declenche une acquisition et une recherche de cible lorsque la camera est configuree en GRAYSCALE. Cette route n est pas utilisee pendant le test JPEG natif.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la recherche. Si absente, conserve la resolution active.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\">\n";
  xml += "    <comment>Etat de la derniere recherche : identite capteur, resolution, cible trouvee, boite, orientation, qualite et timings.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_image\" http=\"GET\" path=\"/target/image.bmp\">\n";
  xml += "    <comment>Retourne le BMP de visualisation de la derniere recherche GRAYSCALE lorsqu il est disponible.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "</api>\n";

  auto *response = request->beginResponse(200, "application/xml", xml);
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

}  // namespace geometrie_camera_app
}  // namespace esphome
