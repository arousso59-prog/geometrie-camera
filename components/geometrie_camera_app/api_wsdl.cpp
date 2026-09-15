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
  xml.reserve(12288);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<api name=\"geometrie-camera\" version=\"1\" style=\"REST-over-HTTP\">\n";
  xml += "  <description>Catalogue WSDL-like des routes HTTP du projet. Ce document doit etre maintenu avec toute evolution d API.</description>\n";
  xml += "  <conventions>\n";
  xml += "    <item>Les routes sont actuellement en HTTP GET.</item>\n";
  xml += "    <item>Les reponses de statut et de commande sont en JSON sauf les images et ce document XML.</item>\n";
  xml += "    <item>Les commentaires de chaque methode sont a completer au fil de la realisation.</item>\n";
  xml += "  </conventions>\n";

  xml += "  <method name=\"api_wsdl\" http=\"GET\" path=\"/api/wsdl\">\n";
  xml += "    <comment>Liste l ensemble des methodes HTTP, parametres et commentaires du firmware.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/xml\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"api_status\" http=\"GET\" path=\"/api/status\">\n";
  xml += "    <comment>Etat general du service camera historique et metadonnees de la derniere image.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
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

  xml += "  <method name=\"api_image\" http=\"GET\" path=\"/image.jpg\">\n";
  xml += "    <comment>Retourne l image courante du CameraManager dans son type MIME natif.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_capture\" http=\"GET\" path=\"/diagnostic/capture\">\n";
  xml += "    <comment>Declenche une capture brute GRAYSCALE de diagnostic.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture. Si absente, conserve la resolution active.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_status\" http=\"GET\" path=\"/diagnostic/status\">\n";
  xml += "    <comment>Etat du diagnostic GRAYSCALE : identite capteur relue a la demande, PID, resolution maximale, resolution active, statistiques brutes et timings.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_image\" http=\"GET\" path=\"/diagnostic/raw.bmp\">\n";
  xml += "    <comment>Retourne le BMP 8 bits de la derniere capture GRAYSCALE de diagnostic.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"rgb565_capture\" http=\"GET\" path=\"/diagnostic-rgb565/capture\">\n";
  xml += "    <comment>Declenche le diagnostic temporaire RGB565. Route conservee pour les essais materiels.</comment>\n";
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
  xml += "    <comment>Declenche une acquisition GRAYSCALE et une recherche de cible sur le framebuffer recu.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la recherche. Si absente, conserve la resolution active.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_status\" http=\"GET\" path=\"/target/status\">\n";
  xml += "    <comment>Etat de la derniere recherche : identite capteur relue a la demande, PID, resolution, cible trouvee, boite, orientation, qualite et timings.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"target_image\" http=\"GET\" path=\"/target/image.bmp\">\n";
  xml += "    <comment>Retourne le BMP de visualisation partage. Si une cible est reconnue, son cadre vert est dessine dans ce meme buffer.</comment>\n";
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
