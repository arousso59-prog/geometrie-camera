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
  xml.reserve(20480);
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
  xml += "    <comment>Expose les intervalles entre passages de la boucle applicative, la memoire disponible et les traitements camera en attente.</comment>\n";
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

  xml += "  <method name=\"api_image\" http=\"GET\" path=\"/image.jpg\">\n";
  xml += "    <comment>Retourne l image courante du CameraManager dans son type MIME natif.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_get\" http=\"GET\" path=\"/api/camera/settings\">\n";
  xml += "    <comment>Lit les reglages d acquisition directement depuis le capteur actif.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_settings_set\" http=\"GET\" path=\"/api/camera/settings/set\">\n";
  xml += "    <comment>Modifie un ou plusieurs reglages d acquisition du capteur. Route temporaire de mise au point.</comment>\n";
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

  xml += "  <method name=\"camera_timing_get\" http=\"GET\" path=\"/api/camera/timing\">\n";
  xml += "    <comment>Lit le diagnostic bas niveau OV5640 : PCLK, VFIFO, HTS/VTS, JPEG mode 0x4713, HREF blanking 0x471F et eventuelle reference memorisee.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_timing_set\" http=\"GET\" path=\"/api/camera/timing/set\">\n";
  xml += "    <comment>Modifie uniquement des registres OV5640 explicitement autorises avec readback. La premiere modification HTS/VTS/JPEG/HREF memorise automatiquement la reference courante.</comment>\n";
  xml += "    <parameter name=\"pclk_divider\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"1..31\">Diviseur PCLK du registre 0x3824.</parameter>\n";
  xml += "    <parameter name=\"hts\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"1..65535\">Horizontal Total Size, registres 0x380C/0x380D. Decimal ou notation 0x...</parameter>\n";
  xml += "    <parameter name=\"vts\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"1..65535\">Vertical Total Size, registres 0x380E/0x380F. Decimal ou notation 0x...</parameter>\n";
  xml += "    <parameter name=\"jpeg_mode\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"2,3\">Valeur de test limitee aux modes JPEG 2 et 3 du registre 0x4713.</parameter>\n";
  xml += "    <parameter name=\"href_blanking\" location=\"query\" required=\"false\" type=\"integer\" allowed=\"0..255\">Valeur du controle HREF DVP 0x471F. Decimal ou notation 0x...</parameter>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"camera_timing_restore\" http=\"GET\" path=\"/api/camera/timing/restore\">\n";
  xml += "    <comment>Restaure HTS, VTS, JPEG mode et HREF blanking aux valeurs memorisees avant la premiere modification de la serie de test. Ne restaure pas le diviseur PCLK.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"409\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_capture\" http=\"GET\" path=\"/diagnostic/capture\">\n";
  xml += "    <comment>Declenche une capture brute lorsque la camera est configuree en GRAYSCALE.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_status\" http=\"GET\" path=\"/diagnostic/status\">\n";
  xml += "    <comment>Etat, statistiques et timings du diagnostic GRAYSCALE.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_image\" http=\"GET\" path=\"/diagnostic/raw.bmp\">\n";
  xml += "    <comment>Retourne le BMP 8 bits pleine resolution lorsqu il est disponible.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"diagnostic_preview\" http=\"GET\" path=\"/diagnostic/preview.bmp\">\n";
  xml += "    <comment>Retourne un BMP 8 bits reduit pour controle visuel rapide d une capture GRAYSCALE.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/bmp\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_capture\" http=\"GET\" path=\"/diagnostic-jpeg/capture\">\n";
  xml += "    <comment>Purge la frame pre-acquise par ESPHome puis demande une frame JPEG fraiche produite par l ISP du capteur.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la capture JPEG. Un changement de resolution peut reprogrammer les registres OV5640 ; choisir la resolution avant une serie de tests de timing/JPEG DVP.</parameter>\n";
  xml += "    <response code=\"202\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"400\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"503\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_status\" http=\"GET\" path=\"/diagnostic-jpeg/status\">\n";
  xml += "    <comment>Expose resolution active, dimensions, taille JPEG, marqueurs SOI/EOI, nombre de frames purgees et timings de la frame fraiche.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"application/json\"/>\n";
  xml += "    <response code=\"500\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"jpeg_image\" http=\"GET\" path=\"/diagnostic-jpeg/image.jpg\">\n";
  xml += "    <comment>Retourne sans recompression la derniere frame JPEG fraiche copiee depuis le framebuffer camera.</comment>\n";
  xml += "    <response code=\"200\" content_type=\"image/jpeg\"/>\n";
  xml += "    <response code=\"404\" content_type=\"application/json\"/>\n";
  xml += "  </method>\n";

  xml += "  <method name=\"rgb565_capture\" http=\"GET\" path=\"/diagnostic-rgb565/capture\">\n";
  xml += "    <comment>Declenche le diagnostic RGB565 lorsque la camera est configuree en RGB565.</comment>\n";
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
  xml += "    <comment>Declenche une acquisition et une recherche de cible lorsque la camera est configuree en GRAYSCALE.</comment>\n";
  xml += "    <parameter name=\"resolution\" location=\"query\" required=\"false\" type=\"string\" allowed=\"";
  xml += allowed_resolutions;
  xml += "\">Resolution a appliquer avant la recherche.</parameter>\n";
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