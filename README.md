# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- capteur confirmé par PID : OV5640 (`0x5640`) ;
- PSRAM : 8 Mo octal ;
- résolution de travail maximale exposée par ESPHome 2026.7.3 : 2560×1920 ;
- XCLK retenu actuellement : 8 MHz.

Brochage caméra :

```text
SIOD  GPIO4
SIOC  GPIO5
VSYNC GPIO6
HREF  GPIO7
XCLK  GPIO15
Y2    GPIO11
Y3    GPIO9
Y4    GPIO8
Y5    GPIO10
Y6    GPIO12
Y7    GPIO18
Y8    GPIO17
Y9    GPIO16
PCLK  GPIO13
```

## Voie image retenue

Les essais GRAYSCALE brut ont montré davantage de bruit et un coût mémoire élevé. Le JPEG natif de l'OV5640 donne une image nettement moins bruitée mais présente un motif parasite régulier vert/noir.

La méthode retenue est désormais :

```text
OV5640 JPEG
   ↓
frame fraîche (purge de la frame ESPHome pré-acquise)
   ↓
décodage par blocs vers grayscale 8 bits
   ↓
JpegArtifactCorrector V3
   ↓
buffer grayscale corrigé
   ↓
TargetDetectionService
   ↓
TargetDetector
   ↓
TargetObservation
   ↓
distance / orientation / géométrie
```

Le correcteur V3 supprime la grande majorité des impulsions vertes et une part importante des petits segments noirs sans appliquer de flou global. Le buffer grayscale corrigé est exposé directement au code C++ ; le BMP n'est qu'une visualisation de diagnostic.

## Architecture actuelle

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

Sous-systèmes conservés :

- `GeometrieCameraApp` : orchestration uniquement ;
- `JpegDiagnostic` : acquisition JPEG fraîche et stockage persistant ;
- `JpegFilteredDiagnostic` : décodage grayscale et préparation de l'image corrigée ;
- `JpegArtifactCorrector` : correction pure des artefacts ;
- `TargetDetectionService` : adaptation sans copie du buffer corrigé vers le détecteur ;
- `TargetDetector` : recherche de la cible 7×7 ;
- `MeasurementManager` / `GeometryMeasurementEngine` : future chaîne distance/orientation/angles ;
- `CameraResolutionController` ;
- `CameraSettingsController` / API ;
- `RuntimeDiagnostics` / API ;
- `ApiWsdlHandler`.

Les anciens chemins de test GRAYSCALE, RGB565, OV3660, registres OV5640, placeholder et TargetSearch GRAYSCALE ont été supprimés après validation de la voie JPEG.

## API actuelle

```text
GET /api/wsdl
GET /api/runtime/status

GET /api/camera/settings
GET /api/camera/settings/set?<parametres>

GET /diagnostic-jpeg/capture?resolution=<optionnel>
GET /diagnostic-jpeg/status
GET /diagnostic-jpeg/image.jpg
GET /diagnostic-jpeg/filter
GET /diagnostic-jpeg/filter-status
GET /diagnostic-jpeg/filtered.bmp

GET /target/detect
GET /target/status
```

`/target/detect` lance la recherche sur la dernière image déjà filtrée. Il ne déclenche volontairement ni nouvelle capture ni nouveau filtrage pendant la phase de validation.

Le résultat contient : cible trouvée ou non, centre en pixels, taille, rotation discrète 0/90/180/270 degrés, qualité et temps de détection.

`/api/wsdl` reste la référence du contrat HTTP et doit être mis à jour dans le même changement que toute évolution d'API.

## Étape actuelle

Valider la cible réelle sur la voie JPEG corrigée :

1. capture et filtrage de l'image ;
2. recherche pleine image avec `/target/detect` ;
3. validation de la position, de la taille, de la rotation discrète et du score ;
4. estimation de distance ;
5. estimation d'orientation fine ;
6. calibration optique et comparaison aux données constructeur.

Les optimisations de vitesse seront faites après cette validation fonctionnelle. La stratégie prévue est de mémoriser la dernière boîte de cible et, après la première recherche globale, de limiter autant que possible le décodage/correction et la recherche à une ROI autour de la cible. En cas de perte de cible, retour automatique à une recherche globale.
