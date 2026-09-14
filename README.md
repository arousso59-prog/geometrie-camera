# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV3660.

## Architecture

Le projet reprend la séparation utilisée dans `equilibreuse-esp32` :

- `geometrie-camera.yaml` : configuration matérielle ESPHome, Wi-Fi, caméra et paramètres de test.
- `components/geometrie_camera_app/` : logique applicative C++.
- Le PC récupérera les mesures par HTTP/JSON et pourra demander une image de contrôle.

## Étape actuelle — V0 API bouchon

La caméra physique n'est pas encore activée tant que le brochage exact du PCB ESP32-S3 + OV3660 n'est pas validé.

En revanche, l'API HTTP est déjà fonctionnelle avec une image JPEG de test monochrome 160 × 120, moitié noire / moitié blanche. Cela permet de développer et tester la communication PC sans attendre la caméra réelle.

### Routes HTTP

#### `GET /api/status`

Retourne l'état du module et indique explicitement que le système fonctionne en mode `placeholder`.

Exemple :

```json
{
  "api_version": 1,
  "status": "ok",
  "mode": "placeholder",
  "camera_ready": true,
  "physical_camera_ready": false,
  "capture_count": 0,
  "last_capture_ms": 0,
  "image": {
    "url": "/image.jpg",
    "format": "jpeg",
    "width": 160,
    "height": 120
  }
}
```

#### `GET /api/capture`

Déclenche une capture logique. En V0, cela incrémente le compteur et met à jour le timestamp, puis renvoie l'URL de l'image bouchon.

```json
{
  "success": true,
  "placeholder": true,
  "capture_id": 1,
  "timestamp_ms": 12543,
  "width": 160,
  "height": 120,
  "size_bytes": 269,
  "image": "/image.jpg"
}
```

#### `GET /image.jpg`

Retourne directement un vrai JPEG 160 × 120 monochrome de test.

Cette route sera conservée lorsque l'OV3660 sera activée : seule la source des octets JPEG changera.

#### `GET /api/measure`

Retourne la structure de mesure prévue pour le traitement de cible. Tant que la détection de cible n'est pas développée, `valid` reste à `false`.

```json
{
  "valid": false,
  "placeholder": true,
  "timestamp_ms": 0,
  "yaw_deg": 0.0,
  "pitch_deg": 0.0,
  "roll_deg": 0.0,
  "quality": 0.0
}
```

## Arborescence

```text
geometrie-camera/
├── geometrie-camera.yaml
├── secrets.yaml.example
└── components/
    └── geometrie_camera_app/
        ├── __init__.py
        ├── geometrie_camera_app.h
        ├── geometrie_camera_app.cpp
        ├── camera_api.h
        ├── camera_api.cpp
        ├── camera_manager.h
        ├── camera_manager.cpp
        ├── placeholder_image.h
        ├── target_detector.h
        ├── target_detector.cpp
        ├── geometry_measurement.h
        └── geometry_measurement.cpp
```

## Brochage caméra

La carte est configurée comme ESP32-S3 et le capteur prévu est l'OV3660. Le brochage parallèle de la caméra dépend toutefois du PCB exact.

Le bloc `esp32_camera` reste donc volontairement désactivé dans le YAML jusqu'à validation de D0..D7, XCLK, PCLK, VSYNC, HREF et SCCB.

## Principe pour la suite

Le contrat HTTP doit rester stable :

```text
PC -> GET /api/capture
ESP -> acquisition réelle OV3660 + traitement éventuel
PC <- JSON avec métadonnées
PC -> GET /image.jpg
ESP -> JPEG réellement capturé
PC -> GET /api/measure
ESP -> position cible + angles calculés
```

Ainsi le passage du bouchon à la vraie caméra ne nécessitera pas de refaire la console PC.

## Étapes suivantes

- valider le brochage réel de la carte reçue ;
- relier `CameraManager` au composant caméra ;
- remplacer le JPEG bouchon par le dernier framebuffer OV3660 ;
- fixer exposition / gain / balance des blancs pour rendre la mesure reproductible ;
- implémenter la détection de cible ;
- passer aux coordonnées sub-pixel puis aux angles ;
- ajouter la calibration optique.
