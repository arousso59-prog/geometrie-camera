# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV3660.

## Architecture

Le projet sépare maintenant strictement les responsabilités :

- `geometrie-camera.yaml` : configuration ESPHome et matérielle ;
- `GeometrieCameraApp` : orchestration uniquement ;
- `ImageProvider` : interface de source d'image ;
- `PlaceholderImageProvider` : image bouchon actuelle ;
- `CameraManager` : cycle d'acquisition et métadonnées ;
- `MeasurementManager` : chaîne de détection et de mesure ;
- `TargetDetector` : détection de cible ;
- `GeometryMeasurementEngine` : calcul mathématique des angles ;
- `CameraApiHandler` : interface HTTP uniquement.

Les règles complètes de développement, découpage des classes et préparation des tests unitaires sont décrites dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Étape actuelle — V0 API bouchon

La caméra physique n'est pas encore activée tant que le brochage exact du PCB ESP32-S3 + OV3660 n'est pas validé.

L'API HTTP fonctionne avec une image JPEG de test monochrome 160 × 120, moitié noire / moitié blanche. Cela permet de développer et tester la communication PC sans attendre la caméra réelle.

### Routes HTTP

#### `GET /api/status`

Retourne l'état du module, la source d'image utilisée et les métadonnées de la dernière image.

Exemple :

```json
{
  "api_version": 1,
  "status": "ok",
  "mode": "placeholder",
  "camera_ready": true,
  "physical_camera_ready": false,
  "capture_count": 0,
  "last_capture_ms": 1234,
  "image": {
    "url": "/image.jpg",
    "format": "jpeg",
    "width": 160,
    "height": 120,
    "size_bytes": 269
  }
}
```

#### `GET /api/capture`

Déclenche une capture logique. En V0, le `PlaceholderImageProvider` fournit le JPEG de test. Le compteur est incrémenté uniquement lors d'une demande explicite de capture.

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

Retourne directement la dernière image fournie par `CameraManager`.

Aujourd'hui il s'agit du JPEG bouchon. Lorsque l'OV3660 sera activée, cette même route retournera l'image réelle sans changer le contrat de l'API PC.

#### `GET /api/measure`

Retourne la dernière mesure connue. Tant que la détection de cible n'est pas développée, `valid` reste à `false`.

```json
{
  "valid": false,
  "timestamp_ms": 0,
  "yaw_deg": 0.0,
  "pitch_deg": 0.0,
  "roll_deg": 0.0,
  "quality": 0.0
}
```

## Arborescence principale

```text
geometrie-camera/
├── ARCHITECTURE.md
├── geometrie-camera.yaml
├── secrets.yaml.example
└── components/
    └── geometrie_camera_app/
        ├── __init__.py
        ├── types.h / types.cpp
        ├── geometrie_camera_app.h / .cpp
        ├── image_provider.h / .cpp
        ├── placeholder_image_provider.h / .cpp
        ├── placeholder_image.h / .cpp
        ├── camera_manager.h / .cpp
        ├── camera_api.h / .cpp
        ├── measurement_manager.h / .cpp
        ├── target_detector.h / .cpp
        └── geometry_measurement.h / .cpp
```

## Brochage caméra

La carte est configurée comme ESP32-S3 et le capteur prévu est l'OV3660. Le brochage parallèle de la caméra dépend toutefois du PCB exact.

Le bloc `esp32_camera` reste volontairement désactivé dans le YAML jusqu'à validation de D0..D7, XCLK, PCLK, VSYNC, HREF et SCCB.

## Principe pour la suite

Le contrat HTTP doit rester stable :

```text
PC -> GET /api/capture
ESP -> acquisition via ImageProvider
PC <- JSON avec métadonnées
PC -> GET /image.jpg
ESP -> dernière image disponible
PC -> GET /api/measure
ESP -> dernière mesure calculée
```

Le futur `Ov3660ImageProvider` remplacera le provider bouchon sans imposer de refonte de `CameraManager`, de l'API ou de la console PC.

## Étapes suivantes

- compiler et valider cette nouvelle architecture sur l'ESP32-S3 ;
- valider le brochage réel de la carte reçue ;
- ajouter `Ov3660ImageProvider` ;
- fixer exposition / gain / balance des blancs ;
- implémenter la détection de cible ;
- ajouter les premiers tests unitaires du calcul géométrique et du gestionnaire de capture ;
- passer aux coordonnées sub-pixel puis aux angles ;
- ajouter la calibration optique.
