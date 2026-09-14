# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV3660.

## Architecture

Le projet sépare strictement les responsabilités :

- `geometrie-camera.yaml` : configuration ESPHome et matérielle ;
- `GeometrieCameraApp` : orchestration uniquement ;
- `ImageProvider` : interface de source d'image ;
- `PlaceholderImageProvider` : image bouchon de l'API principale ;
- `CameraManager` : cycle d'acquisition et métadonnées ;
- `MeasurementManager` : chaîne de détection et de mesure ;
- `TargetDetector` : détection de cible ;
- `GeometryMeasurementEngine` : calcul mathématique des angles ;
- `CameraApiHandler` : interface HTTP principale ;
- `GrayscaleDiagnostic` + `GrayscaleDiagnosticApiHandler` : diagnostic temporaire de la caméra brute.

Les règles de développement et la préparation des tests unitaires sont décrites dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Matériel validé

Carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8.

Capteur : OV3660.

Brochage caméra validé :

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

La capture JPEG réelle a été validée jusqu'à 1600x1200. Un artefact régulier sous forme de petits traits a cependant été observé et reste présent avec différentes résolutions, XCLK 10/20 MHz, PSRAM/DRAM, mire interne et inversion de polarité PCLK.

## Étape actuelle — diagnostic sans compression JPEG

Le firmware est temporairement configuré en :

```yaml
pixel_format: GRAYSCALE
resolution: 640x480
jpeg_quality: 0
frame_buffer_location: PSRAM
idle_framerate: 0 fps
```

Avec `jpeg_quality: 0`, ESPHome conserve la frame GRAYSCALE brute et ne la convertit pas en JPEG.

`GrayscaleDiagnostic` copie cette frame dans un BMP 8 bits **non compressé** uniquement pour permettre son affichage dans un navigateur. Les valeurs de pixels ne sont pas recompressées.

### Procédure de test

1. Demander une acquisition :

```text
GET http://<IP>/diagnostic/capture
```

Réponse attendue :

```json
{
  "accepted": true,
  "status": "capture_requested"
}
```

2. Attendre environ une seconde puis contrôler :

```text
GET http://<IP>/diagnostic/status
```

Une capture valide doit donner `ready: true` avec une taille de 640x480.

3. Afficher la frame brute :

```text
GET http://<IP>/diagnostic/raw.bmp
```

Le navigateur affiche alors un BMP niveaux de gris généré directement à partir de la frame brute OV3660.

### Interprétation

- si les artefacts sont encore visibles dans `/diagnostic/raw.bmp`, ils sont présents **avant toute compression JPEG** : bus caméra, acquisition parallèle, capteur ou driver deviennent les pistes prioritaires ;
- si l'image brute est propre, le problème se situe dans la chaîne JPEG utilisée lors des tests précédents.

## API principale

L'API principale reste actuellement branchée sur `PlaceholderImageProvider` afin de ne pas mélanger diagnostic matériel et architecture finale :

```text
GET /api/status
GET /api/capture
GET /api/measure
GET /image.jpg
```

Le futur `Ov3660ImageProvider` remplacera le bouchon une fois la chaîne caméra validée.

## Étapes suivantes

- exécuter le test GRAYSCALE sans JPEG ;
- selon le résultat, poursuivre le diagnostic bus/PCLK/driver ou chaîne JPEG ;
- revenir ensuite à la résolution de travail validée ;
- créer `Ov3660ImageProvider` ;
- fixer exposition / gain / balance des blancs ;
- implémenter la détection de cible ;
- ajouter les tests unitaires du calcul et des gestionnaires ;
- passer aux coordonnées sub-pixel et à la calibration optique.
