# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- OV5640 confirmé (`0x5640`) ;
- PSRAM 8 Mo octal ;
- résolution exposée jusqu'à 2560×1920 ;
- XCLK actuel : 8 MHz.

## Pipeline actuel

```text
OV5640 JPEG
   ↓
JpegDiagnostic
   ↓
ImageSharpnessEvaluator (mode continu)
   ├── décodage JPEG 1/8
   └── recapture si flou important
   ↓
JpegFilteredDiagnostic V2
   ↓
JpegArtifactCorrector
   ↓
TargetDetector V5.4
   ↓
GeometryMeasurementEngine V2
   ├── distance robuste par taille apparente
   ├── X/Y/Z + bearing
   └── pose homographique validée séparément
```

## Calibration et distance

La cible par défaut mesure 50 mm. La focale est calibrée à partir d'une distance connue :

```text
capture
→ filtre
→ /target/detect
→ /measurement/calibrate?distance_mm=1000&target_size_mm=50
```

Une calibration valide est verrouillée. Pour la remplacer volontairement :

```text
GET /measurement/calibrate?distance_mm=1000&target_size_mm=50&force=1
```

La profondeur principale repose sur :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
z_mm             = min(z_from_width_mm, z_from_height_mm)
```

Puis le centre de cible donne `x_mm`, `y_mm`, `distance_mm`, `bearing_yaw_deg` et `bearing_pitch_deg`.

## Résolution de travail

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
```

Le mode continu utilise la résolution caméra active. `800x600` reste pratique pour les essais rapides ; une évolution haute résolution + ROI est prévue pour augmenter la précision sans traiter toute l'image.

## Mode continu avec contrôle de netteté

Le démarrage est interdit sans calibration valide :

```text
GET /continuous/start?interval_ms=1000
GET /continuous/status
GET /continuous/stop
```

Le cycle est :

```text
capture
→ contrôle rapide de netteté
   ├── flou marqué → recapture immédiate, maximum 2 fois
   └── OK
→ filtre
→ détection
→ mesure
→ cycle suivant
```

Le contrôle de netteté travaille sur un JPEG réduit à 1/8. Sa référence est relative à la session ; une chute sous 60 % de la référence déclenche une recapture. Si la troisième capture reste faible, le pipeline continue quand même afin de ne pas se bloquer.

`/continuous/status` expose maintenant les temps détaillés :

```text
timing.capture_ms
timing.sharpness_ms
timing.filter_ms
timing.detect_ms
timing.compute_ms
timing.cycle_ms
```

et les informations de netteté :

```text
sharpness.score_x100
sharpness.reference_x100
sharpness.ok
sharpness.capture_retries
sharpness.blur_retry_count
```

Cela permet de décider les futures optimisations à partir de mesures réelles.

## Interface Web ESPHome

La page principale garde désormais les **dernières valeurs valides** de distance, X/Y/Z, angles et qualité même si un cycle courant ne retrouve pas la cible. La ligne `03 Cible actuelle` indique séparément si la dernière détection a réussi.

Les timings capture/netteté/filtre/détection/calcul et les compteurs de recapture sont affichés sous les mesures principales.

## API actuelle

```text
GET /api/wsdl
GET /api/runtime/status
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>&resolution=<optionnel>
GET /diagnostic-jpeg/capture?resolution=<optionnel>
GET /diagnostic-jpeg/status
GET /diagnostic-jpeg/image.jpg
GET /diagnostic-jpeg/filter
GET /diagnostic-jpeg/filter-status
GET /diagnostic-jpeg/filtered.bmp
GET /target/detect
GET /target/status
GET /target/preview.bmp
GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>&force=<0|1>
GET /measurement/compute
GET /measurement/status
GET /continuous/start?interval_ms=<optionnel>
GET /continuous/stop
GET /continuous/status
```

`/api/wsdl` est la référence du contrat HTTP. Version actuelle : **13**.

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Étape actuelle

1. compiler/flasher la version avec contrôle de netteté ;
2. observer les scores de netteté sur des captures normales ;
3. provoquer volontairement un flou pour vérifier les recaptures ;
4. comparer le taux de cibles trouvées avant/après ;
5. utiliser les timings détaillés pour prioriser les optimisations ;
6. reprendre ensuite la validation des angles et la future approche haute résolution + ROI.
