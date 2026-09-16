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
   ├── ROI autour de la dernière cible détectée
   ├── décodage JPEG 1/4
   └── recapture si la cible devient nettement floue
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

## Mode continu avec contrôle de netteté ciblé

Le démarrage est interdit sans calibration valide :

```text
GET /continuous/start?interval_ms=1000
GET /continuous/status
GET /continuous/stop
```

Le cycle est :

```text
capture
→ contrôle netteté dans la ROI de la dernière cible connue
   ├── aucune ROI connue → pas de rejet, passage direct au filtre
   ├── cible fortement floue → recapture immédiate, maximum 2 fois
   └── OK
→ filtre
→ détection
→ mise à jour ROI + référence netteté
→ mesure
→ cycle suivant
```

La netteté ne dépend donc plus du décor complet. Sur un mur uniforme, seul le voisinage de la cible influence le score.

La ROI vaut environ quatre fois la taille détectée de la cible, avec un minimum de `64×64 px` dans l'image source. Le JPEG est décodé à `1/4` pour ce contrôle afin de conserver assez de détails quand la cible devient petite.

La référence de netteté est mise à jour uniquement après une détection valide. Une chute du score ROI sous 60 % de cette référence déclenche une recapture. Si la troisième capture reste faible, le pipeline continue quand même pour ne pas se bloquer.

`/continuous/status` expose les temps détaillés :

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
sharpness.roi_active
sharpness.roi_x
sharpness.roi_y
sharpness.roi_width
sharpness.roi_height
```

Cela permet de vérifier que le contrôle travaille bien autour de la cible et de guider les futures optimisations.

## Interface Web ESPHome

La page principale garde les **dernières valeurs valides** de distance, X/Y/Z, angles et qualité même si un cycle courant ne retrouve pas la cible. La ligne `03 Cible actuelle` indique séparément si la dernière détection a réussi.

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

`/api/wsdl` est la référence du contrat HTTP. Version actuelle : **14**.

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Étape actuelle

1. compiler/flasher la version avec netteté ROI ;
2. vérifier que `/continuous/status` expose une ROI cohérente autour de la cible ;
3. provoquer volontairement un flou de la cible ;
4. comparer le taux de cibles trouvées avant/après ;
5. exploiter les timings détaillés ;
6. reprendre ensuite la validation des angles et la future approche haute résolution + ROI.
