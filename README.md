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
   ├── score sur les 20 % de contours les plus forts
   └── recapture seulement si flou important
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

## Mode continu avec contrôle de netteté ciblé V2

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
   ├── cible manifestement floue → recapture immédiate, maximum 2 fois
   └── OK
→ filtre
→ détection
→ mise à jour ROI + référence netteté
→ mesure
→ cycle suivant
```

La netteté ne dépend donc plus du décor complet. Sur un mur uniforme, seul le voisinage de la cible influence le score.

La ROI vaut maintenant environ **2,5 fois la taille détectée de la cible**, avec un minimum de `48×48 px` dans l'image source. Le JPEG est décodé à `1/4` pour conserver assez de détails quand la cible devient petite.

Le score ne moyenne plus tous les pixels de la ROI : il utilise les **20 % de réponses Laplaciennes les plus fortes**, qui correspondent principalement aux transitions noir/blanc du motif. Cela réduit fortement l'influence du fond uniforme.

La référence de netteté est mise à jour uniquement après une détection valide. Une nouvelle valeur est d'abord limitée à ±15 % de la référence précédente, puis intégrée lentement (`7/8` ancienne référence + `1/8` nouvelle valeur bornée). Une capture n'est recapturée que si son score tombe sous **45 %** de cette référence. Si la troisième capture reste faible, le pipeline continue quand même pour ne pas se bloquer.

Les essais V1 ont montré que le mini-décodage de netteté 1/4 coûte encore environ `315–320 ms`. Cette V2 vise d'abord à vérifier que les recaptures deviennent réellement utiles ; ensuite, si le principe est validé, le double décodage JPEG sera un axe prioritaire d'optimisation.

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

`/api/wsdl` est la référence du contrat HTTP. Version actuelle : **15**.

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Étape actuelle

1. compiler/flasher la netteté ROI V2 ;
2. observer `score_x100`, `reference_x100` et `blur_retry_count` avec cible immobile ;
3. provoquer volontairement quelques flous rapides ;
4. vérifier que les recaptures sont beaucoup moins fréquentes sur les images normalement exploitables ;
5. comparer le taux de cibles trouvées ;
6. si le filtre est validé, supprimer à terme le coût du double décodage JPEG ;
7. reprendre ensuite la validation des angles et la future approche haute résolution + ROI.
