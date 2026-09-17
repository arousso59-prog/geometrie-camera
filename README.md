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
CameraViewportController
   ├── SEARCH : plein champ 800×600
   └── PRECISE : ROI native 800×600 dans le repère 2560×1920
   ↓
JpegDiagnostic
   ↓
ImageSharpnessEvaluator
   ↓
JpegFilteredDiagnostic V2
   ↓
JpegArtifactCorrector
   ↓
TargetDetector V5.5
   ├── décodage du motif 7x7
   ├── raffinage des coins
   └── continuité temporelle position/taille
   ↓
conversion coordonnées viewport → repère 2560×1920
   ↓
GeometryMeasurementEngine V3
   ├── distance fusionnée à partir des deux axes
   ├── centre projectif par intersection des diagonales
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

La profondeur V3 conserve :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
```

Quand elles sont proches, `z_mm` utilise leur moyenne harmonique. Lorsque leur écart augmente, le calcul revient progressivement vers la plus petite estimation. La transition commence à 3 % et devient complète à 15 %.

Le centre utilisé pour `x_mm`, `y_mm`, `bearing_yaw_deg` et `bearing_pitch_deg` est calculé à partir de l'intersection des diagonales formées par les quatre coins raffinés. `pose_z_mm` reste un contrôle indépendant issu de l'homographie.

Le moteur de mesure sait déjà redimensionner une calibration réalisée à une autre résolution de même cadrage. En mode tracking, les coordonnées détectées sont converties dans le repère de référence **2560×1920** avant la mesure ; une calibration existante 800×600 reste donc mathématiquement exploitable.

## Réglages caméra

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
GET /api/camera/settings/set?monochrome=1
GET /api/camera/settings/set?monochrome=0
```

`monochrome=1` active l'effet grayscale de l'OV5640 à chaud. Le framebuffer reste en JPEG.

## Tracking haute précision SEARCH / PRECISE

Le tracking est volontairement **désactivé par défaut** pendant la première validation matérielle.

Configuration :

```text
GET /tracking/config
GET /tracking/config/set?enabled=1
GET /tracking/config/set?lost_cycles=3&recenter_threshold_pct=70
GET /tracking/status
GET /api/camera/viewport
```

La première version utilise des dimensions fixes :

```text
repère de référence : 2560×1920
SEARCH              : plein champ 800×600
PRECISE             : crop natif 800×600, sortie 800×600
```

Le PC ne déplace jamais la ROI à chaque cycle. `/continuous/start` démarre seulement l'automate ; l'ESP32 gère ensuite :

```text
SEARCH 800×600 plein champ
      ↓ cible validée
conversion cible vers repère 2560×1920
      ↓
PRECISE ROI native 800×600 centrée sur la cible
      ↓
mesures successives dans cette ROI
      ├── cible proche du bord → recentrage pour le cycle suivant
      └── cible perdue 3 cycles → retour SEARCH
```

Le changement de viewport est effectué **après le calcul du cycle courant**. La capture suivante purge déjà la frame éventuellement pré-acquise avant la reconfiguration, grâce au mécanisme de `JpegDiagnostic`.

Le filtre ne traite donc jamais une image grayscale 2560×1920 complète : il continue à travailler sur une sortie d'environ 800×600. Le gain de précision vient du fait qu'en PRECISE ces 800×600 pixels correspondent directement à une petite zone native du capteur.

`/api/camera/viewport` expose notamment :

```text
mode
reference.width / reference.height
window.x / window.y / window.width / window.height
output.width / output.height
scale.x / scale.y
```

`/continuous/status` contient aussi un bloc `tracking` avec `enabled`, `supported`, `mode`, `target_locked`, `lost_count`, `transition_count` et la ROI courante.

## Détection cible V5.5

Le détecteur utilise le score du motif comme critère de validité et une cohérence temporelle pour stabiliser la sélection. Son historique est remis à zéro à chaque changement de viewport, car SEARCH et PRECISE n'utilisent pas le même repère local.

Dans `/target/preview.bmp` :

```text
cadre plein      = cible validée
cadre pointillé  = meilleur candidat localisé mais rejeté
```

## Mode continu

Le démarrage exige une calibration valide :

```text
GET /continuous/start?interval_ms=1000
GET /continuous/status
GET /continuous/stop
```

Le contrôle de netteté reste local à la cible. Après un changement SEARCH/PRECISE, la ROI et la référence de netteté sont réinitialisées pour éviter de comparer directement les scores de deux niveaux de grossissement différents.

`/continuous/status` expose :

```text
timing.capture_ms
timing.sharpness_ms
timing.filter_ms
timing.detect_ms
timing.compute_ms
timing.processing_ms
timing.orchestration_ms
timing.cycle_ms
```

## API actuelle

```text
GET /api/wsdl
GET /api/runtime/status
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>
GET /api/camera/viewport
GET /tracking/config
GET /tracking/config/set?enabled=<0|1>&lost_cycles=<1..10>&recenter_threshold_pct=<50..90>
GET /tracking/status
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

`/api/wsdl` est la référence du contrat HTTP. Version actuelle : **21**.

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Validation de la première version ROI

1. compiler et flasher avec le tracking laissé désactivé ;
2. vérifier que le mode continu historique fonctionne toujours ;
3. appeler `/tracking/status` et vérifier `supported=true` ;
4. activer `/tracking/config/set?enabled=1` ;
5. démarrer `/continuous/start?interval_ms=1000` ;
6. vérifier dans `/continuous/status` le passage `search` → `precise` après détection ;
7. vérifier que la capture reste déclarée `800x600` en mode PRECISE et que la cible apparaît nettement plus grande ;
8. contrôler `capture_ms`, `filter_ms`, `detect_ms` et `cycle_ms` pour mesurer le coût réel du crop natif ;
9. déplacer doucement la cible vers le bord et vérifier le recentrage ;
10. masquer la cible pendant trois cycles et vérifier le retour automatique en SEARCH.

Cette première version conserve volontairement les timings capteur complets en PRECISE. Une fois le crop natif validé, les timings de lecture OV5640 pourront être resserrés pour réduire aussi le temps d'acquisition, sans modifier l'API.
