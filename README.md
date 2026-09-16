# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- capteur confirmé par PID : OV5640 (`0x5640`) ;
- PSRAM : 8 Mo octal ;
- résolution maximale exposée par ESPHome 2026.7.3 : 2560×1920 ;
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

## Pipeline actuel

```text
OV5640 JPEG
   ↓
frame fraîche
   ↓
JpegFilteredDiagnostic V2
   ↓
JpegArtifactCorrector sparse / lookup masks
   ↓
buffer grayscale corrigé
   ↓
TargetDetector V5.4
   ├── TargetCandidateFinder V5.2
   ├── TargetCornerRefiner V5.4
   └── TargetCodeDecoder V5.4
   ↓
TargetObservation + 4 coins
   ↓
GeometryMeasurementEngine V2
   ├── distance robuste par taille apparente
   ├── X/Y/Z + angles de visée
   └── pose homographique validée séparément
```

La détection V5.4 combine localisation rapide, raffinement pleine résolution des quatre coins et lecture projective du code 7×7. Le candidat brut reste toujours testé en secours.

Le filtre V2 est désormais la base de travail. À 1600×1200, un essai représentatif donne :

```text
decode JPEG   ≈ 1476 ms
correction    ≈ 553 ms
total filtre  ≈ 2036 ms
```

Les essais répétés sont du même ordre de grandeur et la détection n'a pas montré de régression par rapport à la version précédente.

## Mesure V2 : distance robuste et angles

La taille physique de cible par défaut est :

```text
50 mm
```

La résolution de l'image et la taille apparente de la cible ne suffisent pas à connaître une distance absolue : la focale réelle du module caméra doit être calibrée. Le champ de vision commercial annoncé n'est pas utilisé comme référence de précision.

### Calibration initiale

Placer la cible :

- approximativement de face ;
- proche du centre de l'image ;
- à une distance caméra → cible mesurée aussi précisément que possible.

Puis réaliser normalement :

```text
capture
→ filtre
→ /target/detect
```

et appeler par exemple, pour une cible située à 1000 mm :

```text
GET /measurement/calibrate?distance_mm=1000&target_size_mm=50
```

Le firmware estime `fx` et `fy` à partir de la taille réelle de 50 mm et de la taille détectée en pixels. La résolution utilisée pendant cette calibration est mémorisée.

Une calibration valide est ensuite **verrouillée**. Un nouvel appel de calibration sans demande explicite renvoie :

```text
calibration_locked
```

Pour remplacer volontairement la calibration :

```text
GET /measurement/calibrate?distance_mm=1000&target_size_mm=50&force=1
```

Modifier `target_size_mm` par `/measurement/config/set` invalide aussi la calibration précédente.

Pour les autres résolutions de même cadrage optique, les paramètres intrinsèques sont redimensionnés automatiquement.

### Calcul d'une mesure

Après une nouvelle séquence :

```text
capture
→ filtre
→ /target/detect
→ /measurement/compute
```

la distance principale n'est plus issue de la décomposition homographique. Elle repose sur la taille apparente des quatre coins :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
z_mm             = min(z_from_width_mm, z_from_height_mm)
```

Puis le centre de cible permet de calculer `x_mm`, `y_mm` et la distance euclidienne.

La réponse contient notamment :

```text
distance_mm
x_mm
y_mm
z_mm
z_from_width_mm
z_from_height_mm
bearing_yaw_deg
bearing_pitch_deg
pose_valid
target_yaw_deg
target_pitch_deg
target_roll_deg
pose_z_mm
pose_scale_error_pct
quality
```

Le repère caméra est :

```text
X positif = droite
Y positif = bas
Z positif = avant
```

`distance_mm` est la distance euclidienne caméra → centre de cible, alors que `z_mm` représente la profondeur suivant l'axe optique.

Les angles `bearing_*` décrivent la direction du centre de la cible et sont calculés indépendamment de la pose du plan.

Les angles `target_*` décrivent l'orientation du plan de la cible. L'homographie reste utilisée pour cette orientation, mais elle doit être cohérente avec la distance robuste : si `pose_z_mm` diffère de plus de 25 % de `z_mm`, `pose_valid=false` et les angles de pose ne sont pas considérés fiables.

Le roulis accepté est normalisé modulo 180° afin qu'une cible presque droite ne soit pas affichée autour de ±180° uniquement à cause de l'orientation logique du code.

Cette version ne compense pas encore précisément la distorsion radiale de l'objectif. Elle doit d'abord permettre de mesurer l'erreur réelle et la répétabilité avant d'ajouter une calibration optique plus complète.

## Architecture

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

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
GET /target/preview.bmp

GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>&force=<0|1>
GET /measurement/compute
GET /measurement/status
```

`/api/wsdl` est la référence du contrat HTTP compilé. Version actuelle : **11**.

## Étape actuelle

1. compiler/flasher la V2 de mesure ;
2. calibrer une seule fois avec la cible 50 mm à une distance connue ;
3. vérifier que la calibration est verrouillée ;
4. déplacer la cible à plusieurs distances sans recalibrer ;
5. comparer `z_from_width_mm`, `z_from_height_mm` et `z_mm` à la distance réelle ;
6. vérifier les angles de visée en déplaçant la cible horizontalement et verticalement ;
7. observer `pose_valid` avant de retravailler yaw/pitch/roll ;
8. mesurer la répétabilité ;
9. corriger ensuite calibration optique/distorsion si nécessaire ;
10. passer à l'acquisition continue puis revenir sur ROI et optimisation de performance.
