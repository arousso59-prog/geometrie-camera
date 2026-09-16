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
GeometryMeasurementEngine V1
   ↓
distance + X/Y/Z + angles de visée + yaw/pitch/roll cible
```

La détection V5.4 combine localisation rapide, raffinement pleine résolution des quatre coins et lecture projective du code 7×7. Le candidat brut reste toujours testé en secours.

Le filtre V2 est désormais la base de travail. À 1600×1200, un essai représentatif donne :

```text
decode JPEG   ≈ 1476 ms
correction    ≈ 553 ms
total filtre  ≈ 2036 ms
```

Les essais répétés sont du même ordre de grandeur et la détection n'a pas montré de régression par rapport à la version précédente.

## Mesure V1 : distance et angles

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

et appeler par exemple, pour une cible située à 2000 mm :

```text
GET /measurement/calibrate?distance_mm=2000&target_size_mm=50
```

Le firmware estime alors `fx` et `fy` à partir de la taille réelle de 50 mm et de la taille détectée en pixels. La résolution utilisée pendant cette calibration est mémorisée.

Pour les autres résolutions de même cadrage optique, les paramètres intrinsèques sont redimensionnés automatiquement. Par exemple une calibration en 1600×1200 peut servir en 800×600 pour les premiers essais.

### Calcul d'une mesure

Après une nouvelle séquence :

```text
capture
→ filtre
→ /target/detect
→ /measurement/compute
```

la réponse de mesure contient notamment :

```text
distance_mm
x_mm
y_mm
z_mm
bearing_yaw_deg
bearing_pitch_deg
target_yaw_deg
target_pitch_deg
target_roll_deg
quality
```

Le repère caméra est :

```text
X positif = droite
Y positif = bas
Z positif = avant
```

`distance_mm` est la distance euclidienne caméra → centre de cible, alors que `z_mm` représente la profondeur suivant l'axe optique.

Les angles `bearing_*` décrivent la direction du centre de la cible. Les angles `target_*` décrivent l'orientation du plan de la cible, obtenue par décomposition de l'homographie des quatre coins.

Cette V1 ne compense pas encore précisément la distorsion radiale de l'objectif. Elle doit d'abord permettre de mesurer l'erreur réelle et la répétabilité avant d'ajouter une calibration optique plus complète.

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
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>
GET /measurement/compute
GET /measurement/status
```

`/api/wsdl` est la référence du contrat HTTP compilé. Version actuelle : **10**.

## Étape actuelle

1. compiler/flasher la V1 de mesure ;
2. calibrer avec la cible 50 mm à une distance connue ;
3. vérifier la distance à plusieurs distances réelles ;
4. vérifier les angles de visée en déplaçant la cible horizontalement et verticalement ;
5. vérifier yaw/pitch/roll en inclinant la cible ;
6. mesurer la répétabilité ;
7. corriger ensuite la calibration optique/distorsion si nécessaire ;
8. passer à l'acquisition continue ;
9. revenir ensuite sur ROI et optimisation de performance.
