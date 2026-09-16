# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- capteur confirmé par PID : OV5640 (`0x5640`) ;
- PSRAM : 8 Mo octal ;
- résolution maximale exposée par ESPHome : 2560×1920 ;
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

Le filtre V2 est la base de travail. À 1600×1200, un essai représentatif donne environ :

```text
decode JPEG   ≈ 1476 ms
correction    ≈ 553 ms
total filtre  ≈ 2036 ms
```

## Mesure V2 : distance robuste et angles

La taille physique de cible par défaut est :

```text
50 mm
```

La focale réelle du module caméra doit être calibrée à partir d'une distance connue.

### Calibration initiale

Placer la cible approximativement de face, proche du centre de l'image et à une distance connue, puis :

```text
capture
→ filtre
→ /target/detect
→ /measurement/calibrate?distance_mm=1000&target_size_mm=50
```

Une calibration valide est ensuite **verrouillée**. Un nouvel appel sans `force=1` renvoie `calibration_locked`.

Pour remplacer volontairement la calibration :

```text
GET /measurement/calibrate?distance_mm=1000&target_size_mm=50&force=1
```

### Distance robuste

La distance principale repose sur la taille apparente :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
z_mm             = min(z_from_width_mm, z_from_height_mm)
```

Puis le centre de cible permet de calculer `x_mm`, `y_mm`, `distance_mm`, `bearing_yaw_deg` et `bearing_pitch_deg`.

La pose homographique reste séparée. Si sa profondeur n'est pas cohérente avec la distance robuste, `pose_valid=false` et les angles `target_yaw/pitch/roll` sont rejetés.

## Résolution de travail

La résolution active est maintenant exposée avec les autres réglages caméra :

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
```

Le contrôleur de résolution reste propriétaire de cette configuration. Le mode continu utilise simplement la résolution active.

Pour les essais rapides, `800x600` est pratique. Plus tard, une résolution plus élevée combinée à une ROI permettra d'améliorer la précision sans traiter toute l'image.

## Mode continu V1

Le firmware peut maintenant automatiser :

```text
capture
→ filtre
→ détection
→ mesure
→ attente éventuelle
→ cycle suivant
```

Le démarrage est **strictement interdit tant qu'aucune calibration valide n'existe**.

```text
GET /continuous/start?interval_ms=1000
GET /continuous/status
GET /continuous/stop
```

L'intervalle admissible est actuellement `200..10000 ms`. Si un cycle prend plus longtemps que l'intervalle demandé, aucun travail n'est empilé : le cycle suivant repart dès que le précédent est terminé.

Si la calibration est invalidée pendant le fonctionnement, le mode continu s'arrête avec `calibration_lost`.

L'interface Web ESPHome affiche maintenant en priorité :

```text
état acquisition
résolution de travail
cible actuelle
état pose
distance
profondeur Z
X / Y
angles de visée horizontal / vertical
qualité
temps de cycle
compteurs de cycles et mesures
```

Les anciennes commandes manuelles `fx/fy/cx/cy` ont été remplacées par les **vraies valeurs de calibration en lecture seule**.

## Architecture

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

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

`/api/wsdl` est la référence du contrat HTTP compilé. Version actuelle : **12**.

## Étape actuelle

1. compiler/flasher le mode continu V1 ;
2. choisir la résolution de travail, par exemple `800x600` ;
3. calibrer une fois à une distance connue ;
4. démarrer le mode continu ;
5. observer plusieurs dizaines de cycles cible immobile pour quantifier la répétabilité ;
6. valider ensuite les angles de visée par déplacements connus ;
7. retravailler la pose yaw/pitch/roll ;
8. tester 1600×1200 puis haute résolution + ROI ;
9. intégrer la console PC sur les mêmes API.
