# Architecture du projet `geometrie-camera`

## Règles de développement

1. **Les fichiers `.h` sont déclaratifs uniquement.** Les implémentations et données significatives restent dans les `.cpp`.
2. **Une classe = une responsabilité principale.** `GeometrieCameraApp` reste un orchestrateur.
3. **Avant toute nouvelle fonction, revoir l'architecture** et choisir explicitement la classe responsable.
4. **Penser chaque évolution avec les tests** : logique pure séparée du matériel, dépendances injectées.
5. **Toute évolution de l'API HTTP met à jour `GET /api/wsdl` dans le même changement.**
6. **Les gros buffers image/vision sont budgétés et placés en PSRAM**, réutilisés entre appels et jamais alloués en gros temporaires sur la pile HTTP.

## Architecture actuelle

```text
GeometrieCameraApp
├── CameraResolutionController
├── CameraSettingsController
│   └── CameraSettingsApiHandler
├── JpegDiagnostic
│   └── JpegDiagnosticApiHandler
├── ImageSharpnessEvaluator
├── JpegFilteredDiagnostic
│   ├── JpegArtifactCorrector
│   └── JpegFilteredDiagnosticApiHandler
├── TargetDetector
│   ├── TargetCandidateFinder
│   ├── TargetCornerRefiner
│   └── TargetCodeDecoder
├── TargetDetectionService
│   └── TargetDetectionApiHandler
├── TargetDetectionPreview
├── MeasurementManager
│   └── GeometryMeasurementEngine
├── MeasurementApiHandler
├── ContinuousMeasurementController
│   └── ContinuousMeasurementApiHandler
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
└── ApiWsdlHandler
```

Le `TargetDetector` appartient directement à l'application. `MeasurementManager` ne contient pas de second détecteur : la mesure réutilise le `TargetObservation` validé par `TargetDetectionService`.

`ContinuousMeasurementController` ne contient aucune logique de vision ou de géométrie. Il orchestre les briques existantes et porte seulement la politique temporelle : choix de la ROI de netteté à partir de la dernière cible valide, recapture et cadence.

## Chaîne image et mesure

```text
OV5640 JPEG
   ↓
JpegDiagnostic
   ↓
ImageSharpnessEvaluator (mode continu uniquement)
   ├── ROI autour de la dernière cible valide
   ├── TJpgDec en 1/4
   ├── score de netteté par Laplacien dans la ROI
   └── recapture possible avant traitement lourd
   ↓
JpegFilteredDiagnostic V2
   ├── TJpgDec pleine résolution
   ├── RGB -> luminance 8 bits
   └── masque vert brut
   ↓
JpegArtifactCorrector
   ↓
GrayFrameView corrigé
   ↓
TargetDetector V5.4
   ↓
TargetObservation
   ↓
MeasurementManager
   ↓
GeometryMeasurementEngine V2
   ├── distance robuste par taille apparente
   ├── X/Y/Z + angles de visée
   └── pose homographique validée séparément
```

En mode continu :

```text
REQUEST_CAPTURE
      ↓
WAIT_CAPTURE
      ↓
SHARPNESS
  ├── pas de ROI cible connue → pas de rejet, passage au filtre
  ├── ROI floue + essais restants → recapture immédiate
  └── ROI acceptable / essais épuisés
      ↓
FILTER
      ↓
DETECT
      ↓ cible trouvée
mise à jour ROI + référence netteté
      ↓
COMPUTE
      ↓
WAIT_INTERVAL
      ↺
```

Aucun cycle n'est empilé. Si le traitement dépasse l'intervalle demandé, le cycle suivant repart dès que le précédent est terminé.

## Responsabilités principales

### `JpegDiagnostic`

Acquisition JPEG native. Une demande purge la frame pré-acquise par ESPHome puis demande une frame fraîche. Le JPEG est copié en PSRAM car le framebuffer caméra est éphémère.

### `ImageSharpnessEvaluator`

Responsabilité unique : calculer un **score relatif de netteté dans une région demandée** du dernier JPEG.

Méthode actuelle :

- décode le JPEG avec TJpgDec à l'échelle `1/4` ;
- 800×600 devient 200×150 ;
- conserve un petit buffer grayscale persistant ;
- projette la ROI source dans cette image réduite ;
- calcule la moyenne de la valeur absolue du Laplacien uniquement dans la ROI ;
- ne connaît ni la cible ni le détecteur et ne décide pas seul du rejet d'une image.

Le passage de 1/8 à 1/4 est volontaire : à environ 2 m en 800×600, une cible de ~15 px ne représentait qu'environ 2 px en 1/8, contre ~4 px en 1/4.

**Frontière de test :** même ROI nette/floue -> score relatif ; ROI proche des bords ; variation de résolution ; erreur de décodage -> évaluation invalide sans bloquer le pipeline principal.

### `JpegFilteredDiagnostic`

Transforme le JPEG en grayscale corrigible, construit le masque vert et appelle le correcteur. Le workspace TJpgDec de 4 ko est persistant.

Référence 1600×1200 validée :

```text
decode_ms      ≈ 1476 ms
correction_ms  ≈ 553 ms
total_ms       ≈ 2036 ms
```

À 800×600, les essais continus observés sont autour de 400 ms de filtre total, dont environ 350 ms de décodage et 50 ms de correction.

### `TargetDetector`

Orchestre localisation, raffinement des coins et validation du code 7×7. Les facteurs de dilatation du décodeur servent à lire le code et ne modifient pas la géométrie physique mémorisée dans `TargetObservation`.

### `MeasurementManager` / `GeometryMeasurementEngine`

`MeasurementManager` conserve la dernière mesure valide et son compteur. Il ne détecte pas la cible.

La distance V2 repose sur :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
z_mm             = min(z_from_width_mm, z_from_height_mm)
```

Puis :

```text
nx = (center_x - cx) / fx
ny = (center_y - cy) / fy
x_mm = nx × z_mm
y_mm = ny × z_mm
distance_mm = sqrt(x_mm² + y_mm² + z_mm²)
```

L'homographie sert uniquement à l'orientation du plan. `pose_valid` n'est vrai que si sa profondeur reste cohérente avec la distance robuste.

### Calibration

La calibration à distance connue fournit `fx/fy/cx/cy` et mémorise la résolution de référence. Une calibration valide est verrouillée ; `force=1` est nécessaire pour la remplacer. Modifier `target_size_mm` invalide la calibration.

Le mode continu est interdit sans calibration valide et s'arrête si elle disparaît.

### `ContinuousMeasurementController`

Dépendances :

```text
JpegDiagnostic
ImageSharpnessEvaluator
JpegFilteredDiagnostic
TargetDetectionService
MeasurementManager
```

Politique de netteté ROI :

- la ROI est centrée sur la **dernière cible réellement détectée** ;
- sa taille est environ `4 × max(width_px, height_px)` avec un minimum de `64×64 px` dans l'image source ;
- la calibration/détection effectuée avant le démarrage peut fournir la ROI initiale ;
- sans ROI connue, le contrôleur ne rejette jamais une image sur un score global du décor ;
- la référence de netteté n'est mise à jour qu'après une nouvelle détection valide ;
- une image dont le score ROI tombe sous 60 % de la référence est considérée fortement dégradée ;
- au maximum **2 recaptures immédiates** sont effectuées par cycle ;
- si la troisième image reste faible, le pipeline continue malgré tout pour éviter un blocage ;
- en cas d'échec du mini-décodage de netteté, le filtre/détecteur principal continue.

Le contrôleur chronomètre le dernier cycle par poste :

```text
capture_ms       somme des captures du cycle, recaptures incluses
sharpness_ms     somme des contrôles de netteté ROI
filter_ms
detect_ms
compute_ms
cycle_ms
```

Il expose également :

```text
sharpness.score_x100
sharpness.reference_x100
sharpness.ok
sharpness.capture_retries
sharpness.blur_retry_count
sharpness.roi_active
sharpness.roi_x / roi_y
sharpness.roi_width / roi_height
```

**Frontières de test :** calibration absente -> démarrage refusé ; première image sans ROI -> pas de rejet ; cible valide -> ROI mémorisée ; flou dans ROI -> recapture ; décor uniforme hors ROI sans influence ; maximum deux recaptures ; déplacement cible -> nouvelle ROI après détection ; cible absente -> cycle suivant ; perte calibration -> arrêt.

### `CameraResolutionController` / `CameraSettingsApiHandler`

`CameraResolutionController` reste propriétaire de la résolution. `CameraSettingsApiHandler` l'agrège simplement avec les réglages caméra :

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
```

### Interface Web ESPHome

Les valeurs principales (`distance`, `Z`, `X/Y`, angles de visée, qualité) affichent **la dernière mesure valide** et ne repassent plus à `N/A` lorsqu'un cycle ne détecte pas la cible. L'état `03 Cible actuelle` indique séparément le résultat du cycle courant.

Les timings capture/netteté/filtre/détection/calcul et les compteurs de recapture sont affichés afin de guider les optimisations futures.

### `ApiWsdlHandler`

`GET /api/wsdl` est la référence du contrat HTTP. Version actuelle : **14** depuis le passage du contrôle de netteté à une ROI suivie sur la cible et l'exposition de cette ROI dans `/continuous/status`.

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

## Feuille de route immédiate

1. compiler/flasher la netteté ROI ;
2. vérifier dans `/continuous/status` que `sharpness.roi_active=true` et que la ROI encadre bien la dernière cible ;
3. provoquer volontairement un flou de la cible sans changer le décor et observer les recaptures ;
4. vérifier que le mur uniforme hors ROI n'influence plus le score ;
5. comparer le taux de cibles trouvées avant/après ;
6. utiliser les timings détaillés pour prioriser les optimisations ;
7. reprendre ensuite la validation des angles et la future approche haute résolution + ROI.
