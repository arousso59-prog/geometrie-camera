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

`ContinuousMeasurementController` ne contient aucune logique de vision ou de géométrie. Il orchestre uniquement les briques existantes dans le temps.

## Chaîne image et mesure

```text
OV5640 JPEG
   ↓
JpegDiagnostic
   ↓
ImageSharpnessEvaluator (mode continu uniquement)
   ├── TJpgDec en 1/8
   ├── score de netteté par Laplacien
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
  ├── flou + essais restants → recapture immédiate
  └── acceptable / essais épuisés
      ↓
FILTER
      ↓
DETECT
      ↓ cible trouvée
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

Responsabilité unique : donner rapidement un **score relatif de netteté** du dernier JPEG.

Méthode actuelle :

- décode le JPEG avec TJpgDec à l'échelle `1/8` ;
- 800×600 devient 100×75 ;
- conserve un petit buffer grayscale persistant ;
- calcule la moyenne de la valeur absolue du Laplacien, exposée en `score_x100` ;
- ne décide pas seul si une image doit être rejetée : cette politique appartient au contrôleur continu.

Le contrôle de netteté n'est volontairement pas placé dans `JpegDiagnostic`, qui reste une classe d'acquisition pure.

**Frontière de test :** JPEG net/flou connu -> score relatif ; variation de résolution ; erreur de décodage -> évaluation invalide.

### `JpegFilteredDiagnostic`

Transforme le JPEG en grayscale corrigible, construit le masque vert et appelle le correcteur. Le workspace TJpgDec de 4 ko est persistant.

Référence 1600×1200 validée avant cette évolution :

```text
decode_ms      ≈ 1476 ms
correction_ms  ≈ 553 ms
total_ms       ≈ 2036 ms
```

À 800×600, les essais continus observés sont autour de 395 ms de filtre total, dont environ 347 ms de décodage et 47 ms de correction.

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

Politique de netteté V1 :

- la première image valide d'une session initialise la référence ;
- la référence évolue progressivement avec les images acceptées ;
- une image dont le score tombe sous 60 % de la référence est considérée fortement dégradée ;
- au maximum **2 recaptures immédiates** sont effectuées par cycle ;
- si la troisième image reste faible, le pipeline continue malgré tout pour éviter un blocage dû au seuil ;
- les compteurs de recapture sont exposés pour valider ce seuil sur le terrain.

Le contrôleur chronomètre maintenant le dernier cycle par poste :

```text
capture_ms       somme des captures du cycle, recaptures incluses
sharpness_ms     somme des contrôles de netteté
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
```

**Frontières de test :** calibration absente -> démarrage refusé ; image nette -> pipeline normal ; chute forte du score -> recapture ; maximum deux recaptures ; cible absente -> cycle suivant ; perte calibration -> arrêt ; aucun backlog si le cycle dépasse l'intervalle.

### `CameraResolutionController` / `CameraSettingsApiHandler`

`CameraResolutionController` reste propriétaire de la résolution. `CameraSettingsApiHandler` l'agrège simplement avec les réglages caméra :

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
```

### Interface Web ESPHome

Les valeurs principales (`distance`, `Z`, `X/Y`, angles de visée, qualité) affichent **la dernière mesure valide** et ne repassent plus à `N/A` lorsqu'un cycle ne détecte pas la cible. L'état `03 Cible actuelle` indique séparément le résultat du cycle courant.

Les timings capture/netteté/filtre/détection/calcul et les compteurs de recapture sont également affichés afin de guider les optimisations futures.

### `ApiWsdlHandler`

`GET /api/wsdl` est la référence du contrat HTTP. Version actuelle : **13** depuis l'ajout du contrôle de netteté et des timings détaillés du mode continu.

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

1. compiler/flasher le contrôle de netteté V1 ;
2. observer les scores sur plusieurs dizaines de captures nettes puis provoquer volontairement du flou ;
3. vérifier que les recaptures réduisent les détections KO sans créer de faux rejets ;
4. exploiter les timings détaillés pour prioriser les optimisations ;
5. valider ensuite les angles de visée par déplacements connus ;
6. retravailler la pose yaw/pitch/roll ;
7. tester haute résolution + ROI pour augmenter la précision sans multiplier le coût global.
