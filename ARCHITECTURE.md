# Architecture du projet `geometrie-camera`

## Règles de développement

1. **Les fichiers `.h` sont déclaratifs uniquement.**
   - déclarations de classes, structures, enums et méthodes ;
   - implémentations, données significatives et logique dans les `.cpp`.

2. **Une classe = une responsabilité principale.**
   - `GeometrieCameraApp` reste un orchestrateur ;
   - aucun algorithme vision, calcul géométrique ou parsing HTTP métier dans l'orchestrateur.

3. **Avant toute nouvelle fonction, revoir l'architecture.**
   - identifier la responsabilité ;
   - choisir une classe existante seulement si la responsabilité lui appartient réellement ;
   - sinon créer un sous-système dédié ;
   - éviter les dépendances vers l'application complète.

4. **Penser chaque évolution avec les tests.**
   - séparer logique pure et matériel ESP32 ;
   - conserver des frontières testables ;
   - injecter les dépendances lorsqu'un accès matériel empêcherait un test unitaire.

5. **Toute évolution de l'API HTTP met à jour `GET /api/wsdl` dans le même changement.**
   - ajout/suppression de route ;
   - changement de méthode, paramètre ou comportement contractuel ;
   - le WSDL-like doit refléter exactement les routes compilées.

6. **Les gros buffers image/vision doivent être explicitement budgétés et placés en PSRAM.**
   - ne pas utiliser `std::vector` par défaut pour des workspaces pouvant dépasser quelques dizaines de kilo-octets ;
   - réutiliser les buffers entre appels plutôt que réallouer ;
   - réserver la RAM interne aux structures légères et aux besoins temps-réel ;
   - éviter les gros tableaux temporaires sur la pile des handlers HTTP.

## Architecture actuelle

```text
GeometrieCameraApp
├── CameraResolutionController
├── CameraSettingsController
│   └── CameraSettingsApiHandler
├── JpegDiagnostic
│   └── JpegDiagnosticApiHandler
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
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
└── ApiWsdlHandler
```

Le `TargetDetector` appartient directement à l'application. `MeasurementManager` ne contient pas de second détecteur : la mesure réutilise exactement le `TargetObservation` déjà validé par `TargetDetectionService`.

## Chaîne image et mesure actuelle

```text
OV5640 / JPEG natif
        ↓
JpegDiagnostic
        ↓
JpegFilteredDiagnostic / optimisation filtre V2
  - TJpgDec par blocs
  - RGB -> luminance 8 bits
  - masque vert brut pendant le décodage
  - workspace JPEG 4 ko persistant
        ↓
JpegArtifactCorrector V5 lookup
  - deux masques dérivés persistants en PSRAM
  - parcours sparse
        ↓
GrayFrameView corrigé
        ↓
TargetDetector V5.4
  ├── localisation réduite
  ├── raffinement des quatre coins pleine résolution
  └── décodage 7×7 par homographie
        ↓
TargetObservation
  - centre / taille
  - rotation logique du code
  - qualité
  - 4 coins géométriques du candidat validé
        ↓
MeasurementManager
        ↓
GeometryMeasurementEngine V2
  - calibration fx/fy à distance connue
  - adaptation de calibration à la résolution courante
  - distance principale par taille apparente
  - X/Y/Z à partir du rayon du centre de cible
  - homographie conservée uniquement pour l'orientation
  - validation de cohérence de la pose
        ↓
GeometryMeasurement
  - distance / X / Y / Z
  - z depuis largeur et hauteur
  - angles de visée
  - pose_valid
  - yaw / pitch / roll seulement si pose cohérente
```

## Responsabilités

### `GeometrieCameraApp`

Orchestration uniquement : initialisation, injection de la caméra ESPHome, boucle légère, possession des sous-systèmes et enregistrement des handlers HTTP.

### `JpegDiagnostic`

Acquisition JPEG native de l'OV5640. Une demande de capture purge d'abord la frame pré-acquise par ESPHome puis demande une frame fraîche. Le JPEG est copié en PSRAM car le framebuffer caméra est éphémère.

### `JpegFilteredDiagnostic`

Responsabilité : transformer le JPEG en image grayscale corrigible et conserver le résultat exploitable par la vision.

Il gère :

- le décodage TJpgDec ;
- la conversion RGB vers luminance ;
- la construction du masque vert brut pendant le décodage ;
- le buffer BMP/grayscale en PSRAM ;
- l'appel au correcteur ;
- les timings `decode_ms`, `correction_ms` et `total_ms`.

L'optimisation V2 conserve le workspace TJpgDec de 4 ko entre deux traitements et réduit le travail du callback JPEG.

Référence validée à 1600×1200 après V2 :

```text
decode_ms      ≈ 1476 ms
correction_ms  ≈ 553 ms
total_ms       ≈ 2036 ms
```

Les essais répétés restent du même ordre de grandeur et la détection de cible n'a pas montré de régression. Cette V2 devient donc la base de travail actuelle ; les optimisations supplémentaires sont reportées après validation de la mesure.

### `JpegArtifactCorrector`

Responsabilité : corriger les petits segments verts/noirs périodiques sans flou global.

La version actuelle conserve les règles fonctionnelles de correction mais utilise :

- un parcours sparse du masque vert ;
- `near_green_mask_` pour le voisinage horizontal ±2 px ;
- `thin_green_mask_` pour les graines satisfaisant le critère de finesse verticale ;
- des buffers persistants en PSRAM.

À 1600×1200, les deux masques dérivés utilisent environ 480 ko de PSRAM supplémentaires.

### `TargetDetector`

Orchestrateur vision de la cible. Il délègue :

1. localisation globale à `TargetCandidateFinder` ;
2. raffinement des coins à `TargetCornerRefiner` ;
3. validation du code à `TargetCodeDecoder`.

Le candidat brut reste toujours testé. Si le raffinement réussit, le candidat raffiné est également testé. Le score de décodage et la géométrie des coins sont séparés : lorsqu'un candidat raffiné valide la même rotation logique, ses coins peuvent être conservés même si le candidat brut garde un score de code légèrement supérieur.

### `TargetCandidateFinder`

Responsabilité : trouver rapidement les zones où la cible peut se trouver.

Méthode actuelle : réduction vers ~320 px, seuillage adaptatif, composantes connexes, filtrage géométrique et conservation des 8 meilleurs candidats. Les buffers de travail sont persistants en PSRAM.

Les logs flottants détaillés sont désactivés sur le thread HTTP afin d'éviter les conversions `printf` coûteuses en pile observées lors d'un `StoreProhibited` dans `_dtoa_r`.

### `TargetCornerRefiner`

Responsabilité : raffiner les quatre coins d'un candidat sur l'image pleine résolution sans connaître le motif 7×7.

Les quatre coins obtenus sont réutilisables par le décodage puis par la mesure géométrique.

### `TargetCodeDecoder`

Responsabilité : valider que le quadrilatère contient réellement la cible 7×7.

La V5.4 utilise :

- plusieurs dilatations du quadrilatère ;
- homographie carré unité -> quadrilatère ;
- plusieurs échantillons par cellule ;
- rotations 0/90/180/270° ;
- contraste, cadre noir et fond extérieur ;
- seuil final `0.82`.

Les facteurs de dilatation servent uniquement à l'échantillonnage du code. Les coins conservés dans `TargetObservation` restent ceux du candidat géométrique d'entrée afin qu'un changement de facteur de lecture ne modifie pas artificiellement la taille physique utilisée pour la distance.

### `TargetDetectionService`

Pont sans copie entre l'image corrigée et `TargetDetector`. Il mémorise la dernière observation et le compteur de source utilisé. Il ne déclenche ni capture ni filtrage.

### `TargetDetectionPreview`

Construit à la demande une miniature grayscale de diagnostic avec le meilleur résultat.

### `TargetDetectionApiHandler`

```text
GET /target/detect
GET /target/status
GET /target/preview.bmp
```

### `MeasurementManager`

Responsabilité : conserver la dernière mesure et son compteur, puis déléguer les calculs mathématiques à `GeometryMeasurementEngine`.

Il **ne détecte pas la cible**. Son entrée est :

```text
TargetObservation + frame_width + frame_height + timestamp
```

Cette séparation évite un deuxième passage du détecteur et constitue une frontière de test simple.

### `GeometryMeasurementEngine`

Responsabilité : calcul mathématique pur de calibration, distance, position et orientation.

#### Taille de cible

Valeur par défaut actuelle :

```text
target_size_mm = 50.0
```

Cette valeur correspond au carré physique complet utilisé par le détecteur. Elle est configurable par API.

#### Calibration à distance connue

La résolution seule ne permet pas de convertir une taille en pixels en distance absolue : il faut connaître la focale effective de l'objectif. Le champ de vision annoncé par le vendeur n'est donc pas utilisé comme vérité de calibration.

Procédure :

1. placer la cible approximativement de face et proche du centre optique ;
2. mesurer physiquement la distance caméra -> cible ;
3. capture + filtre + détection ;
4. appeler `/measurement/calibrate?distance_mm=...` ;
5. calculer :

```text
fx_px = largeur_cible_px  × distance_connue_mm / taille_cible_mm
fy_px = hauteur_cible_px  × distance_connue_mm / taille_cible_mm
cx_px = centre horizontal de l'image
cy_px = centre vertical de l'image
```

Une fois valide, cette calibration devient **verrouillée**. Un nouvel appel à `/measurement/calibrate` sans `force=1` retourne `calibration_locked` et ne modifie aucune valeur. Modifier explicitement `target_size_mm` invalide la calibration et lève le verrou.

La calibration mémorise la résolution de référence. Pour une autre résolution de **même cadrage optique**, `fx`, `fy`, `cx` et `cy` sont redimensionnés proportionnellement.

Cette calibration ignore encore la distorsion radiale de l'objectif.

#### Distance V2 robuste

La première décomposition homographique a montré qu'un quadrilatère de coins imparfait pouvait produire une profondeur très fausse alors que la taille apparente de la cible restait cohérente. La distance principale ne dépend donc plus de la pose projective.

À partir des quatre coins canoniques :

```text
largeur_px = moyenne(arête haute, arête basse)
hauteur_px = moyenne(arête gauche, arête droite)

z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
z_mm             = min(z_from_width_mm, z_from_height_mm)
```

Le choix du minimum limite la surestimation de distance quand une inclinaison raccourcit une seule dimension projetée.

Le centre de cible définit ensuite le rayon optique :

```text
nx = (center_x - cx) / fx
ny = (center_y - cy) / fy

x_mm = nx × z_mm
y_mm = ny × z_mm
distance_mm = sqrt(x_mm² + y_mm² + z_mm²)
```

Les angles de visée sont obtenus directement depuis `nx` et `ny` et restent indépendants de la pose du plan.

#### Orientation du plan et `pose_valid`

L'homographie des quatre coins est toujours décomposée pour estimer `target_yaw_deg`, `target_pitch_deg` et `target_roll_deg`, mais elle n'a plus le droit de modifier `distance_mm` ou `x/y/z`.

La profondeur issue de cette pose est exposée dans :

```text
pose_z_mm
pose_scale_error_pct
```

avec :

```text
pose_scale_error_pct = abs(pose_z_mm - z_mm) / z_mm × 100
```

La pose n'est acceptée que si l'erreur reste inférieure ou égale à **25 %**. Sinon :

```text
measurement.valid = true
pose_valid = false
target_yaw/pitch/roll = 0
```

La distance et les angles de visée restent donc utilisables même si les quatre coins ne sont pas encore assez fiables pour une orientation de plan précise.

Le `roll` accepté est normalisé modulo 180° dans l'intervalle `[-90°, +90°)` afin qu'une cible presque droite décodée à 180° ne soit pas affichée autour de ±180°.

**Frontières de test :** cible frontale à distance connue, plusieurs distances sans recalibration, déplacement horizontal/vertical, comparaison `z_from_width` / `z_from_height`, rejet d'une pose incohérente, rotation en roulis, inclinaison yaw/pitch, changement 1600×1200 ↔ 800×600, répétabilité sur captures successives.

### `MeasurementApiHandler`

```text
GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>&force=<0|1>
GET /measurement/compute
GET /measurement/status
```

`/measurement/calibrate` et `/measurement/compute` exigent une détection correspondant à la dernière image filtrée. Ils ne relancent ni capture, ni filtre, ni détection.

Le JSON expose désormais :

```text
calibration.locked
measurement.pose_valid
measurement.z_from_width_mm
measurement.z_from_height_mm
measurement.pose_z_mm
measurement.pose_scale_error_pct
```

### `CameraResolutionController`

Maintient l'identité capteur et les résolutions supportées. Capteur confirmé : OV5640 PID `0x5640`, matrice physique 2592×1944 ; ESPHome 2026.7.3 expose QSXGA 2560×1920 comme mode maximal.

### `CameraSettingsController` / `CameraSettingsApiHandler`

Réglages exposition, gain, luminosité et contraste.

### `RuntimeDiagnostics` / `RuntimeDiagnosticsApiHandler`

Instrumentation légère de boucle et mémoire.

### `ApiWsdlHandler`

Catalogue des routes HTTP réellement compilées :

```text
GET /api/wsdl
```

Le WSDL-like est en version **11** depuis le verrouillage de calibration et la distance V2 robuste.

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

## État matériel / image retenu

```text
OV5640
JPEG
2560×1920 QSXGA au démarrage
jpeg_quality = 10
XCLK = 8 MHz
1 framebuffer en PSRAM
idle_framerate = 0
```

## Feuille de route immédiate

1. compiler/flasher la distance V2 robuste ;
2. calibrer une seule fois à une distance précisément connue ;
3. vérifier que `calibration.locked=true` et qu'un deuxième calibrage sans `force=1` est refusé ;
4. déplacer la cible à plusieurs distances sans recalibrer ;
5. comparer `z_from_width_mm`, `z_from_height_mm`, `z_mm` et la distance réelle ;
6. vérifier `bearing_yaw/pitch` par déplacement de la cible ;
7. observer `pose_valid` avant de retravailler le raffinement des coins ;
8. améliorer ensuite calibration optique/distorsion et pose si nécessaire ;
9. passer à l'acquisition continue ;
10. revenir ensuite sur ROI et performances.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
