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

Le `TargetDetector` appartient maintenant directement à l'application. `MeasurementManager` ne contient plus de second détecteur : la mesure réutilise exactement le `TargetObservation` déjà validé par `TargetDetectionService`.

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
  - 4 coins du quadrilatère réellement décodé
        ↓
MeasurementManager
        ↓
GeometryMeasurementEngine V1
  - calibration fx/fy à distance connue
  - adaptation de la calibration à la résolution courante
  - homographie métrique du carré cible
  - décomposition en translation + orientation
        ↓
GeometryMeasurement
  - distance
  - X / Y / Z
  - angles de visée
  - yaw / pitch / roll du plan cible
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

Le candidat brut reste toujours testé. Si le raffinement réussit, le candidat raffiné est également testé et le meilleur résultat est conservé.

### `TargetCandidateFinder`

Responsabilité : trouver rapidement les zones où la cible peut se trouver.

Méthode actuelle : réduction vers ~320 px, seuillage adaptatif, composantes connexes, filtrage géométrique et conservation des 8 meilleurs candidats. Les buffers de travail sont persistants en PSRAM.

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

Lorsqu'un décodage devient le meilleur résultat, ses quatre coins ajustés sont maintenant copiés dans `TargetObservation`. La rotation logique du code permet ensuite de remettre ces coins dans l'ordre physique canonique de la cible.

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

Il **ne détecte plus la cible**. Son entrée est :

```text
TargetObservation + frame_width + frame_height + timestamp
```

Cette séparation évite un deuxième passage du détecteur et constitue une frontière de test simple.

### `GeometryMeasurementEngine`

Responsabilité : calcul mathématique pur de calibration, distance et pose.

#### Taille de cible

Valeur par défaut actuelle :

```text
target_size_mm = 50.0
```

Cette valeur correspond au carré physique complet utilisé par le détecteur. Elle est configurable par API.

#### Calibration V1 à distance connue

La résolution seule ne permet pas de convertir une taille en pixels en distance absolue : il faut connaître la focale effective de l'objectif. Le champ de vision annoncé par le vendeur n'est donc pas utilisé comme vérité de calibration.

Procédure V1 :

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

La calibration mémorise également la résolution de référence. Pour une autre résolution de **même cadrage optique**, `fx`, `fy`, `cx` et `cy` sont redimensionnés proportionnellement.

Cette première calibration ignore encore la distorsion radiale de l'objectif. Elle sert à valider la chaîne de mesure avant une calibration optique complète.

#### Distance et position 3D

Les quatre coins sont remis dans l'ordre canonique grâce à la rotation du code 7×7. Une homographie est construite entre le carré physique de côté `target_size_mm` et ses quatre points image.

La décomposition avec la matrice intrinsèque fournit la translation du centre de cible :

```text
X : droite positive
Y : bas positif
Z : avant positif
```

Le résultat expose :

```text
distance_mm = sqrt(X² + Y² + Z²)
z_mm        = profondeur optique
x_mm
y_mm
```

#### Angles

Deux familles d'angles sont volontairement séparées :

```text
bearing_yaw_deg
bearing_pitch_deg
```

position angulaire du **centre de cible** par rapport à l'axe optique, et :

```text
target_yaw_deg
target_pitch_deg
target_roll_deg
```

orientation du **plan de la cible**.

`roll` utilise l'orientation canonique fournie par le code 7×7 ; une rotation physique de 90° de la cible ne doit donc pas être confondue avec l'ambiguïté géométrique d'un simple carré.

**Frontières de test :** cible frontale à distance connue, plusieurs distances, déplacement horizontal/vertical, rotation en roulis, inclinaison yaw/pitch, changement 1600×1200 ↔ 800×600, répétabilité sur captures successives.

### `MeasurementApiHandler`

```text
GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>
GET /measurement/compute
GET /measurement/status
```

`/measurement/calibrate` et `/measurement/compute` exigent une détection correspondant à la dernière image filtrée. Ils ne relancent ni capture, ni filtre, ni détection.

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

Le WSDL-like est en version **10** depuis l'ajout de la mesure/calibration.

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

1. compiler/flasher la V1 de mesure ;
2. calibrer à une distance connue avec la cible 50 mm bien de face ;
3. vérifier la distance sur plusieurs positions connues ;
4. vérifier `bearing_yaw/pitch` par déplacement de la cible ;
5. vérifier `target_yaw/pitch/roll` en inclinant la cible ;
6. quantifier la répétabilité et les erreurs ;
7. améliorer ensuite la calibration optique/distorsion si nécessaire ;
8. passer à l'acquisition continue ;
9. revenir ensuite sur ROI et performances.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
