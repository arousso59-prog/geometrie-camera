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
├── TargetDetectionService
│   ├── TargetDetector
│   │   ├── TargetCandidateFinder
│   │   ├── TargetCornerRefiner
│   │   └── TargetCodeDecoder
│   └── TargetDetectionApiHandler
├── TargetDetectionPreview
├── MeasurementManager
│   ├── TargetDetector
│   └── GeometryMeasurementEngine
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
└── ApiWsdlHandler
```

## Chaîne image actuelle

```text
OV5640 / JPEG natif
        ↓
JpegDiagnostic
  - purge de la frame ESPHome pré-acquise
  - demande d'une frame fraîche
  - copie JPEG persistante en PSRAM
        ↓
JpegFilteredDiagnostic / optimisation filtre V2
  - décodage TJpgDec par blocs
  - conversion RGB -> luminance 8 bits
  - classification des graines vertes directement dans le callback
  - workspace JPEG 4 ko persistant
        ↓
masque vert brut 1 bit/pixel
        ↓
JpegArtifactCorrector V5 lookup
  - construction de deux masques dérivés persistants en PSRAM
    * graines vertes dilatées horizontalement ±2 px
    * graines vertes fines
  - parcours sparse des défauts
  - réparation verte/noire avec les mêmes seuils fonctionnels
        ↓
GrayFrameView corrigé
        ↓
TargetDetectionService
        ↓
TargetDetector V5.4
  ├── TargetCandidateFinder V5.2
  ├── TargetCornerRefiner V5.4
  └── TargetCodeDecoder V5.4
        ↓
TargetObservation
```

## Responsabilités

### `GeometrieCameraApp`

Orchestration uniquement : initialisation, injection de la caméra ESPHome, boucle légère et enregistrement des handlers HTTP.

### `JpegDiagnostic`

Acquisition JPEG native de l'OV5640. ESPHome conservant une frame pré-acquise, une demande de capture consomme d'abord cette frame puis demande une nouvelle frame fraîche. Le JPEG est copié en PSRAM car le framebuffer caméra est éphémère.

### `JpegFilteredDiagnostic`

Responsabilité : transformer le JPEG en image grayscale corrigible et conserver le résultat exploitable par la vision.

Il gère :

- le décodage TJpgDec ;
- la conversion RGB vers luminance ;
- la construction du masque vert brut pendant le décodage ;
- le buffer BMP/grayscale en PSRAM ;
- l'appel au correcteur ;
- les timings `decode_ms`, `correction_ms` et `total_ms`.

L'optimisation V2 conserve le workspace TJpgDec de 4 ko entre deux traitements au lieu de l'allouer/libérer à chaque appel. La classification d'une graine verte est maintenant une petite fonction locale au même `.cpp` que le callback JPEG, ce qui évite un appel de méthode externe pour chaque pixel décodé tout en conservant exactement les seuils précédents.

Accès métier :

```text
grayscale_data()
grayscale_stride()
width()
height()
```

Référence mesurée après optimisation V1 à 1600×1200 :

```text
decode_ms      ≈ 1661 ms
correction_ms  ≈ 965 ms
total_ms       ≈ 2634 ms
```

La V2 doit être comparée à cette référence sur plusieurs traitements du même JPEG.

### `JpegArtifactCorrector`

Responsabilité : corriger les petits segments verts/noirs périodiques sans appliquer de flou global.

#### V1 / V4 sparse

La V1 a supprimé le balayage horizontal exhaustif : le masque vert compact est parcouru par groupes de bits et les zones sans graine sont sautées. Cette évolution a fait passer la correction d'environ 2,4 s à environ 0,95 s à 1600×1200 sans régression observée sur la détection.

#### V2 / lookup masks

La V2 ne change pas les seuils ni les règles de réparation. Elle évite surtout de recalculer les mêmes voisinages des milliers de fois.

Deux workspaces 1 bit/pixel sont construits une fois par image puis réutilisés pendant toute la correction :

1. `near_green_mask_` : indique directement si une graine verte existe dans le voisinage horizontal ±2 px ;
2. `thin_green_mask_` : indique directement quelles graines satisfont le critère de finesse verticale.

Ainsi :

- le test « graine verte proche » devient une lecture de bit au lieu de cinq lectures ;
- la recherche de référence verticale n'a plus besoin de recalculer ce voisinage ;
- la recherche sparse saute directement entre graines fines ;
- les deux buffers sont persistants et alloués explicitement en PSRAM.

À 1600×1200, chaque masque représente environ 240 ko, soit environ 480 ko de workspace supplémentaire. Avec 8 Mo de PSRAM, ce budget reste acceptable pour la phase actuelle.

**Frontière de test V2 :** pour un même JPEG, vérifier que `green_seed_pixels`, `thin_green_pixels`, `corrected_green_pixels`, `corrected_dark_pixels`, `corrected_total_pixels`, l'image corrigée et le résultat `/target/detect` restent cohérents avec la V1, tout en réduisant `correction_ms` et idéalement `decode_ms`.

### `TargetDetector`

Orchestrateur vision de la cible. Il délègue :

1. localisation globale à `TargetCandidateFinder` ;
2. raffinement pleine résolution des coins à `TargetCornerRefiner` ;
3. lecture/validation du code à `TargetCodeDecoder`.

Pour chaque candidat, le décodeur teste toujours le quadrilatère brut. Si le raffinement réussit, le quadrilatère raffiné est également testé et le meilleur résultat est conservé.

Le tableau de candidats est persistant dans `TargetDetector` afin d'éviter les gros temporaires sur la pile du handler HTTP.

### `TargetCandidateFinder`

Responsabilité : trouver rapidement les zones où la cible peut se trouver.

Méthode V5.2 :

- réduction dynamique vers ~320 px maximum ;
- cinq échantillons par cellule réduite ;
- seuillage adaptatif local ;
- composantes connexes ;
- rejet des formes trop petites/grandes/allongées ;
- estimation approximative des quatre coins ;
- conservation des 8 meilleurs candidats ;
- buffers de travail persistants en PSRAM.

À 1600×1200, la réduction est typiquement ×5 vers 320×240.

### `TargetCornerRefiner`

Responsabilité : raffiner les quatre coins d'un candidat sur l'image pleine résolution.

Méthode V5.4 :

- recherche locale autour de chaque coin ;
- score basé sur les transitions extérieur clair / bord noir ;
- contrôle diagonal ;
- pénalité de déplacement ;
- validation des longueurs d'arêtes et de l'aire ;
- repli sur le candidat brut si le raffinement n'est pas convaincant.

Ces quatre coins seront réutilisés pour la mesure de distance et d'orientation.

### `TargetCodeDecoder`

Responsabilité : valider que le quadrilatère contient réellement la cible 7×7.

Méthode V5.4 :

- petites dilatations du quadrilatère ;
- homographie carré unité -> quadrilatère ;
- plusieurs échantillons par cellule ;
- rotations 0/90/180/270° ;
- contraste minimal ;
- cadre noir ;
- extérieur plus clair que le noir de la cible ;
- seuil final d'acceptation = 0.82.

### `TargetDetectionService`

Pont sans copie entre l'image corrigée et `TargetDetector`. Il ne déclenche ni capture ni filtrage.

### `TargetDetectionPreview`

Construit à la demande une miniature grayscale de diagnostic avec le meilleur résultat.

### `TargetDetectionApiHandler`

```text
GET /target/detect
GET /target/status
GET /target/preview.bmp
```

### `MeasurementManager`

Enchaînera la détection puis `GeometryMeasurementEngine` pour distance et orientation une fois la chaîne image validée.

### `GeometryMeasurementEngine`

Calcul mathématique indépendant du matériel et du réseau. Distance, orientation et calibration seront complétées après validation du filtre V2 et de la détection cible.

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
```

La V2 du filtre ne modifie aucune route, aucun paramètre ni aucun contrat JSON. Le WSDL-like reste donc inchangé.

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

1. **Valider l'optimisation filtre V2** sur plusieurs traitements 1600×1200 ;
2. vérifier que la détection cible n'est pas impactée ;
3. si le gain V2 est significatif, figer cette version du filtre ;
4. passer à la calibration optique ;
5. calculer distance et angles à partir des quatre coins ;
6. mettre en place l'acquisition continue ;
7. ajouter ensuite la ROI autour de la dernière cible pour la voie rapide.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
