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
   - réserver la RAM interne aux structures légères et aux besoins temps-réel.

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

## Chaîne image validée

```text
OV5640 / JPEG natif
        ↓
JpegDiagnostic
  - purge de la frame ESPHome pré-acquise
  - demande d'une frame fraîche
  - copie JPEG persistante en PSRAM
        ↓
JpegFilteredDiagnostic
  - décodage JPEG par blocs
  - conversion immédiate en luminance 8 bits
        ↓
JpegArtifactCorrector V3
  - correction locale des artefacts verts/noirs
        ↓
GrayFrameView corrigé
        ↓
TargetDetectionService
        ↓
TargetDetector V5
  ├── TargetCandidateFinder
  │     - réduction de l'image à ~400 px max
  │     - seuillage local
  │     - composantes sombres
  │     - rejet des objets trop allongés
  │     - estimation de quatre coins
  │     - conservation des meilleurs candidats
  └── TargetCodeDecoder
        - projection du quadrilatère sur l'image pleine résolution
        - lecture du motif 7×7
        - test des quatre orientations logiques
        - validation du cadre noir et du fond extérieur
        ↓
TargetObservation
```

Une fois la détection réelle validée, `MeasurementManager / GeometryMeasurementEngine` utiliseront la géométrie de la cible pour distance et orientation fine.

## Responsabilités

### `GeometrieCameraApp`

Orchestration uniquement : initialisation, injection de la caméra ESPHome, boucle légère et enregistrement des handlers HTTP.

### `JpegDiagnostic`

Acquisition JPEG native de l'OV5640. ESPHome conservant une frame pré-acquise, une demande de capture consomme d'abord cette frame puis demande une nouvelle frame réellement fraîche. Le JPEG est copié en PSRAM car le framebuffer caméra est éphémère.

### `JpegFilteredDiagnostic`

Prépare l'image exploitable par la vision : décodage JPEG par blocs, conversion immédiate en luminance 8 bits, application du correcteur d'artefacts, conservation du buffer grayscale corrigé. Le BMP reste uniquement une visualisation de diagnostic.

Accès métier :

```text
grayscale_data()
grayscale_stride()
width()
height()
```

### `JpegArtifactCorrector`

Algorithme de correction du défaut observé sur la voie JPEG : petits segments verts/noirs périodiques. La V3 corrige localement les défauts reconnus sans flou global.

### `TargetDetector`

Orchestrateur vision de la cible uniquement. Depuis la V5 il ne balaie plus l'image entière avec le code 7×7. Il délègue :

1. la localisation géométrique à `TargetCandidateFinder` ;
2. la lecture/validation du code à `TargetCodeDecoder`.

Il retourne le meilleur `TargetObservation`. Si aucun code n'est validé mais qu'une zone candidate existe, il retourne cette localisation avec `valid=false` afin que `/target/preview.bmp` reste utile pour le diagnostic.

### `TargetCandidateFinder`

Responsabilité : **trouver la cible dans l'image**, sans connaître le contenu exact du code 7×7.

Méthode V5 :

- réduction dynamique pour garder le plus grand côté proche de 400 px ;
- moyenne des blocs source lors de la réduction ;
- calcul de luminosité locale par tuiles ;
- seuillage adaptatif : seules les zones significativement plus sombres que leur environnement sont retenues ;
- composantes connexes 4-voisins ;
- rejet des composantes trop petites, trop grandes ou trop allongées ;
- estimation de quatre coins par extrema `x+y` / `x-y` de la composante ;
- maximum 24 candidats conservés, classés principalement par forme carrée.

À 1600×1200, la réduction est typiquement ×4 : une cible de 40 px reste donc de l'ordre de 10 px dans la carte de localisation. À 2560×1920, le facteur augmente automatiquement pour garder un coût voisin.

Le workspace de localisation est persistant et alloué explicitement en PSRAM. À 1600×1200, il représente typiquement environ 120 ko pour l'image réduite et jusqu'à 480 ko pour la file de composantes connexes. Ces buffers ne doivent pas revenir dans le heap interne via des conteneurs STL par défaut.

**Frontières de test :** carré sombre sur fond clair, lignes verticales parasites, plusieurs objets, cible déplacée dans l'image, faible contraste local, légère rotation/perspective, allocation du workspace en PSRAM.

### `TargetCodeDecoder`

Responsabilité : **dire si un candidat géométrique est réellement notre cible**.

Méthode V5 :

- utilise les quatre coins fournis par le localisateur ;
- teste plusieurs petites dilatations du quadrilatère pour compenser l'imprécision de la segmentation basse résolution ;
- projette les centres de cellules 7×7 dans le quadrilatère ;
- chaque cellule utilise plusieurs échantillons de l'image pleine résolution ;
- teste les quatre rotations logiques 0/90/180/270 degrés ;
- exige un contraste minimal, un cadre noir cohérent et un extérieur plus clair que le noir de la cible ;
- retourne centre, taille, rotation logique et qualité.

La projection quadrilatérale est volontairement déjà présente : elle rend le décodage moins sensible aux petits angles et prépare la future estimation d'orientation.

**Frontières de test :** vrai code, faux carré noir, sous-motif interne, quatre rotations, perspective légère, contraste faible, fond extérieur sombre.

### `TargetDetectionService`

Pont très fin entre l'image corrigée et `TargetDetector` : construit un `GrayFrameView` sans copie, appelle le détecteur et mémorise résultat, source et temps de détection. Il ne déclenche ni capture ni filtrage.

### `TargetDetectionPreview`

Diagnostic visuel séparé : construit à la demande une miniature grayscale de largeur maximale 640 px et dessine un rectangle noir/blanc autour du meilleur résultat. Il ne modifie jamais le buffer métier.

### `TargetDetectionApiHandler`

```text
GET /target/detect
GET /target/status
GET /target/preview.bmp
```

Les routes restent inchangées avec la V5.

### `MeasurementManager`

Enchaînera `TargetDetector` puis `GeometryMeasurementEngine` une fois la cible réelle suffisamment robuste. Il conserve déjà la dernière mesure et le compteur de mesures valides.

### `GeometryMeasurementEngine`

Calcul mathématique indépendant du matériel et du réseau. Distance, orientation et calibration seront complétées après validation de la détection de cible.

### `CameraResolutionController`

Maintient l'identité capteur et les résolutions supportées. Capteur confirmé : OV5640 PID `0x5640`, matrice physique 2592×1944 ; ESPHome 2026.7.3 expose QSXGA 2560×1920 comme mode maximal.

### `CameraSettingsController` / `CameraSettingsApiHandler`

Conservés pour exposition, gain, luminosité et contraste. Les réglages caméra doivent être jugés par comparaison A/B avec la qualité de détection et non seulement visuellement.

### `RuntimeDiagnostics` / `RuntimeDiagnosticsApiHandler`

Instrumentation légère de boucle et mémoire, conservée pour les futures optimisations.

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

## Sous-systèmes supprimés après validation

Les anciens chemins GRAYSCALE, RGB565, TargetSearch GRAYSCALE, essais de registres OV5640, configurateur OV3660, placeholder/ImageProvider/CameraManager et anciennes API historiques ont été supprimés après validation de la voie JPEG.

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

Les essais de timing n'ont pas supprimé le motif parasite ; la correction logicielle V3 reste la voie retenue.

## Optimisations reportées après validation cible/distance/orientation

La V5 réduit déjà le coût de **localisation** parce que l'ancien balayage exhaustif empêchait une validation pratique. Les optimisations de pipeline restent reportées :

1. première recherche globale ;
2. mémorisation de la dernière cible ;
3. recherche suivante dans une ROI ;
4. correction d'artefacts limitée à cette ROI ;
5. décodage JPEG partiel/par blocs si nécessaire ;
6. retour automatique à la recherche globale si la cible est perdue.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
