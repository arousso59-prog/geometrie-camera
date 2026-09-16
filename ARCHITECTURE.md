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
   - éviter aussi les gros tableaux temporaires sur la pile des handlers HTTP : les candidats V5 sont persistants dans `TargetDetector`.

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
  - comptage des graines vertes pendant le décodage
        ↓
JpegArtifactCorrector V4 sparse
  - correction locale des artefacts verts/noirs
  - parcours direct du masque compact au lieu d'un scan pixel par pixel
        ↓
GrayFrameView corrigé
        ↓
TargetDetectionService
        ↓
TargetDetector V5.4
  ├── TargetCandidateFinder V5.2
  │     - réduction de l'image à ~320 px max
  │     - seuillage local
  │     - composantes sombres
  │     - estimation de quatre coins approximatifs
  │     - conservation des 8 meilleurs candidats
  ├── TargetCornerRefiner V5.4
  │     - recherche locale autour des quatre coins
  │     - évaluation des transitions clair extérieur / noir intérieur
  │     - validation géométrique du quadrilatère raffiné
  └── TargetCodeDecoder V5.4
        - test du candidat brut en secours
        - test du candidat raffiné
        - projection projective par homographie
        - lecture du motif 7×7
        - test des quatre orientations logiques
        - validation du contraste, du cadre noir et du fond extérieur
        - seuil final d'acceptation = 0.82
        ↓
TargetObservation
```

Une fois la détection réelle validée, `MeasurementManager / GeometryMeasurementEngine` réutiliseront la géométrie de la cible pour distance et orientation fine.

## Responsabilités

### `GeometrieCameraApp`

Orchestration uniquement : initialisation, injection de la caméra ESPHome, boucle légère et enregistrement des handlers HTTP.

### `JpegDiagnostic`

Acquisition JPEG native de l'OV5640. ESPHome conservant une frame pré-acquise, une demande de capture consomme d'abord cette frame puis demande une nouvelle frame réellement fraîche. Le JPEG est copié en PSRAM car le framebuffer caméra est éphémère.

### `JpegFilteredDiagnostic`

Prépare l'image exploitable par la vision : décodage JPEG par blocs, conversion immédiate en luminance 8 bits, construction du masque d'artefacts, application du correcteur et conservation du buffer grayscale corrigé. Le BMP reste uniquement une visualisation de diagnostic.

Accès métier :

```text
grayscale_data()
grayscale_stride()
width()
height()
```

Avant l'optimisation V1 du filtre, les mesures à 1600×1200 tournaient autour de ~1,6 s pour le décodage JPEG et ~2,4 s pour la correction d'artefacts, soit environ 4 s au total, alors que la détection V5 était déjà de l'ordre de quelques dixièmes de seconde. La V1 supprime plusieurs parcours complets inutiles : les graines vertes sont comptées pendant le décodage, le buffer image n'est plus intégralement remis à zéro avant d'être réécrit, et le correcteur parcourt directement les octets non nuls du masque. Les nouveaux temps doivent être validés sur l'ESP32 avant de fixer une nouvelle référence.

### `JpegArtifactCorrector`

Responsabilité : corriger le défaut observé sur la voie JPEG : petits segments verts/noirs périodiques, sans flou global.

La V4 conserve les règles de correction de la V3 mais remplace le balayage horizontal exhaustif par une recherche **sparse** :

- le masque vert reste compact à 1 bit/pixel ;
- les lignes sont examinées par groupes de 8 pixels ;
- les groupes sans aucune graine verte sur la ligne ou ses voisines sont sautés immédiatement ;
- les positions réellement candidates sont ensuite validées avec les mêmes règles de finesse verticale ;
- les intervalles et les règles de réparation vert/noir restent inchangés ;
- le nombre brut de graines vertes est fourni directement par le décodeur JPEG, supprimant un second scan complet du masque.

La frontière de test reste simple : à JPEG identique, les statistiques de correction et l'image grayscale corrigée doivent rester équivalentes à la V3, tandis que `correction_ms` doit diminuer sensiblement.

### `TargetDetector`

Orchestrateur vision de la cible uniquement. Depuis la V5 il ne balaie plus l'image entière avec le code 7×7. En V5.4 il délègue :

1. la localisation globale à `TargetCandidateFinder` ;
2. le raffinement pleine résolution des coins à `TargetCornerRefiner` ;
3. la lecture/validation du code à `TargetCodeDecoder`.

Pour chaque candidat, le décodeur teste toujours le quadrilatère brut. Si le raffinement des coins réussit, le quadrilatère raffiné est également décodé et le meilleur résultat est conservé. Le raffinement ne peut donc pas supprimer une détection déjà obtenue avec le candidat brut.

Le tableau des candidats est persistant dans l'objet `TargetDetector` et non local à `detect()`. Cette règle a été introduite après avoir observé un `vApplicationStackOverflowHook` dans le thread HTTP avec la première V5.

### `TargetCandidateFinder`

Responsabilité : **trouver rapidement les zones où la cible peut se trouver**, sans chercher encore une géométrie subpixel.

Méthode V5.2 :

- réduction dynamique pour garder le plus grand côté proche de 320 px ;
- cinq échantillons rapides par cellule réduite plutôt qu'une moyenne exhaustive du bloc source ;
- calcul de luminosité locale par tuiles ;
- seuillage adaptatif ;
- composantes connexes 4-voisins ;
- rejet des composantes trop petites, trop grandes ou trop allongées ;
- estimation approximative de quatre coins par extrema `x+y` / `x-y` ;
- maximum 8 candidats conservés, classés principalement par forme carrée ;
- pendant la validation, les huit candidats sont journalisés avec centre, taille et score sans modifier l'API.

À 1600×1200, la réduction est typiquement ×5 et produit 320×240 pixels : une cible d'environ 40 px reste de l'ordre de 8 px dans la carte de localisation. Cette précision est suffisante pour proposer une zone mais pas toujours pour décoder correctement une cible en perspective ; c'est précisément la responsabilité du `TargetCornerRefiner`.

Le workspace de localisation est persistant et alloué explicitement en PSRAM. À 1600×1200, la carte réduite représente environ 75 ko et la file de composantes peut atteindre environ 300 ko.

**Frontières de test :** carré sombre sur fond clair, lignes verticales parasites, plusieurs objets, cible déplacée, faible contraste local, allocation PSRAM, stabilité de pile, présence de la vraie cible dans les huit candidats.

### `TargetCornerRefiner`

Responsabilité : **transformer un quadrilatère approximatif issu de l'image réduite en quatre coins plus précis sur l'image pleine résolution**.

Méthode V5.4 :

- rayon de recherche adapté à la taille du candidat, borné entre 3 et 12 px ;
- recherche indépendante autour de chacun des quatre coins ;
- score fondé sur les deux transitions attendues au coin : extérieur clair vers bord noir sur chaque côté ;
- contrôle diagonal supplémentaire ;
- pénalité de déplacement pour éviter de sauter vers un objet voisin ;
- reconstruction du centre, largeur et hauteur ;
- validation des longueurs d'arêtes et de l'aire du quadrilatère ;
- conservation du candidat brut si le raffinement n'améliore pas suffisamment le score des coins.

Cette classe ne connaît pas le motif 7×7 : elle ne fait que de la géométrie/contraste de bord. Les coins obtenus sont donc réutilisables plus tard pour la distance, la perspective et l'orientation.

**Frontière de test principale :** `GrayFrameView + TargetCandidate approximatif -> TargetCandidate raffiné`. Cas prioritaires : cible droite, cible inclinée, perspective, coin proche d'un autre bord sombre, faible contraste, candidat déjà précis.

### `TargetCodeDecoder`

Responsabilité : **dire si un candidat géométrique est réellement notre cible**.

Méthode V5.4 :

- teste plusieurs petites dilatations du quadrilatère pour compenser les incertitudes restantes ;
- utilise une homographie directe carré unité -> quadrilatère pour projeter correctement une cible plane vue en perspective ;
- chaque cellule du 7×7 utilise plusieurs échantillons de l'image pleine résolution ;
- teste les quatre rotations logiques 0/90/180/270 degrés ;
- exige indépendamment un contraste minimal, un cadre noir cohérent et un extérieur plus clair que le noir de la cible ;
- seuil final d'acceptation = 0.82 ;
- journalise score, `pattern`, `border`, contraste, niveau extérieur, niveau noir, facteur d'expansion et rotation.

L'ancienne interpolation bilinéaire reste uniquement un repli de sécurité si le calcul projectif devient dégénéré.

**Frontières de test :** vrai code, faux carré noir, sous-motif interne, quatre rotations, perspective légère et marquée, contraste faible, fond extérieur sombre, cible absente avec `target_found=false`.

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

Les routes restent inchangées avec la V5.4.

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

Les essais de timing n'ont pas supprimé le motif parasite ; la correction logicielle V4 sparse reste la voie retenue.

## Optimisations après validation V5.4

La V5 réduit déjà fortement le coût de détection. Le chantier de performance actuel porte sur `JpegFilteredDiagnostic` / `JpegArtifactCorrector` :

1. **V1 réalisée :** supprimer les scans complets évitables et rendre la correction sparse sans changer ses règles fonctionnelles ;
2. mesurer les nouveaux temps sur plusieurs frames et comparer les statistiques/images à la V3 ;
3. après première détection globale, mémoriser la dernière cible ;
4. recherche suivante dans une ROI ;
5. correction d'artefacts limitée à cette ROI lorsque l'architecture de décodage le permet ;
6. étudier le décodage JPEG partiel/par blocs ;
7. retour automatique à la recherche globale si la cible est perdue.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
