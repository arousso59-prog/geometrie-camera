# Architecture du projet `geometrie-camera`

## Règles de développement

Ces règles s'appliquent aux nouveaux développements du projet.

1. **Les fichiers `.h` sont déclaratifs uniquement.**
   - déclarations de classes, structures, enums et méthodes ;
   - pas de corps de méthode dans les headers ;
   - les constructeurs, destructeurs, getters et setters sont implémentés dans les `.cpp` ;
   - les données importantes ne sont pas stockées dans les headers.

2. **Une classe = une responsabilité principale.**
   - une classe qui grossit ou commence à gérer plusieurs responsabilités doit être découpée ;
   - `GeometrieCameraApp` reste un orchestrateur et ne doit pas devenir une classe fourre-tout.

3. **Avant d'ajouter une nouvelle fonction, on revoit l'architecture.**
   - identifier la responsabilité de la fonction ;
   - décider si elle appartient à une classe existante ;
   - sinon créer une nouvelle classe ou un nouveau sous-système ;
   - identifier les dépendances et éviter les dépendances inutiles vers l'application complète.

4. **Chaque évolution doit être pensée avec les tests unitaires.**
   - séparer autant que possible la logique pure du matériel ESP32 ;
   - utiliser des interfaces ou des classes injectables quand un composant matériel doit pouvoir être remplacé par un bouchon de test ;
   - préciser quelles classes et quels comportements devront recevoir des tests.

## Architecture actuelle

```text
GeometrieCameraApp
├── PlaceholderImageProvider : ImageProvider
├── CameraManager
├── MeasurementManager
│   ├── TargetDetector
│   └── GeometryMeasurementEngine
├── CameraApiHandler
├── Ov3660CameraConfigurator
└── Diagnostics camera temporaires
    ├── GrayscaleDiagnostic
    │   └── GrayscaleDiagnosticApiHandler
    └── Rgb565Diagnostic
        └── Rgb565DiagnosticApiHandler
```

### `GeometrieCameraApp`

Responsabilité : orchestration ESPHome uniquement.

- initialise les sous-systèmes ;
- appelle leur boucle ;
- reçoit par injection la vraie `ESP32Camera` déclarée dans le YAML ;
- enregistre les handlers HTTP ;
- expose quelques façades nécessaires au YAML.

Il ne doit pas contenir les algorithmes caméra, vision, calcul géométrique ou sérialisation HTTP.

### `ImageProvider`

Interface de source d'image.

Elle permet à `CameraManager` de fonctionner sans connaître le matériel concret.

Implémentations prévues :

- `PlaceholderImageProvider` : bouchon actuel JPEG noir/blanc ;
- futur `Ov3660ImageProvider` : acquisition réelle ESP32-S3 + OV3660 ;
- éventuellement un provider de test pour les tests unitaires.

**Point de test :** `CameraManager` doit pouvoir être testé avec un faux `ImageProvider` contrôlé.

### `CameraManager`

Responsabilité : cycle d'acquisition d'une image.

- demande une capture au provider ;
- conserve la dernière image ;
- conserve les métadonnées de capture ;
- maintient le compteur de captures.

Il ne doit pas effectuer de traitement de cible ou de calcul d'angle.

**Tests unitaires prévus :**

- capture réussie ;
- capture refusée si provider non prêt ;
- mise à jour compteur/timestamp/métadonnées ;
- propagation d'un échec de capture.

### `MeasurementManager`

Responsabilité : chaîne de mesure.

- demande la détection de cible à `TargetDetector` ;
- demande le calcul d'angle à `GeometryMeasurementEngine` ;
- conserve la dernière mesure ;
- maintient le compteur de mesures valides.

**Tests unitaires prévus :**

- aucune mesure valide si aucune cible ;
- compteur seulement sur mesure valide ;
- transmission correcte du timestamp ;
- plus tard, scénarios de détection connus.

### `TargetDetector`

Responsabilité : transformer une image de traitement en observation de cible.

Il sera découpé si l'algorithme devient important. Sous-classes possibles :

```text
TargetDetector
├── ThresholdProcessor
├── ContourDetector
├── CornerExtractor
├── TargetValidator
└── SubpixelRefiner
```

Le découpage ne sera effectué que lorsque ces responsabilités apparaîtront réellement dans le code.

### `GeometryMeasurementEngine`

Responsabilité : calcul mathématique des angles à partir d'une observation et d'une calibration.

Cette classe doit rester indépendante du réseau et du matériel autant que possible.

**Tests unitaires prioritaires :**

- cible au centre => yaw/pitch proches de zéro ;
- décalages connus => angles attendus ;
- calibration invalide => mesure invalide ;
- rotation cible => roll transmis correctement.

### `CameraApiHandler`

Responsabilité : interface HTTP de l'application uniquement.

- routage des URL `/api/*` et `/image.jpg` ;
- transformation des états en réponses HTTP/JSON ;
- transmission de l'image fournie par `CameraManager`.

Elle dépend directement de `CameraManager` et `MeasurementManager`, pas de toute l'application.

Les diagnostics caméra ne sont volontairement pas ajoutés dans cette classe.

### `GrayscaleDiagnostic`

Responsabilité : diagnostic temporaire de la chaîne d'acquisition brute OV3660 -> ESP32-S3 en niveaux de gris.

- reçoit la vraie `ESP32Camera` par injection ;
- demande une frame uniquement sur ordre explicite ;
- accepte uniquement une frame `PIXFORMAT_GRAYSCALE` ;
- copie la frame brute dans un BMP 8 bits non compressé stocké en PSRAM ;
- calcule quelques statistiques brutes pour vérifier la dynamique reçue ;
- ne fait aucun traitement géométrique et aucune compression JPEG.

Ce sous-système a permis de vérifier que la chaîne parallèle peut produire une image exploitable sans JPEG.

**Tests unitaires / tests de composant à prévoir :**

- rejet d'un format autre que GRAYSCALE ;
- validation de la taille minimale du framebuffer ;
- génération correcte de l'en-tête BMP ;
- conservation exacte des valeurs de pixels source ;
- statistiques min/max/moyenne sur un tableau connu.

### `GrayscaleDiagnosticApiHandler`

Responsabilité : HTTP du diagnostic GRAYSCALE uniquement.

Routes temporaires :

```text
GET /diagnostic/capture
GET /diagnostic/status
GET /diagnostic/raw.bmp
```

Cette classe ne connaît pas `CameraManager`, `MeasurementManager` ou les calculs de géométrie.

### `Rgb565Diagnostic`

Responsabilité : diagnostic temporaire de la chaîne d'acquisition brute couleur en `PIXFORMAT_RGB565`.

- reçoit la même `ESP32Camera` par injection ;
- demande une frame uniquement sur ordre explicite ;
- refuse tout format autre que RGB565 ;
- vérifie que la taille source vaut au moins `largeur × hauteur × 2` ;
- transforme la frame RGB565 en BMP 24 bits non compressé avec le convertisseur du driver Espressif ;
- conserve uniquement le BMP et ses métadonnées pour inspection HTTP.

Ce diagnostic reste séparé de `GrayscaleDiagnostic` afin de ne pas transformer ce dernier en classe multi-format et pour conserver des responsabilités simples pendant les essais matériels.

**Tests unitaires / tests de composant à prévoir :**

- rejet d'un format autre que RGB565 ;
- validation de la taille minimale du framebuffer ;
- conversion de quelques pixels RGB565 connus vers les valeurs RGB attendues ;
- remplacement/libération correcte du buffer BMP entre deux captures.

### `Rgb565DiagnosticApiHandler`

Responsabilité : HTTP du diagnostic RGB565 uniquement.

Routes temporaires :

```text
GET /diagnostic-rgb565/capture
GET /diagnostic-rgb565/status
GET /diagnostic-rgb565/raw.bmp
```

Il ne dépend que de `Rgb565Diagnostic`.

### `Ov3660CameraConfigurator`

Responsabilité : essais bas niveau de registres spécifiques à l'OV3660.

- accès au capteur via `esp_camera_sensor_get()` ;
- vérification du PID OV3660 ;
- application optionnelle d'un diviseur `PCLK_RATIO` demandé depuis le YAML ;
- aucune modification si le diviseur demandé vaut `0`.

Il reste isolé du traitement d'image et n'est utilisé que lorsqu'un test matériel explicite le nécessite.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder une nouvelle fonctionnalité, répondre à ces questions :

1. Quelle est sa responsabilité ?
2. Quelle classe doit la porter ?
3. La classe concernée reste-t-elle cohérente et suffisamment petite ?
4. Faut-il créer une nouvelle classe ou interface ?
5. La logique peut-elle être testée sans ESP32 ni matériel ?
6. Quels tests unitaires devront être ajoutés ou modifiés ?

Cette revue doit être signalée avant l'implémentation lorsqu'une évolution change l'architecture.
