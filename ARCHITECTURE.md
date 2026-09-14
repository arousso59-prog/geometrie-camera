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
└── Diagnostic camera temporaire
    ├── GrayscaleDiagnostic
    └── GrayscaleDiagnosticApiHandler
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

Le diagnostic caméra n'est volontairement pas ajouté dans cette classe.

### `GrayscaleDiagnostic`

Responsabilité : diagnostic temporaire de la chaîne d'acquisition brute OV3660 -> ESP32-S3.

- reçoit la vraie `ESP32Camera` par injection ;
- demande une frame uniquement sur ordre explicite ;
- accepte uniquement une frame `PIXFORMAT_GRAYSCALE` ;
- copie la frame brute dans un BMP 8 bits non compressé stocké en PSRAM ;
- ne fait aucun traitement géométrique et aucune compression JPEG.

Ce sous-système permet de déterminer si les artefacts observés existent déjà dans les données brutes avant compression JPEG.

**Tests unitaires / tests de composant à prévoir :**

- rejet d'un format autre que GRAYSCALE ;
- validation de la taille minimale du framebuffer ;
- génération correcte de l'en-tête BMP ;
- inversion correcte des lignes pour le format BMP ;
- conservation exacte des valeurs de pixels source.

### `GrayscaleDiagnosticApiHandler`

Responsabilité : HTTP du diagnostic brut uniquement.

Routes temporaires :

```text
GET /diagnostic/capture
GET /diagnostic/status
GET /diagnostic/raw.bmp
```

Cette classe ne connaît pas `CameraManager`, `MeasurementManager` ou les calculs de géométrie.

### `Ov3660CameraConfigurator`

Classe de diagnostic bas niveau créée pour les essais de registres OV3660/PCLK.

Elle n'est plus instanciée dans l'application après l'échec du test de polarité PCLK. Le fichier est conservé temporairement pour tracer et éventuellement réutiliser les essais matériels, sans modifier l'orchestrateur principal.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder une nouvelle fonctionnalité, répondre à ces questions :

1. Quelle est sa responsabilité ?
2. Quelle classe doit la porter ?
3. La classe concernée reste-t-elle cohérente et suffisamment petite ?
4. Faut-il créer une nouvelle classe ou interface ?
5. La logique peut-elle être testée sans ESP32 ni matériel ?
6. Quels tests unitaires devront être ajoutés ou modifiés ?

Cette revue doit être signalée avant l'implémentation lorsqu'une évolution change l'architecture.
