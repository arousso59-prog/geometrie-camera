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

5. **Toute évolution de l'API HTTP doit mettre à jour le WSDL-like dans le même changement.**
   - l'endpoint de référence est `GET /api/wsdl` ;
   - toute nouvelle route, suppression de route, modification de méthode HTTP ou ajout/suppression/modification de paramètre doit être reporté dans `ApiWsdlHandler` ;
   - chaque méthode doit conserver un commentaire fonctionnel, à enrichir au fur et à mesure de la réalisation ;
   - les valeurs autorisées d'un paramètre doivent être exposées quand elles sont connues ;
   - une évolution API n'est pas considérée terminée tant que le descripteur n'est pas cohérent avec le code.

## Architecture actuelle

```text
GeometrieCameraApp
├── PlaceholderImageProvider : ImageProvider
├── CameraManager
├── MeasurementManager
│   ├── TargetDetector
│   └── GeometryMeasurementEngine
├── ApiWsdlHandler
├── CameraApiHandler
├── CameraResolutionController
├── CameraSettingsController
│   └── CameraSettingsApiHandler
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
├── Ov3660CameraConfigurator
└── Diagnostics camera temporaires
    ├── GrayscaleDiagnostic
    │   └── GrayscaleDiagnosticApiHandler
    ├── Rgb565Diagnostic
    │   └── Rgb565DiagnosticApiHandler
    └── TargetSearchDiagnostic
        └── TargetSearchDiagnosticApiHandler
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

La première implémentation réelle reconnaît le repère 7×7 imprimé pour le prototype : bord noir continu et motif intérieur asymétrique. La recherche est multi-échelle et identifie pour l'instant les orientations discrètes 0/90/180/270°.

Le détecteur travaille uniquement sur un `GrayFrameView` non propriétaire : il ne copie pas le framebuffer caméra et reste indépendant d'ESPHome, du réseau et de l'affichage.

La plage de taille de cible est adaptée à la résolution reçue. Le minimum conserve un plancher compatible avec le motif 7×7 et le maximum actuel vaut 50 % du petit côté de l'image.

**Tests unitaires prioritaires :**

- cible synthétique connue au centre ;
- cible aux quatre orientations ;
- cible absente ;
- contraste insuffisant ;
- plusieurs échelles et plusieurs résolutions d'image ;
- stabilité des coordonnées et du score de qualité.

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

### `ApiWsdlHandler`

Responsabilité : publier le contrat descriptif de l'API HTTP.

Route :

```text
GET /api/wsdl
```

Le document est un catalogue XML **WSDL-like** : le projet reste une API REST/HTTP et non un service SOAP. Il liste les routes disponibles, méthodes HTTP, paramètres, valeurs autorisées connues, types de réponse et commentaires fonctionnels.

`ApiWsdlHandler` ne déclenche aucune acquisition et ne contient aucune logique métier. Il dépend uniquement de `CameraResolutionController` pour publier dynamiquement la liste courante des résolutions autorisées.

**Règle de maintenance :** toute modification d'une API doit modifier ce descripteur dans le même changement Git.

### `CameraApiHandler`

Responsabilité : interface HTTP de l'application uniquement.

- routage des URL `/api/*` et `/image.jpg` ;
- transformation des états en réponses HTTP/JSON ;
- transmission de l'image fournie par `CameraManager`.

Elle dépend directement de `CameraManager` et `MeasurementManager`, pas de toute l'application.

Les diagnostics caméra ne sont volontairement pas ajoutés dans cette classe.

### `CameraResolutionController`

Responsabilité : connaître le capteur caméra actif et piloter les changements de résolution à chaud.

- lit le PID du capteur via le driver Espressif ;
- expose le nom du capteur et sa résolution maximale connue ;
- maintient la résolution active ;
- valide les résolutions autorisées avant application ;
- applique `set_framesize()` au capteur.

### `CameraSettingsController`

Responsabilité : lire et appliquer les réglages d'acquisition du capteur, indépendamment du HTTP et du traitement d'image.

Réglages actuellement exposés :

- `brightness` ;
- `contrast` ;
- activation de l'exposition automatique `exposure_ctrl` ;
- compensation automatique `ae_level` ;
- exposition manuelle `aec_value` ;
- activation du gain automatique `gain_ctrl` ;
- gain manuel `agc_gain`.

Le contrôleur valide les plages avant d'appeler les fonctions du driver Espressif. Il ne modifie ni le framebuffer ni l'image BMP de diagnostic.

**Tests à prévoir :** validation des plages, rejet des valeurs invalides, comportement sans capteur, lecture d'un snapshot cohérent et application d'un réglage avec un adaptateur capteur simulé si l'accès matériel est abstrait ultérieurement.

### `CameraSettingsApiHandler`

Responsabilité : exposer les réglages d'acquisition par HTTP pendant la phase de mise au point.

Routes :

```text
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>
```

La route `set` est volontairement une route GET temporaire cohérente avec l'API de développement actuelle. Lors de la stabilisation future de l'API v1, elle pourra devenir une opération REST de type `PUT` ou `PATCH` avec corps JSON.

### `RuntimeDiagnostics`

Responsabilité : instrumentation légère de la réactivité de la boucle ESPHome et de la mémoire disponible.

- `GeometrieCameraApp::loop()` appelle uniquement `record_loop()` ;
- mesure l'intervalle courant, moyen et maximal entre deux passages de la boucle applicative ;
- expose la mémoire interne libre et son plus gros bloc ;
- expose la PSRAM libre et son plus gros bloc ;
- ne déclenche aucune capture et ne traite aucune image.

Le maximum d'intervalle est conservé depuis le démarrage afin de mettre en évidence les blocages provoqués par une capture ou une recherche de cible.

**Tests à prévoir :** calcul de moyenne/max sur une source temporelle injectable si le diagnostic devient permanent. Les lectures de heap restent des tests d'intégration ESP32.

### `RuntimeDiagnosticsApiHandler`

Responsabilité : sérialisation HTTP du diagnostic d'exécution uniquement.

Route :

```text
GET /api/runtime/status
```

La réponse expose les statistiques de boucle, la mémoire disponible et les drapeaux `capture_pending` / `target_search_pending`. Le handler dépend directement de `RuntimeDiagnostics`, `GrayscaleDiagnostic` et `TargetSearchDiagnostic`, pas de toute l'application.

### `GrayscaleDiagnostic`

Responsabilité : acquisition brute et stockage de l'unique image de diagnostic GRAYSCALE.

- reçoit la vraie `ESP32Camera` par injection ;
- demande une frame uniquement sur ordre explicite ;
- accepte uniquement une frame `PIXFORMAT_GRAYSCALE` ;
- copie la frame brute dans un BMP 8 bits non compressé stocké en PSRAM lorsque la résolution le permet ;
- à haute résolution, conserve uniquement un preview réduit afin de ne pas dupliquer plusieurs mégaoctets en PSRAM ;
- calcule quelques statistiques brutes pour vérifier la dynamique reçue ;
- expose ce même buffer BMP au diagnostic de recherche de cible afin d'éviter un second buffer image persistant ;
- sait réserver une entrée de palette et tracer un cadre vert directement dans le BMP existant.

Le framebuffer caméra reste la source de calcul. Le BMP n'est qu'une visualisation HTTP et n'est jamais réinjecté dans l'algorithme de détection.

**Tests unitaires / tests de composant à prévoir :**

- rejet d'un format autre que GRAYSCALE ;
- validation de la taille minimale du framebuffer ;
- génération correcte de l'en-tête BMP ;
- conservation exacte des valeurs de pixels source en mode brut ;
- réservation de la couleur d'overlay sans duplication du buffer ;
- tracé correct du cadre vert ;
- statistiques min/max/moyenne sur un tableau connu.

### `GrayscaleDiagnosticApiHandler`

Responsabilité : HTTP du diagnostic GRAYSCALE uniquement.

Routes temporaires :

```text
GET /diagnostic/capture?resolution=<optionnel>
GET /diagnostic/status
GET /diagnostic/raw.bmp
GET /diagnostic/preview.bmp
```

### `TargetSearchDiagnostic`

Responsabilité : piloter une acquisition destinée à la recherche de cible et mesurer les temps de la chaîne complète.

- demande une nouvelle frame à `ESP32Camera` ;
- transmet directement le framebuffer GRAYSCALE à `TargetDetector` sans copie ;
- mesure séparément acquisition, détection, préparation de visualisation et cycle total ;
- demande à `GrayscaleDiagnostic` de remplacer l'unique BMP de visualisation ;
- si la cible est trouvée, demande le tracé d'un cadre vert dans ce même BMP ;
- ne possède aucun second buffer image.

Cette classe prépare la future stratégie à deux régimes : recherche pleine image pour l'accrochage initial, puis recherche dans une ROI autour de la dernière position pour atteindre une cadence élevée en suivi.

### `TargetSearchDiagnosticApiHandler`

Routes de validation de la chaîne cible :

```text
GET /target/search?resolution=<optionnel>
GET /target/status
GET /target/image.bmp
```

`/target/status` expose notamment `target_found`, la boîte détectée, l'orientation discrète, le score de qualité et les temps `acquisition_ms`, `detection_ms`, `visualization_ms` et `total_cycle_ms`.

### `Rgb565Diagnostic`

Responsabilité : diagnostic temporaire de la chaîne d'acquisition brute couleur en `PIXFORMAT_RGB565`.

- reçoit la même `ESP32Camera` par injection ;
- demande une frame uniquement sur ordre explicite ;
- refuse tout format autre que RGB565 ;
- vérifie que la taille source vaut au moins `largeur × hauteur × 2` ;
- transforme la frame RGB565 en BMP 24 bits non compressé avec le convertisseur du driver Espressif ;
- conserve uniquement le BMP et ses métadonnées pour inspection HTTP.

Ce diagnostic reste séparé de `GrayscaleDiagnostic` afin de conserver l'historique et les outils des essais matériels.

### `Rgb565DiagnosticApiHandler`

Routes temporaires :

```text
GET /diagnostic-rgb565/capture
GET /diagnostic-rgb565/status
GET /diagnostic-rgb565/raw.bmp
```

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
7. Si l'API HTTP change, le descripteur `GET /api/wsdl` a-t-il été mis à jour dans le même changement ?

Cette revue doit être signalée avant l'implémentation lorsqu'une évolution change l'architecture.
