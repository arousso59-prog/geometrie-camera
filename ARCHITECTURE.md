# Architecture du projet `geometrie-camera`

## Règles de développement

Ces règles s'appliquent aux nouveaux développements du projet.

1. **Les fichiers `.h` sont déclaratifs uniquement.**
   - déclarations de classes, structures, enums et méthodes ;
   - pas de corps de méthode significatif dans les headers ;
   - constructeurs, destructeurs, getters et setters dans les `.cpp`.

2. **Une classe = une responsabilité principale.**
   - une classe qui commence à gérer plusieurs responsabilités doit être découpée ;
   - `GeometrieCameraApp` reste un orchestrateur et ne contient ni algorithme vision, ni calcul géométrique, ni sérialisation HTTP métier.

3. **Avant une nouvelle fonction, revoir l'architecture.**
   - identifier la responsabilité ;
   - décider si elle appartient à une classe existante ;
   - sinon créer une classe ou un sous-système dédié ;
   - éviter les dépendances vers l'application complète.

4. **Penser chaque évolution avec les tests.**
   - séparer logique pure et matériel ESP32 ;
   - garder des frontières testables ;
   - utiliser injection/interface lorsque cela devient utile.

5. **Toute évolution de l'API HTTP met à jour `GET /api/wsdl` dans le même changement.**
   - nouvelle route, suppression, méthode HTTP, paramètre ou comportement contractuel : mise à jour du WSDL-like ;
   - chaque méthode conserve un commentaire fonctionnel et les valeurs autorisées connues.

## Architecture actuelle

```text
GeometrieCameraApp
├── PlaceholderImageProvider : ImageProvider
├── CameraManager
├── MeasurementManager
│   ├── TargetDetector
│   └── GeometryMeasurementEngine
├── CameraResolutionController
├── CameraSettingsController
│   └── CameraSettingsApiHandler
├── Ov5640TimingController
│   └── Ov5640TimingApiHandler
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
├── ApiWsdlHandler
├── CameraApiHandler
├── Ov3660CameraConfigurator
└── Diagnostics camera temporaires
    ├── GrayscaleDiagnostic
    │   └── GrayscaleDiagnosticApiHandler
    ├── JpegDiagnostic
    │   └── JpegDiagnosticApiHandler
    ├── Rgb565Diagnostic
    │   └── Rgb565DiagnosticApiHandler
    └── TargetSearchDiagnostic
        └── TargetSearchDiagnosticApiHandler
```

## Responsabilités principales

### `GeometrieCameraApp`

Orchestration ESPHome uniquement : initialise les sous-systèmes, injecte la vraie `ESP32Camera`, enregistre les handlers HTTP et appelle les boucles légères.

### `ImageProvider` / `CameraManager`

`ImageProvider` abstrait une source d'image. `CameraManager` pilote le cycle historique d'acquisition et conserve les métadonnées de la dernière image. Le `PlaceholderImageProvider` reste un bouchon de développement.

**Frontière de test :** `CameraManager` doit pouvoir être testé avec un faux `ImageProvider`.

### `MeasurementManager`

Chaîne de mesure : demande une observation à `TargetDetector`, puis le calcul à `GeometryMeasurementEngine`, conserve la dernière mesure et le compteur de mesures valides.

### `TargetDetector`

Transforme un `GrayFrameView` non propriétaire en observation de cible. Il reste indépendant d'ESPHome, du HTTP et du stockage d'image. La cible actuelle est le motif 7×7 asymétrique.

**Tests prioritaires :** cible synthétique, quatre orientations, absence de cible, contraste insuffisant, différentes tailles/résolutions, stabilité des coordonnées et du score.

Si l'algorithme grossit, le découpage prévu pourra faire apparaître des classes comme `ThresholdProcessor`, `ContourDetector`, `CornerExtractor`, `TargetValidator` et `SubpixelRefiner`.

### `GeometryMeasurementEngine`

Calcul mathématique des angles à partir d'une observation et d'une calibration. Cette classe doit rester indépendante du matériel et du réseau.

### `CameraResolutionController`

Lit le PID, expose l'identité du capteur, maintient la résolution active et applique les changements de `framesize` autorisés.

Le capteur confirmé est un OV5640 (PID `0x5640`) physiquement 2592×1944. ESPHome 2026.7.3 expose actuellement au maximum le mode QSXGA 2560×1920.

### `CameraSettingsController` / `CameraSettingsApiHandler`

Lecture et application des réglages capteur : brightness, contrast, exposition automatique/manuelle et gain automatique/manuel. La logique matérielle est séparée du parsing HTTP.

Routes :

```text
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>
```

### `Ov5640TimingController` / `Ov5640TimingApiHandler`

Diagnostic bas niveau du timing DVP et du timing de trame spécifique au vrai capteur OV5640.

Le contrôleur :

- refuse toute écriture si le PID n'est pas `OV5640` ;
- lit et peut modifier le registre `PCLK_RATIO` `0x3824` ;
- lit `VFIFO_CTRL0C` `0x460C` pour vérifier que le PCLK manuel est actif ;
- lit `HTS` via `0x380C/0x380D` et `VTS` via `0x380E/0x380F` ;
- permet de modifier HTS et VTS avec relecture immédiate de contrôle ;
- mémorise automatiquement les valeurs HTS/VTS courantes avant la première modification d'une série de tests ;
- peut restaurer cette référence sans reflasher ;
- n'expose pas d'écriture arbitraire vers les autres registres du capteur ;
- n'effectue aucune capture et ne traite aucune image.

Routes temporaires de mise au point :

```text
GET /api/camera/timing
GET /api/camera/timing/set?pclk_divider=<1..31>&hts=<optionnel>&vts=<optionnel>
GET /api/camera/timing/restore
```

Les valeurs `hts` et `vts` peuvent être données en décimal ou en notation `0x...`.

**Important :** un changement de `framesize` peut reprogrammer les registres de timing du driver. Pour un test reproductible : choisir d'abord la résolution, lire le timing de référence, appliquer HTS/VTS, puis effectuer les captures sans changer de résolution. La restauration mémorisée est destinée à cette même série de tests.

Le test HTS/VTS actuel est motivé par le correctif expérimental Espressif/OV5640 connu pour les lignes verticales : commencer par `HTS=0x1FFF`, puis, uniquement si nécessaire, ajouter `VTS=0x2FFF`.

**Tests à prévoir :** rejet d'un PID différent, échec si accès registres absent, validation des plages, capture unique de la référence, vérification des readbacks HTS/VTS/PCLK, restauration de la référence et sérialisation HTTP cohérente. L'accès réel aux registres reste un test d'intégration matériel tant qu'il n'est pas abstrait derrière une interface capteur.

### `RuntimeDiagnostics` / `RuntimeDiagnosticsApiHandler`

Instrumentation légère des intervalles entre passages de la boucle ESPHome et de la mémoire interne/PSRAM disponible. Aucun traitement d'image n'est effectué ici.

Route :

```text
GET /api/runtime/status
```

Le maximum d'intervalle est conservé depuis le démarrage pour détecter les blocages provoqués par les callbacks image.

### `GrayscaleDiagnostic`

Diagnostic de la voie brute `PIXFORMAT_GRAYSCALE`.

- demande une frame uniquement sur ordre explicite ;
- calcule les statistiques brutes ;
- peut produire un BMP pleine résolution lorsque la mémoire le permet ;
- produit un preview réduit à haute résolution ;
- le framebuffer caméra reste la source des futurs calculs, le BMP ne sert qu'à l'affichage.

Routes :

```text
GET /diagnostic/capture?resolution=<optionnel>
GET /diagnostic/status
GET /diagnostic/raw.bmp
GET /diagnostic/preview.bmp
```

Ces routes d'acquisition exigent que la caméra soit configurée en GRAYSCALE.

### `JpegDiagnostic`

Diagnostic dédié au JPEG natif produit directement par l'ISP de l'OV5640. Cette classe ne décode ni ne recompresse l'image.

ESPHome maintient une frame pré-acquise dans sa tâche caméra. Avec un framebuffer unique et une acquisition sur demande, cette frame peut être antérieure à la requête HTTP. `JpegDiagnostic` utilise donc un cycle en deux temps :

1. la première frame reçue est volontairement purgée et n'est jamais publiée ;
2. une seconde demande est lancée depuis `GeometrieCameraApp::loop()` après retour du callback ;
3. seule cette seconde frame est copiée dans le buffer JPEG persistant et devient l'image publiée.

Cette purge est également nécessaire après un changement de `framesize`, car la frame pré-acquise peut encore avoir l'ancienne résolution.

- accepte uniquement `PIXFORMAT_JPEG` ;
- copie le JPEG natif dans un buffer persistant, de préférence en PSRAM, car le framebuffer caméra est éphémère ;
- conserve dimensions et taille ;
- vérifie les marqueurs JPEG SOI (`FF D8`) et EOI (`FF D9`) pour repérer une troncature grossière ;
- mesure séparément demande initiale, purge, demande fraîche, acquisition fraîche, copie et cycle total ;
- compte séparément les frames utiles et les frames purgées.

Routes :

```text
GET /diagnostic-jpeg/capture?resolution=<optionnel>
GET /diagnostic-jpeg/status
GET /diagnostic-jpeg/image.jpg
```

**But du test courant :** conserver la bonne qualité générale et le faible bruit du JPEG natif tout en supprimant les lignes vertes périodiques. Les essais de diviseur PCLK 4/8/10 et de XCLK 20/16/10/8 MHz n'ont pas supprimé le défaut. La piste active est maintenant HTS/VTS, d'après le correctif expérimental connu sur OV5640.

**Tests à prévoir :** rejet d'un format non JPEG, première frame non publiée, seconde frame publiée, compteur utile/purge, changement de résolution suivi d'une purge, copie exacte d'un buffer connu, détection SOI/EOI, réutilisation/allocation du buffer, état en cas d'échec mémoire.

### `Rgb565Diagnostic`

Diagnostic temporaire couleur `PIXFORMAT_RGB565`, séparé des autres formats. Il convertit la frame reçue en BMP uniquement pour inspection.

Routes :

```text
GET /diagnostic-rgb565/capture
GET /diagnostic-rgb565/status
GET /diagnostic-rgb565/raw.bmp
```

### `TargetSearchDiagnostic`

Diagnostic de la chaîne de recherche GRAYSCALE : demande une frame, transmet directement le framebuffer à `TargetDetector`, mesure acquisition/détection/visualisation et produit une image de contrôle.

Routes :

```text
GET /target/search?resolution=<optionnel>
GET /target/status
GET /target/image.bmp
```

Il n'est pas utilisé pendant le test JPEG natif actuel.

### `ApiWsdlHandler`

Publie le contrat descriptif de toutes les routes HTTP :

```text
GET /api/wsdl
```

Le document reste un catalogue REST WSDL-like et non un service SOAP.

### `Ov3660CameraConfigurator`

Ancien outil d'essai bas niveau spécifique OV3660/PCLK. Le capteur réel étant désormais confirmé OV5640, cette classe reste inactive tant qu'aucun diviseur n'est demandé. Elle sera renommée ou supprimée uniquement lors d'un refactoring ciblé, pas pendant la validation JPEG.

## État courant de la voie caméra

Le YAML est temporairement configuré en :

```text
OV5640
2560×1920 QSXGA
PIXFORMAT_JPEG
jpeg_quality = 10
XCLK = 8 MHz
1 framebuffer en PSRAM
idle_framerate = 0
```

Cette configuration sert uniquement à valider la piste JPEG native. Le code GRAYSCALE et RGB565 reste présent pour permettre un retour rapide sans réécriture.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder une nouvelle fonctionnalité, vérifier :

1. Quelle est sa responsabilité ?
2. Quelle classe doit la porter ?
3. La classe reste-t-elle cohérente et suffisamment petite ?
4. Faut-il créer une nouvelle classe ou interface ?
5. La logique peut-elle être testée sans ESP32 ni matériel ?
6. Quels tests doivent être ajoutés ou modifiés ?
7. Si l'API HTTP change, `GET /api/wsdl` a-t-il été mis à jour dans le même changement ?