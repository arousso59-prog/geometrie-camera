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

Diagnostic bas niveau du timing DVP, du timing de trame, de XCLK et de quelques registres de sortie JPEG spécifiques au vrai capteur OV5640.

Le contrôleur :

- refuse les opérations matérielles si le PID n'est pas `OV5640` ;
- lit la fréquence XCLK nominale connue du driver via `sensor_t::xclk_freq_hz` ;
- peut demander dynamiquement `XCLK=5..8 MHz` via le callback `sensor_t::set_xclk`, ce qui permet de tester sous la limite YAML ESPHome de 8 MHz sans reflasher ;
- lit et peut modifier le registre `PCLK_RATIO` `0x3824` ;
- lit `VFIFO_CTRL0C` `0x460C` pour vérifier que le PCLK manuel est actif ;
- lit `HTS` via `0x380C/0x380D` et `VTS` via `0x380E/0x380F` ;
- lit et peut modifier, dans une liste blanche stricte, `JPEG mode` `0x4713` et `DVP HREF control` `0x471F` ;
- limite `jpeg_mode` aux valeurs de test 2 ou 3 ;
- permet de modifier XCLK/HTS/VTS/JPEG mode/HREF blanking avec contrôle immédiat ;
- mémorise automatiquement XCLK/HTS/VTS/JPEG/HREF avant la première modification d'une série de tests ;
- peut restaurer cette référence sans reflasher ;
- n'expose pas d'écriture arbitraire vers les autres registres du capteur ;
- n'effectue aucune capture et ne traite aucune image.

Routes temporaires de mise au point :

```text
GET /api/camera/timing
GET /api/camera/timing/set?xclk_mhz=<5..8>&pclk_divider=<optionnel>&hts=<optionnel>&vts=<optionnel>&jpeg_mode=<2|3>&href_blanking=<0..255>
GET /api/camera/timing/restore
```

Les valeurs numériques peuvent être données en décimal ou en notation `0x...` lorsque cela est pertinent.

**Important :** un changement de `framesize` peut reprogrammer plusieurs paramètres du driver. Pour un test reproductible : choisir d'abord la résolution, lire la référence, appliquer XCLK ou les paramètres de registre, puis effectuer les captures sans changer de résolution. Après un changement de XCLK, `JpegDiagnostic` purge de toute façon la frame pré-acquise avant de publier la frame fraîche suivante. La restauration mémorisée est destinée à cette même série de tests.

Les essais PCLK 4/8/10, XCLK YAML 20/16/10/8 MHz, HTS/VTS, HREF blanking 0x40/0x60/0x80/0xC0 et JPEG mode 2/3 n'ont pas supprimé les lignes vertes. Le test actif consiste à descendre XCLK dynamiquement à 7, 6 puis 5 MHz, sans modifier le reste de la configuration.

**Tests à prévoir :** rejet d'un PID différent, absence de callback `set_xclk`, validation de la plage XCLK 5..8, validation des autres plages, capture unique de la référence, vérification des readbacks HTS/VTS/PCLK/JPEG/HREF, restauration XCLK et registres, sérialisation HTTP cohérente. L'accès réel au capteur reste un test d'intégration matériel tant qu'il n'est pas abstrait derrière une interface capteur.

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

Cette purge est également nécessaire après un changement de `framesize` ou de XCLK, car la frame pré-acquise peut encore correspondre aux paramètres précédents.

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

**But du test courant :** conserver la bonne qualité générale et le faible bruit du JPEG natif tout en supprimant les lignes vertes périodiques. Les essais PCLK, XCLK jusqu'à 8 MHz, HTS/VTS, HREF et JPEG mode 2/3 n'ont pas supprimé le défaut. La piste active est un test XCLK runtime à 7, 6 et 5 MHz, valeurs non acceptées directement par le validateur YAML ESPHome sous 8 MHz.

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

Le YAML reste à 8 MHz pour démarrer dans une configuration acceptée par ESPHome. Les essais à 7/6/5 MHz sont appliqués uniquement à chaud par le contrôleur de timing et disparaissent au redémarrage.

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