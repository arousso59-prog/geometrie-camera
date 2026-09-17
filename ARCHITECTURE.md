# Architecture du projet `geometrie-camera`

## Règles de développement

1. **Les fichiers `.h` sont déclaratifs uniquement.** Les implémentations et données significatives restent dans les `.cpp`.
2. **Une classe = une responsabilité principale.** `GeometrieCameraApp` reste un orchestrateur.
3. **Avant toute nouvelle fonction, revoir l'architecture** et choisir explicitement la classe responsable.
4. **Penser chaque évolution avec les tests** : logique pure séparée du matériel, dépendances injectées.
5. **Toute évolution de l'API HTTP met à jour `GET /api/wsdl` dans le même changement.**
6. **Les gros buffers image/vision sont placés en PSRAM et réutilisés.**
7. **Le PC configure et observe ; l'ESP32 orchestre les cycles temps réel.** Le PC ne déplace jamais la ROI à chaque image.

## Architecture actuelle

```text
GeometrieCameraApp
├── CameraResolutionController
├── CameraSettingsController
│   └── CameraSettingsApiHandler
├── CameraViewportController
├── TargetTrackingController
│   └── TargetTrackingApiHandler
├── JpegDiagnostic
│   └── JpegDiagnosticApiHandler
├── ImageSharpnessEvaluator
├── JpegFilteredDiagnostic
│   ├── JpegArtifactCorrector
│   └── JpegFilteredDiagnosticApiHandler
├── TargetDetector V5.5
│   ├── TargetCandidateFinder
│   ├── TargetCornerRefiner
│   └── TargetCodeDecoder
├── TargetDetectionService
│   └── TargetDetectionApiHandler
├── TargetDetectionPreview
├── MeasurementManager
│   └── GeometryMeasurementEngine V3
├── MeasurementApiHandler
├── ContinuousMeasurementController
│   └── ContinuousMeasurementApiHandler
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
└── ApiWsdlHandler
```

## Tracking haute résolution

### `CameraViewportController`

Responsabilité unique : piloter le cadrage du capteur et convertir les coordonnées locales vers un repère stable.

Repère de référence actuel :

```text
2560 × 1920
```

Deux viewports sont supportés :

```text
SEARCH
  fenêtre référence : 2560×1920
  sortie             : 800×600
  scale              : 3,2 × 3,2

PRECISE
  fenêtre référence : 800×600 centrée sur la cible
  sortie             : 800×600
  scale              : 1 × 1
```

En PRECISE, le crop est programmé directement dans l'OV5640 via `sensor_t::set_res_raw()`. Le filtre ne reçoit donc jamais un grayscale 2560×1920 complet.

La conversion générale est :

```text
x_ref = roi_x + (x_local + 0,5) × scale_x - 0,5
y_ref = roi_y + (y_local + 0,5) × scale_y - 0,5
```

Les quatre coins, le centre et les dimensions de `TargetObservation` sont convertis avant le calcul géométrique.

### `TargetTrackingController`

Responsabilité unique : décider **quand** changer de viewport.

Configuration actuelle :

```text
enabled                 false par défaut pendant validation
lost_cycles             3
recenter_threshold_pct  70
```

Automate :

```text
SEARCH
  ↓ cible valide
PRECISE
  ├── cible proche du bord → recentrage PRECISE
  ├── cible valide         → rester PRECISE
  └── cible perdue N fois  → SEARCH
```

Le contrôleur ne réalise ni capture, ni filtre, ni détection, ni mesure.

### Changement de viewport

Un changement de viewport ne doit jamais modifier le repère pendant le calcul du cycle en cours :

```text
capture
→ filtre
→ détection dans viewport courant
→ conversion vers repère 2560×1920
→ mesure
→ éventuel changement de viewport
→ cycle suivant
```

Après changement de viewport :

- l'historique temporel du `TargetDetector` est remis à zéro ;
- la ROI de netteté est invalidée ;
- la référence de netteté est remise à zéro ;
- `JpegDiagnostic` purge la frame éventuellement pré-acquise avant de demander la frame fraîche suivante.

## Chaîne image et mesure

```text
OV5640 JPEG
   ↓
CameraViewportController
   ↓ sortie 800×600
JpegDiagnostic
   ↓
ImageSharpnessEvaluator
   ↓
JpegFilteredDiagnostic V2
   ├── JPEG -> luminance 8 bits
   ├── masque vert
   └── JpegArtifactCorrector
   ↓
TargetDetector V5.5
   ↓ TargetObservation locale
CameraViewportController::to_reference()
   ↓ TargetObservation 2560×1920
GeometryMeasurementEngine V3
```

## `TargetDetector V5.5`

Le détecteur orchestre localisation, raffinement des coins et validation du motif 7×7.

Pour sélectionner entre plusieurs candidats valides, il ajoute une cohérence temporelle de position et de taille. Cette continuité est remise à zéro lors d'un changement SEARCH/PRECISE, car les coordonnées locales et la taille apparente changent brutalement.

Preview :

```text
cadre plein      cible validée
cadre pointillé  meilleur candidat localisé mais rejeté
```

## `GeometryMeasurementEngine V3`

La distance conserve les estimations indépendantes :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
```

`z_mm` les fusionne de façon continue : moyenne harmonique si elles sont proches, transition progressive vers la plus petite estimation entre 3 % et 15 % de désaccord.

Le centre utilisé pour X/Y et les bearings est l'intersection des diagonales des quatre coins raffinés.

L'homographie pilote l'orientation du plan et fournit `pose_z_mm` comme contrôle indépendant de cohérence.

## Calibration

`CameraCalibration` mémorise `fx/fy/cx/cy` ainsi que sa résolution de référence. `effective_calibration()` redimensionne ces paramètres vers la résolution logique utilisée par le calcul.

En mode tracking, la mesure est toujours appelée avec le repère logique 2560×1920. Une calibration plein champ réalisée à 800×600 reste donc redimensionnable ; après validation du tracking, la procédure de référence pourra être standardisée en 2560×1920.

## `ContinuousMeasurementController`

Dépendances :

```text
JpegDiagnostic
ImageSharpnessEvaluator
JpegFilteredDiagnostic
TargetDetectionService
MeasurementManager
TargetTrackingController
```

Il orchestre :

```text
REQUEST_CAPTURE
→ WAIT_CAPTURE
→ SHARPNESS
→ FILTER
→ DETECT
→ COMPUTE
→ mise à jour tracking/viewport
→ WAIT_INTERVAL
```

Aucun cycle n'est empilé.

Le snapshot de timing publié correspond toujours au dernier cycle terminé.

## Mémoire image

Le principe retenu est de **ne pas décoder une image grayscale 2560×1920 complète**.

Ordres de grandeur :

```text
grayscale 800×600    ≈ 0,48 Mo
grayscale 2560×1920  ≈ 4,92 Mo
```

Avec 8 Mo de PSRAM, le plein format laisserait trop peu de marge pour JPEG, masque, framebuffer et workspaces. Le mode PRECISE conserve une sortie 800×600 et gagne en précision par crop natif du capteur.

## API tracking

```text
GET /api/camera/viewport
GET /tracking/config
GET /tracking/config/set?enabled=<0|1>&lost_cycles=<1..10>&recenter_threshold_pct=<50..90>
GET /tracking/status
```

`/continuous/status` expose également un bloc `tracking` pour éviter un polling supplémentaire depuis la supervision PC.

`GET /api/wsdl` est la référence du contrat HTTP. Version actuelle : **21**.

## Validation immédiate

1. compiler et flasher avec tracking désactivé ;
2. vérifier le pipeline historique ;
3. vérifier `/tracking/status` avec `supported=true` ;
4. activer le tracking ;
5. valider SEARCH → PRECISE ;
6. confirmer que le JPEG reste réellement 800×600 en PRECISE ;
7. vérifier le grossissement apparent de la cible et la stabilité des coins ;
8. mesurer `capture_ms`, `filter_ms`, `detect_ms`, `cycle_ms` ;
9. tester recentrage puis perte de cible ;
10. seulement après validation, optimiser les timings bruts OV5640 et supprimer éventuellement les traitements devenus inutiles en N/B.
