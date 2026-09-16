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
│   ├── TargetDetector (fourni par MeasurementManager)
│   └── TargetDetectionApiHandler
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
  - détection des impulsions vertes fines
  - correction locale des segments verts/noirs
  - pas de flou global
        ↓
GrayFrameView corrigé
        ↓
TargetDetectionService
        ↓
TargetDetector V2
        ↓
TargetObservation
```

Une fois la détection de la cible réelle validée, `MeasurementManager / GeometryMeasurementEngine` utiliseront cette observation pour la distance et l'orientation fine.

## Responsabilités

### `GeometrieCameraApp`

Orchestration uniquement : initialisation, injection de la caméra ESPHome, appel des boucles légères et enregistrement des handlers HTTP.

### `JpegDiagnostic`

Acquisition JPEG native de l'OV5640.

ESPHome conserve une frame pré-acquise. Une demande de capture suit donc volontairement deux étapes :

1. consommer et jeter la frame déjà en attente ;
2. rendre le framebuffer au driver puis demander une nouvelle frame ;
3. publier uniquement cette nouvelle frame.

Le JPEG est copié en PSRAM parce que le framebuffer caméra est éphémère.

**Frontières de test :** première frame non publiée, seconde frame publiée, compteurs purge/capture, SOI/EOI, réutilisation du buffer, changement de résolution.

### `JpegFilteredDiagnostic`

Prépare l'image exploitable par la vision :

- décode le JPEG par blocs avec TJpgDec ;
- écrit directement une luminance 8 bits ;
- applique `JpegArtifactCorrector` ;
- conserve le buffer grayscale corrigé ;
- construit encore un en-tête/palette BMP pour validation visuelle, mais le BMP n'est pas une dépendance du pipeline métier.

Accès métier :

```text
grayscale_data()
grayscale_stride()
width()
height()
```

### `JpegArtifactCorrector`

Algorithme pur de correction du défaut observé sur la voie JPEG : petits segments verts/noirs périodiques. La V3 corrige localement les défauts reconnus par interpolation depuis des pixels sains, sans lisser l'image complète.

**Tests prioritaires :** motifs synthétiques verts/noirs, vrais traits noirs verticaux à préserver, bords de cible, absence d'artefact, différentes résolutions.

### `TargetDetector`

Transforme un `GrayFrameView` non propriétaire en `TargetObservation`. Il reste indépendant d'ESPHome, du JPEG, du HTTP et du stockage d'image.

La V2 est adaptée à la cible réelle observée après la chaîne JPEG corrigée :

- motif 7×7 inchangé ;
- quatre orientations discrètes 0/90/180/270 degrés ;
- contraste minimal abaissé à 12 niveaux car la cible réelle reste reconnaissable malgré un contraste bien plus faible que les images synthétiques ;
- le score est désormais la proportion de cellules correctement classées une fois le contraste minimal validé ;
- balayage global plus grossier puis raffinement local autour du meilleur candidat ;
- taille maximale de recherche limitée à 25 % du petit côté pour éviter les candidats irréalistes ;
- même si le meilleur candidat reste sous le seuil d'acceptation, ses coordonnées/taille/score sont retournés avec `valid=false` pour faciliter le diagnostic.

Le seuil d'acceptation reste à `0.78`. L'orientation fine/perspective sera ajoutée seulement après validation robuste de la cible réelle.

**Tests prioritaires :** cible synthétique, cible réelle à faible contraste, quatre orientations, absence de cible, différentes tailles/résolutions, stabilité des coordonnées et du score, meilleur candidat sous le seuil.

### `TargetDetectionService`

Pont métier très fin entre l'image corrigée et `TargetDetector` :

- vérifie qu'une image filtrée valide est disponible ;
- construit un `GrayFrameView` sans copie ;
- appelle `TargetDetector` ;
- mémorise le dernier `TargetObservation`, le numéro de source et le temps de détection.

Il ne déclenche volontairement ni capture ni filtrage : les trois étapes restent indépendantes pendant la validation fonctionnelle.

**Frontière de test :** injecter une source grayscale connue et un détecteur, vérifier propagation du résultat et rejet d'une source indisponible.

### `TargetDetectionApiHandler`

Expose uniquement la validation de cible :

```text
GET /target/detect
GET /target/status
```

`/target/detect` traite la dernière image filtrée ; `/target/status` relit le dernier résultat sans retraitement. `target_found` indique l'acceptation finale, tandis que le bloc `target` expose le meilleur candidat disponible même si celui-ci reste sous le seuil.

### `MeasurementManager`

Enchaînera `TargetDetector` puis `GeometryMeasurementEngine` une fois la cible réelle validée. Il conserve déjà la dernière mesure et le compteur de mesures valides.

### `GeometryMeasurementEngine`

Calcul mathématique de la mesure à partir de l'observation et de la calibration. Il doit rester indépendant du matériel et du réseau. Distance/orientation/calibration seront complétées après validation de la cible réelle.

### `CameraResolutionController`

Maintient l'identité capteur et les résolutions supportées. Capteur confirmé : OV5640 PID `0x5640`, matrice physique 2592×1944 ; ESPHome 2026.7.3 expose QSXGA 2560×1920 comme mode maximal.

### `CameraSettingsController` / `CameraSettingsApiHandler`

Conservés car exposition, gain, luminosité et contraste seront utiles lors de la validation cible et de la calibration.

```text
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>
```

### `RuntimeDiagnostics` / `RuntimeDiagnosticsApiHandler`

Instrumentation légère de la boucle et de la mémoire. Conservée pour les futures optimisations de temps de traitement.

```text
GET /api/runtime/status
```

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
```

## Sous-systèmes supprimés après validation

Le nettoyage suivant est volontaire : ces composants correspondaient à des branches d'essai qui ne font plus partie de la voie retenue.

- `GrayscaleDiagnostic` et son API ;
- `Rgb565Diagnostic` et son API ;
- ancien `TargetSearchDiagnostic` GRAYSCALE et son API ;
- `Ov5640TimingController` / API de registres ;
- `Ov3660CameraConfigurator` ;
- `ImageProvider`, `PlaceholderImageProvider`, image placeholder ;
- `CameraManager` historique ;
- `CameraApiHandler` historique (`/api/status`, `/api/capture`, `/api/measure`, `/image.jpg`).

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

Les essais de timing ont servi à isoler le problème mais ne font plus partie du firmware : XCLK 20/16/10/8 puis runtime 7/6/5 MHz, PCLK divider, HTS/VTS, HREF blanking et JPEG mode 2/3 n'ont pas supprimé le motif parasite. La correction logicielle V3 est la voie retenue.

## Optimisations reportées après validation cible/distance/orientation

Ne pas optimiser prématurément la voie actuelle. Après validation fonctionnelle :

1. première recherche sur l'image complète ;
2. mémorisation de la boîte cible ;
3. pour les mesures suivantes, travailler sur une ROI autour de la dernière cible ;
4. limiter correction et recherche à cette ROI ;
5. étudier aussi le décodage JPEG partiel/par blocs pour éviter le coût pleine image ;
6. si la cible est perdue ou le score devient insuffisant, revenir automatiquement à une recherche globale.

## Revue obligatoire avant nouvelle fonctionnalité

Avant de coder :

1. quelle est sa responsabilité ?
2. quelle classe doit la porter ?
3. la classe reste-t-elle cohérente ?
4. faut-il une nouvelle interface ?
5. quelle logique peut être testée sans ESP32 ?
6. quels tests doivent être ajoutés ?
7. si l'API change, `/api/wsdl` a-t-il été modifié dans le même changement ?
