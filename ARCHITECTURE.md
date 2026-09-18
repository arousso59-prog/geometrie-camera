# Architecture du projet `geometrie-camera`

## Principes

1. Les fichiers `.h` restent déclaratifs ; les implémentations sont dans les `.cpp`.
2. Une classe conserve une responsabilité principale.
3. `GeometrieCameraApp` orchestre les composants mais ne porte pas les algorithmes.
4. Les gros buffers image sont alloués en PSRAM et réutilisés.
5. Toute évolution de l'API met à jour `GET /api/wsdl`.
6. Le PC déclenche les trois workflows opérationnels ; l'ESP32 orchestre acquisition, tracking et calcul.
7. Le tracking haute précision est permanent et n'est pas un réglage utilisateur.

## Architecture actuelle

```text
GeometrieCameraApp
├── CameraResolutionController
├── CameraSettingsController          # usage interne calibration
├── CameraViewportController
├── TargetTrackingController
│   └── TargetTrackingApiHandler      # lecture seule
├── JpegDiagnostic                    # acquisition interne
├── JpegFilteredDiagnostic            # décodage JPEG -> gris uniquement
├── TargetDetector V6.1
│   ├── TargetCandidateFinder
│   ├── TargetCornerRefiner
│   ├── TargetSubpixelRefiner
│   └── TargetCodeDecoder
├── TargetDetectionService
│   └── TargetDetectionApiHandler     # status + preview
├── TargetDetectionPreview
├── MeasurementManager
│   └── GeometryMeasurementEngine V6
├── MeasurementApiHandler
├── FullCalibrationController
│   └── FullCalibrationApiHandler
├── ContinuousMeasurementController
│   └── ContinuousMeasurementApiHandler
├── RuntimeDiagnostics
│   └── RuntimeDiagnosticsApiHandler
└── ApiWsdlHandler
```

Les anciens composants de réglage caméra manuel, diagnostic JPEG HTTP, correction d'artefacts et contrôle de netteté ont été supprimés.

## Tracking haute précision

Le repère canonique reste :

```text
2560 × 1920
```

Le tracking suit automatiquement :

```text
SEARCH       2560×1920
  ↓
ZOOM_WIDE   1920×1440
  ↓
ZOOM_MEDIUM 1280×960
  ↓
ZOOM_FINE   1024×768
  ↓
PRECISE      800×600 natif
```

Chaque niveau produit une image 800×600. Le gain de précision vient du crop natif OV5640 : en PRECISE, un pixel de sortie correspond à un pixel de la fenêtre native.

Le changement de viewport est appliqué après le traitement du cycle courant afin de ne jamais changer de repère au milieu d'une mesure.

Après un changement de viewport :

- l'historique temporel de détection est réinitialisé ;
- la stabilisation des mesures est réinitialisée ;
- la prochaine capture utilise le nouveau cadrage.

## Pipeline image

```text
OV5640 JPEG
   ↓
JpegDiagnostic
   ↓
JpegFilteredDiagnostic
   └── décodage JPEG -> luminance 8 bits
   ↓
TargetDetector V6.1
   ↓
TargetObservation locale
   ↓
CameraViewportController::to_reference()
   ↓
TargetObservation 2560×1920
   ↓
GeometryMeasurementEngine V6
```

Il n'existe plus de contrôle de netteté ni de correction d'artefacts dans cette chaîne.

## Détection V6.1

Le raffinement subpixel ajuste les quatre bords de la cible.

Pour préserver la répétabilité observée en V5 :

- largeur et hauteur principales proviennent de la séparation des droites robustes opposées ;
- les séparations locales V6 sont conservées comme contrôle indépendant ;
- un désaccord entre mesure globale et locale augmente l'incertitude, mais ne déplace pas directement la dimension mesurée.

Les valeurs d'incertitude servent ensuite à pondérer les estimations de distance.

## Calibration automatique

La calibration est un workflow autonome :

```text
tracking jusqu'à PRECISE
→ recherche automatique du profil optique
→ verrouillage exposition/gain
→ acquisition de N échantillons
→ calcul robuste fx/fy
→ stockage calibration
```

Le `CameraSettingsController` reste volontairement interne.

Le score optique est basé sur :

- qualité du décodage de cible ;
- RMS subpixel ;
- incertitude largeur/hauteur ;
- P10/P90 ;
- contraste ;
- pixels écrêtés noirs/blancs ;
- pénalité de gain.

Il ne dépend plus d'un score de netteté séparé.

## Mode continu

`ContinuousMeasurementController` orchestre :

```text
REQUEST_CAPTURE
→ WAIT_CAPTURE
→ DECODE
→ DETECT
→ COMPUTE       uniquement en PRECISE
→ WAIT_INTERVAL
```

SEARCH et les zooms intermédiaires ne publient pas de mesure géométrique.

Le statut publie :

```text
capture_ms
decode_ms
detect_ms
compute_ms
processing_ms
orchestration_ms
cycle_ms
```

## API

Contrat : `GET /api/wsdl`, version **22**.

### Lecture seule

```text
GET /api/runtime/status
GET /api/camera/viewport
GET /tracking/status
GET /target/status
GET /target/preview.bmp
```

### Calibration

```text
GET /calibration/full/start
GET /calibration/full/status
GET /calibration/full/preview.bmp
GET /calibration/full/cancel
```

### Mesure / continu

```text
GET /measurement/status
GET /continuous/start
GET /continuous/status
GET /continuous/stop
```

Les routes historiques du `MeasurementApiHandler` peuvent rester disponibles tant qu'elles servent aux validations internes, mais elles ne font plus partie de l'interface PC normale.

## Mémoire image

Le système ne décode jamais une image gris 2560×1920 complète.

```text
grayscale 800×600    ≈ 0,48 Mo
grayscale 2560×1920  ≈ 4,92 Mo
```

La sortie de travail 800×600 protège la marge PSRAM tout en permettant la pleine résolution locale en PRECISE.

## Validation après nettoyage

1. compiler le firmware ;
2. flasher sans modifier la position caméra/cible ;
3. lancer une calibration complète ;
4. vérifier la progression automatique jusqu'à PRECISE ;
5. prendre une mesure ponctuelle depuis la console ;
6. lancer le suivi continu ;
7. vérifier les transitions SEARCH → ZOOM → PRECISE ;
8. vérifier la stabilité V6.1 et les temps capture/decode/detect/compute ;
9. masquer puis retrouver la cible pour valider le retour SEARCH ;
10. comparer la dispersion avec les mesures précédentes.
