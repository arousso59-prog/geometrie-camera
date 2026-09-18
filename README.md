# geometrie-camera

Capteur optique de géométrie automobile basé sur ESP32-S3 + OV5640.

## Architecture actuelle

Le projet est volontairement recentré sur trois usages :

1. **Calibration automatique**
2. **Prise de mesure haute précision**
3. **Suivi continu**

Le suivi haute précision est permanent. Il n'existe plus de mode utilisateur permettant de l'activer ou de le désactiver.

```text
OV5640 JPEG
   ↓
acquisition interne
   ↓
décodage JPEG → niveaux de gris
   ↓
TargetDetector V6.1
   ↓
tracking automatique
SEARCH → ZOOM_WIDE → ZOOM_MEDIUM → ZOOM_FINE → PRECISE
   ↓
mesure uniquement sur PRECISE natif 800×600
   ↓
GeometryMeasurementEngine V6
```

Les anciens chemins manuels de capture, filtrage, recherche de cible et réglage caméra ont été supprimés.

## Traitement image

Le pipeline de mesure ne contient plus :

- de correction d'artefacts JPEG ;
- de contrôle de netteté ;
- de recapture conditionnée par un score de netteté.

Le JPEG est uniquement décodé en image 8 bits nécessaire à la détection. La V6.1 conserve la séparation des droites robustes pour les dimensions principales et utilise les séparations locales comme information de confiance/incertitude.

## Réglage caméra automatique

Les réglages OV5640 ne sont plus exposés dans l'API utilisateur.

Pendant une calibration, l'ESP recherche automatiquement un profil optique à partir de :

- la qualité de détection ;
- le RMS subpixel ;
- l'incertitude largeur/hauteur ;
- la luminance P10/P90 ;
- le contraste ;
- l'écrêtage noir/blanc ;
- le niveau de gain.

Le contrôleur de réglages caméra reste donc une brique interne à la calibration.

## API opérationnelle

La référence du contrat HTTP est :

```text
GET /api/wsdl
```

Version actuelle : **26**.

### Diagnostic lecture seule

```text
GET /api/runtime/status
GET /api/camera/viewport
GET /tracking/status
GET /target/status
GET /target/preview.bmp
```

### Calibration

```text
GET /calibration/full/start?distance_mm=<mm>&target_size_mm=<mm>&samples=<3..20>&force=<0|1>
GET /calibration/full/status
GET /calibration/full/preview.bmp
GET /calibration/full/cancel
```

### Mesure

```text
GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/compute
GET /measurement/status
```

### Mode continu

```text
GET /continuous/start?interval_ms=<200..10000>
GET /continuous/status
GET /continuous/stop
```

La V23 ajoute un diagnostic parallèle qui ne modifie pas la mesure officielle V6.1 :

- dimensions par coins ;
- V5 : séparation directe des droites robustes ;
- V6 : séparation locale robuste ;
- V6.1 : valeur officielle ;
- distance Z diagnostique de chaque méthode ;
- RMS et gradient de chacun des quatre bords.

Le statut continu expose les temps :

```text
capture_ms
decode_ms
detect_ms
compute_ms
processing_ms
orchestration_ms
cycle_ms
```

## Matériel validé

- GOOUUU ESP32-S3-CAM V1.5 ;
- ESP32-S3 N16R8 ;
- OV5640 ;
- PSRAM 8 Mo octal ;
- repère capteur 2560×1920 ;
- sortie de travail 800×600 ;
- XCLK 8 MHz.

## Principe de mesure

SEARCH et les trois niveaux de zoom servent uniquement à trouver et centrer la cible. Aucune mesure géométrique n'est publiée depuis ces images redimensionnées.

Une mesure n'est calculée qu'après verrouillage en **PRECISE natif 800×600**. La calibration reste référencée dans le repère 2560×1920.

Les modes supprimés ne doivent pas être réintroduits comme dépendances des trois workflows opérationnels.


## Recherche optique ciblée

Le réglage n'utilise plus de valeur nominale supposée. Il part des valeurs réellement choisies par l'OV5640 en AEC/AGC automatique, les valide en manuel, puis ne cherche que très localement autour de ce point.

Après validation du verrouillage manuel :

- exposition : meilleur point ±10 ;
- gain : meilleur point ±1 ;
- contraste : 0, +1, -1, puis +2 uniquement si +1 progresse ;
- luminosité : 0, -1, +1.

Les images saturées, noires, sans contraste ou sans détection subpixel valide ont un score nul.

## Optimisation optique V24

Après l'optimisation exposition/gain, la calibration affine maintenant aussi les traitements capteur :

- contraste : baseline 0, puis +1 et -1 ; +2 uniquement si +1 améliore le score ;
- luminosité : baseline 0, puis -1 et +1 ;
- exposition/gain restent verrouillés sur leur meilleur couple pendant ces essais ;
- le profil final conserve exposition, gain, contraste et luminosité.

Le même score métrologique est utilisé pour tous les candidats : RMS, sigma, gradients horizontaux/verticaux, équilibre des gradients, P10/P90, contraste utile, clipping et qualité de détection.


## Verrouillage caméra V25

Le réglage optique ne suppose plus une exposition/gain nominale.

1. La cible est verrouillée en PRECISE avec AEC/AGC automatiques.
2. Quatre images laissent l'OV5640 stabiliser son exposition.
3. L'ESP lit directement les registres matériels OV5640 :
   - exposition : 0x3500..0x3502 ;
   - gain : 0x350A..0x350B.
4. Ces valeurs réelles sont appliquées en mode manuel.
5. Une capture valide obligatoirement le verrouillage manuel.
6. Si la capture devient blanche/noire, perd la cible ou obtient un score nul, le verrouillage est refusé.
7. L'ESP réactive alors AEC/AGC auto, attend trois images de récupération et réalise la calibration en mode auto.

Si le verrouillage manuel est valide, l'affinage reste volontairement local : exposition ±10, gain ±1, puis contraste et luminosité.


## Optimisation finale caméra V26

Chaque candidat manuel est maintenant observé sur **2 images consécutives** avec exactement le même réglage.

Un candidat est rejeté si l'une des deux images perd la cible ou ne fournit pas de raffinement subpixel valide. Sinon, le score instantané moyen est pénalisé par la dispersion entre les deux mesures de largeur et de hauteur :

- `repW` : sigma inter-image de la largeur ;
- `repH` : sigma inter-image de la hauteur ;
- le plus mauvais des deux axes pilote la pénalité.

Le but n'est plus de choisir l'image ponctuellement la plus flatteuse, mais le profil donnant la meilleure **répétabilité métrologique**. À qualité proche, la pénalité de gain déjà présente dans le score favorise naturellement le gain le plus faible.

Cette V26 est destinée à figer le réglage caméra. Si les essais sont meilleurs ou équivalents à la V25, aucune nouvelle modification caméra n'est prévue.
