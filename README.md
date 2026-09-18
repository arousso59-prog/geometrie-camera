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

Version actuelle : **30**.

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

La calibration ne balaie plus les cinq niveaux AE automatiques ni les bornes extrêmes d'exposition/gain.

Le profil final restant en AEC=OFF / AGC=OFF, la recherche est centrée sur les paramètres réellement utilisés en mesure :

- centre initial : dernier bon profil manuel si disponible, sinon exposition 400 / gain 7 ;
- exposition : ±160, ±80, ±40, ±20 ;
- gain : ±4, ±2, ±1 ;
- 15 captures de réglage au maximum.

Le score privilégie RMS faible, sigma faible et gradients horizontaux/verticaux forts et équilibrés.


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

Si le verrouillage manuel est valide, l'affinage est volontairement local : exposition ±20 puis ±10, gain ±1, puis contraste et luminosité.


## Configuration figée V27

La partie caméra est figée sur la stratégie V25 validée expérimentalement :

- PRECISE natif 800×600 ;
- AEC/AGC automatiques pendant la stabilisation initiale ;
- lecture des registres réels OV5640 ;
- validation du verrouillage manuel ;
- affinage local exposition/gain puis contraste/luminosité ;
- retour automatique en AEC/AGC auto si le verrouillage manuel dégrade l'image.

La tentative V26 basée sur deux images par candidat a été abandonnée car elle a dégradé la répétabilité sur l'essai réel.

La méthode de mesure opérationnelle est également figée :

- **V6.1** pour les dimensions et la pondération de distance ;
- **stabilisation robuste sur 5 mesures** dans `MeasurementManager` ;
- V5 et V6 restent calculées uniquement comme diagnostics comparatifs ;
- la pose/homographie ne pilote jamais la distance principale.

Cette combinaison constitue désormais la référence avant le travail spécifique sur yaw/pitch/roll.


## Pose V2 V28

La distance et le réglage caméra restent strictement figés :

- caméra : stratégie V25 validée ;
- distance : V6.1-robust5 ;
- V5/V6 : diagnostics uniquement.

La V28 modifie uniquement l'orientation de la cible.

L'ancienne pose V1 par homographie est conservée comme diagnostic. La nouvelle pose V2 :

1. récupère les quatre droites subpixel ajustées directement sur les profils haut/droite/bas/gauche ;
2. prend la pose homographique V1 comme initialisation ;
3. fixe la translation sur X/Y/Z déjà produits par V6.1-robust5 ;
4. optimise uniquement la rotation du carré 3D ;
5. minimise prioritairement la distance entre les bords projetés et les quatre droites mesurées ;
6. utilise les coins seulement comme faible régularisation ;
7. rejette V2 si l'erreur de reprojection devient excessive ;
8. conserve V1 en repli sans jamais modifier la distance.

Le roll est stabilisé avec une périodicité de 180 degrés. Yaw/pitch sont stabilisés via la normale 3D moyenne plutôt qu'en moyennant directement les angles.


## Pose V3 V29 — motif complet 7×7

La V29 ne modifie ni la caméra ni la distance :

- caméra : stratégie V25 figée ;
- distance : V6.1-robust5 figée ;
- V5/V6 : diagnostics uniquement.

La pose V3 ajoute un raffinement spécifique à l'orientation :

1. le décodeur connaît le motif 7×7 exact ;
2. toutes les frontières internes noir/blanc exploitables sont recherchées ;
3. chaque frontière est localisée en subpixel par recherche du gradient orienté ;
4. les coordonnées sont remappées dans l'orientation canonique du motif ;
5. une homographie robuste est ajustée sur l'ensemble de ces transitions ;
6. les résidus sont repondérés de façon robuste et les outliers sont rejetés ;
7. cette homographie produit une orientation initiale indépendante des quatre coins externes ;
8. X/Y/Z restent ceux de V6.1-robust5 ;
9. seule la rotation est réoptimisée sur une grille de 25 points du plan cible ;
10. le dernier pas angulaire vaut 0,0015° = 0,09 minute d'arc.

V3 est rejetée si l'homographie du motif ou la reprojection de pose devient incohérente. Le repli est alors V2, puis V1.

La stabilisation sur 5 mesures ne mélange jamais les méthodes : si V3 est majoritaire, seuls les échantillons V3 participent à la moyenne de normale/roll.


## Capture pipelinée V30

La V30 optimise uniquement l'orchestration du mode continu. Aucun paramètre caméra ou algorithme métrologique n'est modifié.

ESPHome pré-acquiert naturellement la frame suivante dans sa tâche caméra. Jusqu'à V29, cette frame était toujours jetée puis une nouvelle capture était demandée. Cette stratégie garantissait une image postérieure à la requête, mais empêchait tout chevauchement entre acquisition et traitement.

En V30 :

- après démarrage ou changement de viewport : comportement historique, frame pré-acquise purgée ;
- en régime stable avec intervalle <= 1500 ms : la frame séquentielle déjà en cours d'acquisition pendant le décodage/détection du cycle précédent est acceptée ;
- avec un intervalle > 1500 ms : retour automatique à une capture strictement postérieure à la requête ;
- calibration : comportement historique inchangé, donc purge systématique après réglages caméra/ROI.

Le but est de chevaucher le temps capteur avec le décodage et la détection sans changer XCLK, JPEG, exposition, gain, contraste, luminosité, V6.1 ou la pose.

`/continuous/status` expose `timing.capture_pipelined` pour vérifier quels cycles utilisent cette optimisation.
