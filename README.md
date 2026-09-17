# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- OV5640 confirmé (`0x5640`) ;
- PSRAM 8 Mo octal ;
- résolution exposée jusqu'à 2560×1920 ;
- XCLK actuel : 8 MHz.

## Pipeline actuel

```text
OV5640 JPEG
   ↓
JpegDiagnostic
   ↓
ImageSharpnessEvaluator (mode continu)
   ├── ROI autour de la dernière cible détectée
   ├── décodage JPEG 1/4
   ├── score sur les 20 % de contours les plus forts
   └── recapture seulement si flou important
   ↓
JpegFilteredDiagnostic V2
   ↓
JpegArtifactCorrector
   ↓
TargetDetector V5.5
   ├── décodage du motif 7x7
   ├── raffinage des coins
   └── continuité temporelle position/taille pour stabiliser la sélection
   ↓
GeometryMeasurementEngine V3
   ├── distance fusionnée à partir des deux axes du carré
   ├── centre projectif par intersection des diagonales
   ├── X/Y/Z + bearing
   └── pose homographique validée séparément
```

## Calibration et distance

La cible par défaut mesure 50 mm. La focale est calibrée à partir d'une distance connue :

```text
capture
→ filtre
→ /target/detect
→ /measurement/calibrate?distance_mm=1000&target_size_mm=50
```

Une calibration valide est verrouillée. Pour la remplacer volontairement :

```text
GET /measurement/calibrate?distance_mm=1000&target_size_mm=50&force=1
```

La profondeur V3 conserve les deux estimations indépendantes :

```text
z_from_width_mm  = fx × target_size_mm / largeur_px
z_from_height_mm = fy × target_size_mm / hauteur_px
```

Quand elles sont proches, `z_mm` utilise leur moyenne harmonique, ce qui revient à moyenner les tailles apparentes normalisées et évite le basculement brutal de l'ancien `min()`. Si leur écart augmente, le calcul revient progressivement vers la plus petite estimation, qui est en général la moins affectée par le raccourcissement dû à l'inclinaison de la cible. La transition commence à 3 % d'écart et devient complète à 15 %.

Le centre utilisé pour `x_mm`, `y_mm`, `bearing_yaw_deg` et `bearing_pitch_deg` n'est plus le centre grossier du candidat : il est calculé à partir de l'intersection des diagonales formées par les quatre coins raffinés. `pose_z_mm` reste exposé comme contrôle indépendant de cohérence de la pose homographique.

## Réglages caméra

```text
GET /api/camera/settings
GET /api/camera/settings/set?resolution=800x600
GET /api/camera/settings/set?monochrome=1
GET /api/camera/settings/set?monochrome=0
```

`monochrome=1` active l'effet grayscale de l'OV5640 à chaud. Le framebuffer reste en JPEG : ce réglage ne change ni le `pixel_format`, ni le pipeline de capture/détection. Le changement réel de `pixel_format` reste un réglage de démarrage nécessitant recompilation/reflash.

Le mode continu utilise la résolution caméra active. `800x600` reste pratique pour les essais rapides ; une évolution haute résolution + ROI est prévue pour augmenter la précision sans traiter toute l'image.

## Détection cible V5.5

Le détecteur conserve le score du décodage du motif comme critère de validité, mais utilise maintenant une cohérence temporelle pour choisir entre plusieurs candidats valides. Quand une cible valide a déjà été trouvée, les candidats proches en position et de taille cohérente reçoivent un bonus de sélection. Les sauts importants de position ou de taille sont pénalisés, sans empêcher une vraie réacquisition si le nouveau candidat est nettement meilleur.

Après trois détections manquées consécutives, le suivi temporel est remis à zéro afin de permettre une réacquisition libre.

Dans `/target/preview.bmp` :

```text
cadre plein      = cible validée par le décodage du motif
cadre pointillé  = meilleur candidat localisé mais rejeté par la validation
```

Le cadre pointillé permet donc de distinguer un échec de localisation d'un candidat bien repéré mais non reconnu comme cible.

## Mode continu avec contrôle de netteté ciblé V2

Le démarrage est interdit sans calibration valide :

```text
GET /continuous/start?interval_ms=1000
GET /continuous/status
GET /continuous/stop
```

Le cycle est :

```text
capture
→ contrôle netteté dans la ROI de la dernière cible connue
   ├── aucune ROI connue → pas de rejet, passage direct au filtre
   ├── cible manifestement floue → recapture immédiate, maximum 2 fois
   └── OK
→ filtre
→ détection
→ mise à jour ROI + référence netteté
→ mesure
→ cycle suivant
```

La netteté ne dépend donc plus du décor complet. Sur un mur uniforme, seul le voisinage de la cible influence le score.

La ROI vaut maintenant environ **2,5 fois la taille détectée de la cible**, avec un minimum de `48×48 px` dans l'image source. Le JPEG est décodé à `1/4` pour conserver assez de détails quand la cible devient petite.

Le score ne moyenne plus tous les pixels de la ROI : il utilise les **20 % de réponses Laplaciennes les plus fortes**, qui correspondent principalement aux transitions noir/blanc du motif. Cela réduit fortement l'influence du fond uniforme.

La référence de netteté est mise à jour uniquement après une détection valide. Une nouvelle valeur est d'abord limitée à ±15 % de la référence précédente, puis intégrée lentement (`7/8` ancienne référence + `1/8` nouvelle valeur bornée). Une capture n'est recapturée que si son score tombe sous **45 %** de cette référence. Si la troisième capture reste faible, le pipeline continue quand même pour ne pas se bloquer.

Les essais V1 ont montré que le mini-décodage de netteté 1/4 coûte encore environ `315–320 ms`. Cette V2 vise d'abord à vérifier que les recaptures deviennent réellement utiles ; ensuite, si le principe est validé, le double décodage JPEG sera un axe prioritaire d'optimisation.

`/continuous/status` expose les temps détaillés :

```text
timing.capture_ms
timing.sharpness_ms
timing.filter_ms
timing.detect_ms
timing.compute_ms
timing.processing_ms
timing.orchestration_ms
timing.cycle_ms
```

`processing_ms` est la somme des cinq traitements chronométrés. `orchestration_ms` correspond au temps mural restant dans le cycle entre ces étapes et les passages de boucle ESPHome ; ce temps est interne à l'ESP32 et ne correspond pas à un traitement réalisé par le PC.

et les informations de netteté :

```text
sharpness.score_x100
sharpness.reference_x100
sharpness.ok
sharpness.capture_retries
sharpness.blur_retry_count
sharpness.roi_active
sharpness.roi_x
sharpness.roi_y
sharpness.roi_width
sharpness.roi_height
```

## Interface Web ESPHome

La page principale garde les **dernières valeurs valides** de distance, X/Y/Z, angles et qualité même si un cycle courant ne retrouve pas la cible. La ligne `03 Cible actuelle` indique séparément si la dernière détection a réussi.

Les timings capture/netteté/filtre/détection/calcul et les compteurs de recapture sont affichés sous les mesures principales.

## API actuelle

```text
GET /api/wsdl
GET /api/runtime/status
GET /api/camera/settings
GET /api/camera/settings/set?<parametres>&resolution=<optionnel>&monochrome=<0|1>
GET /diagnostic-jpeg/capture?resolution=<optionnel>
GET /diagnostic-jpeg/status
GET /diagnostic-jpeg/image.jpg
GET /diagnostic-jpeg/filter
GET /diagnostic-jpeg/filter-status
GET /diagnostic-jpeg/filtered.bmp
GET /target/detect
GET /target/status
GET /target/preview.bmp
GET /measurement/config
GET /measurement/config/set?target_size_mm=<mm>
GET /measurement/calibrate?distance_mm=<mm>&target_size_mm=<optionnel>&force=<0|1>
GET /measurement/compute
GET /measurement/status
GET /continuous/start?interval_ms=<optionnel>
GET /continuous/stop
GET /continuous/status
```

`/api/wsdl` est la référence du contrat HTTP. Version actuelle : **20**.

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Étape actuelle

1. compiler/flasher le firmware courant ;
2. refaire une calibration à distance connue avec la cible la plus frontale possible ;
3. laisser la cible parfaitement immobile et comparer la dispersion de `z_mm`, `z_from_width_mm`, `z_from_height_mm` et `pose_z_mm` ;
4. incliner légèrement la cible puis vérifier que `z_mm` reste continu quand largeur et hauteur échangent leur rôle dominant ;
5. vérifier en parallèle la stabilité de X/Y et des bearing grâce au centre calculé par les quatre coins ;
6. seulement après cette validation, décider si un filtrage temporel léger est encore nécessaire.
