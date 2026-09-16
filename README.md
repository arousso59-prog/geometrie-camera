# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- capteur confirmé par PID : OV5640 (`0x5640`) ;
- PSRAM : 8 Mo octal ;
- résolution maximale exposée par ESPHome 2026.7.3 : 2560×1920 ;
- XCLK retenu actuellement : 8 MHz.

Brochage caméra :

```text
SIOD  GPIO4
SIOC  GPIO5
VSYNC GPIO6
HREF  GPIO7
XCLK  GPIO15
Y2    GPIO11
Y3    GPIO9
Y4    GPIO8
Y5    GPIO10
Y6    GPIO12
Y7    GPIO18
Y8    GPIO17
Y9    GPIO16
PCLK  GPIO13
```

## Voie image retenue

```text
OV5640 JPEG
   ↓
frame fraîche
   ↓
décodage grayscale 8 bits
   ↓
JpegArtifactCorrector V3
   ↓
buffer grayscale corrigé
   ↓
TargetDetectionService
   ↓
TargetDetector V5.3
   ├── TargetCandidateFinder V5.2
   └── TargetCodeDecoder V5.3
   ↓
TargetObservation
   ↓
distance / orientation / géométrie
```

Le correcteur V3 supprime la grande majorité des artefacts verts/noirs sans appliquer de flou global.

La V5 sépare deux problèmes auparavant mélangés :

1. **retrouver la petite cible dans toute l'image** ;
2. **lire le code 7×7 une fois la zone localisée**.

`TargetCandidateFinder` réduit l'image à environ 320 px maximum, applique un seuillage local puis recherche des composantes sombres quasi carrées. Les lignes parasites et objets très allongés sont rejetés. Les 8 meilleurs candidats sont conservés dans un buffer persistant afin de limiter le coût CPU et la pile du handler HTTP.

`TargetCodeDecoder` revient ensuite sur l'image pleine résolution, projette le motif 7×7 dans le quadrilatère candidat, teste plusieurs petits ajustements de taille et les quatre orientations logiques, puis valide contraste, cadre noir et fond extérieur. Depuis la V5.3, le score final est accepté à partir de `0.82`, après passage de ces gardes structurelles indépendantes.

Sur les essais réels en 1600×1200, la localisation/détection complète est désormais de l'ordre de 0,16 s. Le principal goulot de performance est maintenant le filtre JPEG/correction d'artefacts, autour de 4 s au total sur les mesures actuelles.

## Architecture

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

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
GET /target/preview.bmp
```

Les routes de cible restent inchangées avec la V5.3 :

- `/target/detect` traite la dernière image déjà filtrée ;
- `/target/status` relit le dernier résultat ;
- `/target/preview.bmp` affiche une miniature annotée du meilleur résultat/candidat.

`/api/wsdl` est la référence du contrat HTTP compilé.

## Étape actuelle

Valider la V5.3 sur la cible réelle :

1. capture JPEG en 1600×1200 ;
2. filtre V3 ;
3. `/target/detect` ;
4. vérifier que la cible réelle passe maintenant `target_found=true` ;
5. retirer complètement la cible et vérifier `target_found=false` ;
6. déplacer la cible à plusieurs endroits ;
7. tester plusieurs distances et rotations ;
8. une fois cette robustesse confirmée, passer à la distance puis à l'orientation fine.

La prochaine optimisation de performance doit porter sur `JpegFilteredDiagnostic` / `JpegArtifactCorrector`, pas sur `TargetDetector`. La future voie rapide utilisera ensuite une ROI autour de la dernière cible connue et reviendra à la recherche globale en cas de perte.
