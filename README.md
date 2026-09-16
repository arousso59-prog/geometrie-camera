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
TargetDetector V5
   ├── TargetCandidateFinder
   └── TargetCodeDecoder
   ↓
TargetObservation
   ↓
distance / orientation / géométrie
```

Le correcteur V3 supprime la grande majorité des artefacts verts/noirs sans appliquer de flou global.

La V5 sépare désormais deux problèmes auparavant mélangés :

1. **retrouver la petite cible dans toute l'image** ;
2. **lire le code 7×7 une fois la zone localisée**.

`TargetCandidateFinder` réduit l'image à environ 400 px maximum, applique un seuillage local puis recherche des composantes sombres quasi carrées. Les lignes parasites et objets très allongés sont rejetés. Pour chaque composante retenue, quatre coins approximatifs sont estimés.

`TargetCodeDecoder` revient ensuite sur l'image pleine résolution, projette le motif 7×7 dans le quadrilatère candidat, teste plusieurs petits ajustements de taille et les quatre orientations logiques, puis valide contraste, cadre noir et fond extérieur.

Cette organisation doit être plus robuste aux petites rotations/perspectives et beaucoup moins coûteuse que l'ancien balayage exhaustif du motif 7×7 sur toute l'image.

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

Les routes de cible restent inchangées avec la V5 :

- `/target/detect` traite la dernière image déjà filtrée ;
- `/target/status` relit le dernier résultat ;
- `/target/preview.bmp` affiche une miniature annotée du meilleur résultat/candidat.

## Étape actuelle

Valider la V5 sur la cible réelle :

1. capture JPEG en 1600×1200 ;
2. filtre V3 ;
3. `/target/detect` ;
4. vérification visuelle avec `/target/preview.bmp` ;
5. déplacer la cible à plusieurs endroits ;
6. tester plusieurs distances et rotations ;
7. une fois la localisation stable, passer à la distance puis à l'orientation fine.

Les optimisations de pipeline restent volontairement reportées : après la première détection globale, la future voie rapide travaillera sur une ROI autour de la dernière cible connue et reviendra à la recherche globale en cas de perte.
